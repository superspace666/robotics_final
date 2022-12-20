#include <iostream>
#include <fstream>
#include "MotionPlan.h"
// #include "Srobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include "eigen3/Eigen/Dense"
#include "math.h"
// #include "HLrobotconfig.h"
#include "HLRobotKinematics.h"
#include "vector"

// #define WRITE_CARTESIAN
#ifndef WRITE_CARTESIAN
#define WRITE_JOINT
#endif

using namespace std;
using namespace Eigen;

HLMotionPlan::HLMotionPlan()
{
	ctrlPoints.clear();

	mSampleTime = 0.001;
}

HLMotionPlan::~HLMotionPlan() {}

void HLMotionPlan::SetSampleTime(const double& sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
	cout << "[Planning]\tsample time is set to: " << sampleTime << "s" << endl;
}

void HLMotionPlan::SetCtrlPoints(const vector<Matrix<double, 6, 1>>& points, const vector<int>& plan_method, const vector<VelConfig>& vel_config)
{
	ctrlPoints = points;
	planMethod = plan_method;
	velConfig = vel_config;
}


void HLMotionPlan::PlanInCartesian(const PosStruct& startPoint, const PosStruct& endPoint, const VelConfig& vel_config, ofstream& file)
{
	Vector3d startPos(startPoint.x, startPoint.y, startPoint.z);
	Vector3d startAng(startPoint.yaw, startPoint.pitch, startPoint.roll);
	Vector3d endPos(endPoint.x, endPoint.y, endPoint.z);
	Vector3d endAng(endPoint.yaw, endPoint.pitch, endPoint.roll);
	vector<Vector3d> wayPos;
	vector<Vector3d> wayAng;
	// triangle linear velocity
	if ((endPos - startPos).norm() < vel_config.Vel * vel_config.Vel * (vel_config.Acc + vel_config.Dec) / (2 * vel_config.Acc * vel_config.Dec))
		PlanTriangle(vel_config.Acc, vel_config.Dec, startPos, endPos, wayPos);
	// trapezoidal linear velocity
	else
		PlanTrapezoidal(vel_config.Vel, vel_config.Acc, vel_config.Dec, startPos, endPos, wayPos);
	// triangle angular velocity
	if ((endAng - startAng).norm() < vel_config.AngVel * vel_config.AngVel * (vel_config.AngAcc + vel_config.AngDec) / (2 * vel_config.AngAcc * vel_config.AngDec))
		PlanTriangle(vel_config.AngAcc, vel_config.AngDec, startAng, endAng, wayAng);
	// trapezoidal angular velocity
	else
		PlanTrapezoidal(vel_config.AngVel, vel_config.AngAcc, vel_config.AngDec, startAng, endAng, wayAng);
	Matrix<double, 6, 1> wayPoint;
	// bool config[3] = { 1, 0, 0 };
	double theta[6] = { 0.0 };
	for (int i = 0; i < max(wayPos.size(), wayAng.size()); i++)
	{
		wayPoint.topLeftCorner(3, 1) = (i < wayPos.size()) ? wayPos[i] : wayPos[wayPos.size() - 1];
		wayPoint.bottomLeftCorner(3, 1) = (i < wayAng.size()) ? wayAng[i] : wayAng[wayAng.size() - 1];
#ifdef WRITE_CARTESIAN
		file << wayPoint[0] << "  " << wayPoint[1] << "  " << wayPoint[2] << "  " << wayPoint[3] << "  " << wayPoint[4] << "  " << wayPoint[5] << endl;
#endif
#ifdef WRITE_JOINT
		HLRobot::SetRobotEndPos(wayPoint[0], wayPoint[1], wayPoint[2], wayPoint[3], wayPoint[4], wayPoint[5]);
		HLRobot::GetJointAngles(theta[0], theta[1], theta[2], theta[3], theta[4], theta[5]);
		file << theta[0] << "  " << theta[1] << "  " << theta[2] << "  "
			<< theta[3] << "  " << theta[4] << "  " << theta[5] << endl;
#endif
	}
}

void HLMotionPlan::GetPlanPoints(const char* filename)
{
	ofstream outfile;
	outfile.open(filename);
	bool config[3] = { 1, 0, 0 };
	for (int i = 0; i < ctrlPoints.size() - 1; i++)
	{
		// cout << "[Planning]\t" << to_string(((double)(i) / (double)(ctrlPoints.size() - 1)) * 100) << "% finished  \r";
		/*HLRobot::SetRobotEndPos(ctrlPoints[i].x, ctrlPoints[i].y, ctrlPoints[i].z, ctrlPoints[i].yaw, ctrlPoints[i].pitch, ctrlPoints[i].roll);
		HLRobot::GetJointAngles(startJoint(0), startJoint(1), startJoint(2), startJoint(3), startJoint(4), startJoint(5));
		HLRobot::SetRobotEndPos(ctrlPoints[i + 1].x, ctrlPoints[i + 1].y, ctrlPoints[i + 1].z, ctrlPoints[i + 1].yaw, ctrlPoints[i + 1].pitch, ctrlPoints[i + 1].roll);
		HLRobot::GetJointAngles(endJoint(0), endJoint(1), endJoint(2), endJoint(3), endJoint(4), endJoint(5));*/
#ifdef WRITE_CARTESIAN
		outfile << ctrlPoints[i].x << "  " << ctrlPoints[i].y << "  " << ctrlPoints[i].z
			<< "  " << ctrlPoints[i].yaw << "  " << ctrlPoints[i].pitch << "  " << ctrlPoints[i].roll << endl;
#endif
#ifdef WRITE_JOINT
		outfile << ctrlPoints[i][0] << "  " << ctrlPoints[i][1] << "  " << ctrlPoints[i][2] << "  "
			<< ctrlPoints[i][3] << "  " << ctrlPoints[i][4] << "  " << ctrlPoints[i][5] << endl;
#endif
		if (planMethod[i] == PLAN_IN_CARTESIAN) cout << "test" << endl;
		// PlanInCartesian(ctrlPoints[i], ctrlPoints[i + 1], velConfig[i], outfile);
		else if (planMethod[i] == PLAN_IN_JOINT)
			PlanInJoint(ctrlPoints[i], ctrlPoints[i + 1], velConfig[i], outfile);
		// cout << "[Planning]\t" << to_string(((double)(i + 1) / (double)(ctrlPoints.size() - 1)) * 100) << "% finished  \r";
	}
	cout << endl;
	/*Vector3d epos, eang;
	epos << ctrlPoints[ctrlPoints.size() - 1].x,
		ctrlPoints[ctrlPoints.size() - 1].y,
		ctrlPoints[ctrlPoints.size() - 1].z;
	eang << ctrlPoints[ctrlPoints.size() - 1].yaw,
		ctrlPoints[ctrlPoints.size() - 1].pitch,
		ctrlPoints[ctrlPoints.size() - 1].roll;*/
#ifdef WRITE_CARTESIAN
	outfile << epos.x() << "  " << epos.y() << "  " << epos.z() << "  "
		<< eang.x() << "  " << eang.y() << "  " << eang.z() << endl;
#endif
#ifdef WRITE_JOINT
	outfile << ctrlPoints[ctrlPoints.size() - 1][0] << "  " << ctrlPoints[ctrlPoints.size() - 1][1] << "  " << ctrlPoints[ctrlPoints.size() - 1][2] << "  "
		<< ctrlPoints[ctrlPoints.size() - 1][3] << "  " << ctrlPoints[ctrlPoints.size() - 1][4] << "  " << ctrlPoints[ctrlPoints.size() - 1][5] << endl;
#endif
	outfile.close();
}

void HLMotionPlan::PlanTriangle(const double& mAcc, const double& mDec, const Vector3d& startPoint, const Vector3d& endPoint, vector<Vector3d>& wayPoints)
{
	// the result doesn't include the start and end point
	Vector3d direction;
	if ((endPoint - startPoint).norm() < 1e-2)
		direction << 0.0, 0.0, 0.0;
	else
		direction = (endPoint - startPoint) / (endPoint - startPoint).norm();
	double time1 = sqrt(2 * (endPoint - startPoint).norm() * mDec / (mAcc + mDec) / mAcc);
	double time2 = sqrt(2 * (endPoint - startPoint).norm() * mAcc / (mAcc + mDec) / mDec);
	double time = 0.0;
	while (time < time1 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(startPoint + direction * 0.5 * mAcc * time * time);
	}
	Vector3d midPoint = startPoint + direction * 0.5 * mAcc * time1 * time1;
	wayPoints.push_back(midPoint);
	double vel = mAcc * time1;
	time = 0.0;
	while (time < time2 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(midPoint + direction * (vel * time - 0.5 * mDec * time * time));
	}
}

void HLMotionPlan::PlanTrapezoidal(const double& mVel, const double& mAcc, const double& mDec, const Vector3d& startPoint, const Vector3d& endPoint, vector<Vector3d>& wayPoints)
{
	// the result doesn't include the start and end point
	Vector3d direction = (endPoint - startPoint) / (endPoint - startPoint).norm();
	double time1 = mVel / mAcc;
	double time3 = mVel / mDec;
	double time2 = ((endPoint - startPoint).norm() - 0.5 * (mAcc * time1 * time1 + mDec * time3 * time3)) / mVel;
	double time = 0.0;
	while (time < time1 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(startPoint + direction * 0.5 * mAcc * time * time);
	}
	Vector3d midPoint = startPoint + direction * 0.5 * mAcc * time1 * time1;
	wayPoints.push_back(midPoint);
	time = 0.0;
	while (time < time2 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(midPoint + direction * mVel * time);
	}
	midPoint = midPoint + direction * mVel * time2;
	wayPoints.push_back(midPoint);
	time = 0.0;
	while (time < time3 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(midPoint + direction * (mVel * time - 0.5 * mDec * time * time));
	}
}

bool HLMotionPlan::JudgeLocation(const Matrix<double, 6, 1>& cur, const Matrix<double, 6, 1>& tar, double pos_eps)
{
	if ((cur - tar).norm() < pos_eps)
		return true;
	else
		return false;
}

void HLMotionPlan::PlanTrapezoidal(const double& mVel, const double& mAcc, const double& mDec, const double& start, const double& end, vector<double>& wayPoints)
{
	int dir = end > start ? 1 : -1;
	double time1 = abs(mAcc) < 1e-3 ? 0.0 : mVel / mAcc;
	double time3 = abs(mDec) < 1e-3 ? 0.0 : mVel / mDec;
	double time2 = abs(mVel) < 1e-3 ? 0.0 : (abs(end - start) - 0.5 * (mAcc * time1 * time1 + mDec * time3 * time3)) / mVel;
	double time = 0.0;
	while (time < time1 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(start + dir * 0.5 * mAcc * time * time);
	}
	double mid = start + dir * 0.5 * mAcc * time1 * time1;
	wayPoints.push_back(mid);
	time = 0.0;
	while (time < time2 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(mid + dir * mVel * time);
	}
	mid = mid + dir * mVel * time2;
	wayPoints.push_back(mid);
	time = 0.0;
	while (time < time3 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(mid + dir * (mVel * time - 0.5 * mDec * time * time));
	}
}

void HLMotionPlan::PlanTriangle(const double& mAcc, const double& mDec, const double& start, const double& end, vector<double>& wayPoints)
{
	int dir;
	if (abs(end - start) < 1e-3)
		dir = 0;
	else
		dir = end > start ? 1 : -1;
	double time1, time2;
	time1 = abs(mAcc) < 1e-3 ? 0 : sqrt(2 * abs(end - start) * mDec / (mAcc + mDec) / mAcc);
	time2 = abs(mDec) < 1e-3 ? 0 : sqrt(2 * abs(end - start) * mAcc / (mAcc + mDec) / mDec);
	double time = 0.0;
	while (time < time1 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(start + dir * 0.5 * mAcc * time * time);
	}
	double mid = start + dir * 0.5 * mAcc * time1 * time1;
	wayPoints.push_back(mid);
	double vel = time1 * mAcc;
	time = 0.0;
	while (time < time2 - mSampleTime)
	{
		time += mSampleTime;
		wayPoints.push_back(mid + dir * (vel * time - 0.5 * mDec * time * time));
	}
}

void HLMotionPlan::PlanInJoint(const Matrix<double, 6, 1>& startPoint, const Matrix<double, 6, 1>& endPoint, const VelConfig& vel_config, ofstream& file)
{
	int max_idx = -1;
	double max_dis = 0.0;
	for (int i = 0; i < 6; i++)
	{
		if (abs((endPoint(i) - startPoint(i))) > max_dis)
		{
			max_idx = i;
			max_dis = abs((endPoint(i) - startPoint(i)));
		}
	}
	vector<double> vel = { 0.0 };
	vector<double> acc = { 0.0 };
	vector<double> dec = { 0.0 };
	vector<vector<double>> wayPoints;
	vector<double> jointWayPoints;
	int max_len = 0;
	if (abs(endPoint(max_idx) - startPoint(max_idx)) < vel_config.JointVel * vel_config.JointVel * (vel_config.JointAcc + vel_config.JointDec) / (2 * vel_config.JointAcc * vel_config.JointDec))
	{
		double time1 = sqrt(2 * abs(endPoint(max_idx) - startPoint(max_idx)) * vel_config.JointDec / (vel_config.JointAcc + vel_config.JointDec) / vel_config.JointAcc);
		double time2 = sqrt(2 * abs(endPoint(max_idx) - startPoint(max_idx)) * vel_config.JointAcc / (vel_config.JointAcc + vel_config.JointDec) / vel_config.JointDec);
		vector<double> acc, dec;
		acc.clear(); dec.clear();
		for (int i = 0; i < 6; i++)
		{
			acc.push_back(2.0 * abs(endPoint(i) - startPoint(i)) / (time1 + time2) / time1);
			dec.push_back(2.0 * abs(endPoint(i) - startPoint(i)) / (time1 + time2) / time2);
		}
		for (int i = 0; i < 6; i++)
		{
			jointWayPoints.clear();
			PlanTriangle(acc[i], dec[i], startPoint(i), endPoint(i), jointWayPoints);
			wayPoints.push_back(jointWayPoints);
			max_len = jointWayPoints.size() > max_len ? jointWayPoints.size() : max_len;
		}
	}
	else
	{
		double time1 = vel_config.JointVel / vel_config.JointAcc;
		double time3 = vel_config.JointVel / vel_config.JointDec;
		double time2 = (abs(endPoint[max_idx] - startPoint[max_idx]) - 0.5 * (vel_config.JointAcc * time1 * time1 + vel_config.JointDec * time3 * time3)) / vel_config.JointVel;
		vector<double> acc, vel, dec;
		acc.clear(); vel.clear(); dec.clear();
		for (int i = 0; i < 6; i++)
		{
			acc.push_back(2.0 * abs(endPoint(i) - startPoint(i)) / (time1 + 2.0 * time2 + time3) / time1);
			vel.push_back(2.0 * abs(endPoint(i) - startPoint(i)) / (time1 + 2.0 * time2 + time3));
			dec.push_back(2.0 * abs(endPoint(i) - startPoint(i)) / (time1 + 2.0 * time2 + time3) / time3);
		}
		for (int i = 0; i < 6; i++)
		{
			jointWayPoints.clear();
			PlanTrapezoidal(vel[i], acc[i], dec[i], startPoint(i), endPoint(i), jointWayPoints);
			wayPoints.push_back(jointWayPoints);
			max_len = jointWayPoints.size() > max_len ? jointWayPoints.size() : max_len;
		}
	}
	for (int i = 0; i < 6; i++)
	{
		while (wayPoints[i].size() < max_len)
		{
			wayPoints[i].push_back(wayPoints[i][wayPoints[i].size() - 1]);
		}
	}
	double theta[6] = { 0.0 };
	for (int i = 0; i < max_len; i++)
	{
#ifdef WRITE_JOINT
		file << wayPoints[0][i] << "  " << wayPoints[1][i] << "  " << wayPoints[2][i] << "  "
			<< wayPoints[3][i] << "  " << wayPoints[4][i] << "  " << wayPoints[5][i] << endl;
#endif
#ifdef WRITE_CARTESIAN
		HLRobot::SetRobotJoint(wayPoints[0][i], wayPoints[1][i], wayPoints[2][i],
			wayPoints[3][i], wayPoints[4][i], wayPoints[5][i]);
		HLRobot::GetJointEndPos(theta[0], theta[1], theta[2], theta[3], theta[4], theta[5]);
		file << theta[0] << "  " << theta[1] << "  " << theta[2] << "  "
			<< theta[3] << "  " << theta[4] << "  " << theta[5] << endl;
#endif
	}
}

void HLMotionPlan::GetContinuousAng(vector<double>& theta)
{
	static double last[6] = { theta[0], theta[1], theta[2], theta[3], theta[4], theta[5] };
	for (int i = 0; i < 6; i++)
	{
		if (last[i] - theta[i] > 10) theta[i] += 360;
		else if (theta[i] - last[i] > 10) theta[i] -= 360;
		last[i] = theta[i];
	}
}