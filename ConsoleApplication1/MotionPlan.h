#pragma once
#include <vector>
#include "eigen3/Eigen/Core"
#include "iostream"
using namespace std;

#define PLAN_IN_CARTESIAN 1
#define PLAN_IN_JOINT 2

using namespace Eigen;

struct PosStruct
{
	double x;				// mm
	double y;				// mm
	double z;				// mm
	double yaw;				// deg
	double pitch;			// deg
	double roll;			// deg
};

struct VelConfig
{
	double Vel;
	double Acc;
	double Dec;
	double AngVel;
	double AngAcc;
	double AngDec;
	double JointVel;
	double JointAcc;
	double JointDec;
};

class HLMotionPlan
{
public:
	HLMotionPlan();
	~HLMotionPlan();

	void SetSampleTime(const double& sampleTime = 0.001);
	void SetCtrlPoints(const vector<Matrix<double, 6, 1>>& points, const vector<int>& plan_method, const vector<VelConfig>& vel_config);
	void GetPlanPoints(const char* filename);
	static bool JudgeLocation(const Matrix<double, 6, 1>& cur, const Matrix<double, 6, 1>& tar, double pos_eps);
private:
	void GetContinuousAng(vector<double>& wp);
	void PlanInCartesian(const PosStruct& startPoint, const PosStruct& endPoint, const VelConfig& vel_config, ofstream& file);
	void PlanTrapezoidal(const double& mVel, const double& mAcc, const double& mDec, const Vector3d& startPoint, const Vector3d& endPoint, vector<Vector3d>& wayPoints);
	void PlanTrapezoidal(const double& mVel, const double& mAcc, const double& mDec, const double& start, const double& end, vector<double>& wayPoints);
	void PlanTriangle(const double& mAcc, const double& mDec, const Vector3d& startPoint, const Vector3d& endPoint, vector<Vector3d>& wayPoints);
	void PlanTriangle(const double& mAcc, const double& mDec, const double& start, const double& end, vector<double>& wayPoints);
	void PlanInJoint(const Matrix<double, 6, 1>& startPoint, const Matrix<double, 6, 1>& endPoint, const VelConfig& vel_config, ofstream& file);
	vector<Matrix<double, 6, 1>> ctrlPoints;				// way points in cartesian frame
	vector<int> planMethod;
	vector<VelConfig> velConfig;
	double mSampleTime;							// sample time, sencond, sample time in the robot is 0.001s
	//double mVel;								// max linear velocity, mm/s
	//double mAcc;								// max linear acceleration, mm/s/s
	//double mDec;								// max linear deceleration, mm/s/s
	//double mAngVel;								// max angular velocity, deg/s
	//double mAngAcc;								// max angular acceleration, deg/s/s
	//double mAngDec;								// max angular deceleration, deg/s/
	//double mJointVel;
	//double mJointAcc;
	//double mJointDec;
};