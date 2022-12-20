#include "HLRobotKinematics.h"
#include <cmath>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

namespace HLRobot
{
	double mTransMatrix[16]{ 0 };
	double q[6]{ 0 };
	Matrix4d gst0;
	Vector3d Rx, Ry, Rz;
	Vector3d q1, q2, q3, q4, q5, q6;
	Vector4d q1_bar, q2_bar, q3_bar, q4_bar, q5_bar, q6_bar;
	Vector6d kesi1, kesi2, kesi3, kesi4, kesi5, kesi6;
	Matrix4d e1, e2, e3, e4, e5, e6;


	Matrix3d getso3(Vector3d w) {
		Matrix3d so3;
		so3 << 0, -w(2), w(1),
			w(2), 0, -w(0),
			-w(1), w(0), 0;
		return so3;
	}

	Matrix3d getSO3(Vector3d w, double theta)
	{
		double theta_rad = theta / degree2rad;
		Matrix3d so3 = getso3(w);
		Matrix3d SO3 = Matrix3d::Identity() + sin(theta_rad) * so3 + (1 - cos(theta_rad)) * so3 * so3;
		return SO3;
	}

	Matrix4d getSE3(Vector3d v, Vector3d w, double theta)
	{
		Matrix3d SO3 = getSO3(w, theta);
		Vector3d p = (Matrix3d::Identity() - SO3) * (w.cross(v)) + theta * w.transpose() * v * w;
		Matrix4d SE3 = Matrix4d::Identity();
		SE3.block<3, 3>(0, 0) = SO3;
		SE3.block<3, 1>(0, 3) = p;
		return SE3;
	}

	Matrix4d inverseSE3(Matrix4d gst)
	{
		Matrix4d gst_inv = Matrix4d::Identity();
		Vector3d p = gst.block<3, 1>(0, 3);
		Matrix3d rotation_matrix = gst.block<3, 3>(0, 0);
		gst_inv.block<3, 3>(0, 0) = rotation_matrix.transpose();
		gst_inv.block<3, 1>(0, 3) = -rotation_matrix.transpose() * p;
		return gst_inv;
	}

	double SubProblem1(Vector3d r, Vector3d p, Vector3d q, Vector3d w)
	{
		//输入：r-轴上一点 | p-转动起始点 | q-转动终点 | w-转轴方向向量
		//输出：旋转角度
		Vector3d u = p - r;
		Vector3d v = q - r;
		Vector3d u_ = u - w * w.transpose() * u;
		Vector3d v_ = v - w * w.transpose() * v;
		double theta_rad = atan2(w.transpose() * (u_.cross(v_)), u_.transpose() * v_);
		return theta_rad * degree2rad;
	}

	void SubProblem2(Vector3d r, Vector3d p, Vector3d q, Vector3d w1, Vector3d w2, double& theta1, double& theta2)
	{
		//输入：r-旋转轴交点 | p-转动起始点 | q-转动终点 | w1、2-转轴方向向量 | theta1、2-求解的角度
		//输出：无
		Vector3d u = p - r;
		Vector3d v = q - r;
		double w12 = w1.transpose() * w2;
		double w2u = w2.transpose() * u;
		double w1v = w1.transpose() * v;
		double alpha = (w12 * w2u - w1v) / (w12 * w12 - 1);
		double beta = (w12 * w1v - w2u) / (w12 * w12 - 1);
		double gamma_square = (u.dot(u) - alpha * alpha - beta * beta - 2 * alpha * beta * w12) / (w1.cross(w2)).dot((w1.cross(w2)));
		//cout << gamma_square << "    ...   ";
		if (gamma_square > 0)
		{
			double gamma = sqrt(gamma_square);
			Vector3d z1 = alpha * w1 + beta * w2 - gamma * (w1.cross(w2));
			//Vector3d z2 = alpha * w1 + beta * w2 + gamma * (w1.cross(w2));
			Vector3d c1 = z1 + r;
			//Vector3d c2 = z2 + r;
			theta2 = SubProblem1(r, p, c1, w2);
			theta1 = SubProblem1(r, c1, q, w1);

			//确保角度连续
			/*if (theta1 > 90)
				theta1 = theta1 - 180;
			else if (theta1 < -90)
				theta1 = theta1 + 180;*/
				/*
				if (theta2 > 90)
					theta2 = theta2 - 180;
				else if (theta2 < -90)
					theta2 = theta2 + 180;*/
		}
	}

	double SubProblem3(Vector3d r, Vector3d p, Vector3d q, Vector3d w, double delta)
	{
		Vector3d u = p - r;
		Vector3d v = q - r;
		Vector3d u_ = u - w.transpose() * u * w;
		Vector3d v_ = v - w.transpose() * v * w;
		double theta0 = atan2(w.transpose() * (u_.cross(v_)), u_.transpose() * v_);
		double temp = (w.transpose() * (p - q));
		double delta_square = delta * delta - temp * temp;
		double phi = acos((u_.dot(u_) + v_.dot(v_) - delta_square) / (2 * u_.norm() * v_.norm()));
		if (u_.norm() * v_.norm() < 0.0000000001)
			return 0;
		else
			return theta0 * degree2rad - phi * degree2rad;
	}

	void robotInit()
	{
		gst0 << -1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, 1, L1 + L2 + L3 + L4,
			0, 0, 0, 1;
		Rx << 1, 0, 0;
		Ry << 0, 1, 0;
		Rz << 0, 0, 1;
		q1 << 0, 0, 1;
		q2 << 0, 0, L1;
		q3 << 0, 0, L1 + L2;
		q4 << 0, 0, 1;
		q5 << 0, 0, L1 + L2 + L3;
		q6 << 0, 0, L1 + L2 + L3 + L4;
		q1_bar << 0, 0, 1, 1;
		q2_bar << 0, 0, L1, 1;
		q3_bar << 0, 0, L1 + L2, 1;
		q4_bar << 0, 0, 1, 1;
		q5_bar << 0, 0, L1 + L2 + L3, 1;
		q6_bar << 0, 0, L1 + L2 + L3 + L4, 1;

		kesi1 << -Rz.cross(q1), Rz;
		kesi2 << -Ry.cross(q2), Ry;
		kesi3 << -Ry.cross(q3), Ry;
		kesi4 << -Rz.cross(q4), Rz;
		kesi5 << -Ry.cross(q5), Ry;
		kesi6 << -Rz.cross(q6), Rz;
		cout << "[RobotInit]\tRobot Initialized." << endl;
	}

	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
		Vector3d p;
		p << x, y, z;
		Matrix3d rotation_matrix = Matrix3d::Identity();
		AngleAxisd Rz_(AngleAxisd(yaw / degree2rad, Vector3d::UnitZ()));
		AngleAxisd Ry_(AngleAxisd(pitch / degree2rad, Vector3d::UnitY()));
		AngleAxisd Rz__(AngleAxisd(roll / degree2rad, Vector3d::UnitZ()));
		rotation_matrix = Rz_ * Ry_ * Rz__;

		Matrix4d gst = Matrix4d::Identity();
		gst.block<3, 3>(0, 0) = rotation_matrix;
		gst.block<3, 1>(0, 3) = p;

		for (int i = 0; i < 16; i++)
		{
			mTransMatrix[i] = gst(i / 4, i % 4);
		}
	}

	void GetJointAngles(double& angle1, double& angle2, double& angle3, double& angle4, double& angle5, double& angle6)
	{
		double joint[6]{ 0 };
		robotBackward(mTransMatrix, joint);
		angle1 = joint[0];
		angle2 = joint[1];
		angle3 = joint[2];
		angle4 = joint[3];
		angle5 = joint[4];
		angle6 = joint[5];
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6)
	{
		q[0] = angle1;
		q[1] = angle2;
		q[2] = angle3;
		q[3] = angle4;
		q[4] = angle5;
		q[5] = angle6;
	}

	void GetJointEndPos(double& x, double& y, double& z, double& yaw, double& pitch, double& roll)
	{
		// robotInit();
		double foward_TransMatrix[16]{ 0 };
		robotForward(q, foward_TransMatrix);
		x = foward_TransMatrix[3];
		y = foward_TransMatrix[7];
		z = foward_TransMatrix[11];
		Matrix3d rotation_matrix;
		rotation_matrix << foward_TransMatrix[0], foward_TransMatrix[1], foward_TransMatrix[2],
			foward_TransMatrix[4], foward_TransMatrix[5], foward_TransMatrix[6],
			foward_TransMatrix[8], foward_TransMatrix[9], foward_TransMatrix[10];
		Vector3d eulerAngle = rotation_matrix.eulerAngles(2, 1, 2);
		roll = eulerAngle(2) * degree2rad;
		pitch = eulerAngle(1) * degree2rad;
		yaw = eulerAngle(0) * degree2rad;

	}

	void GetToolSpeed(double* joint_last, double* joint_present, double* xyz_speed)
	{
		Vector6d joint_speed;
		joint_speed << joint_present[0] - joint_last[0],
			joint_present[1] - joint_last[1],
			joint_present[2] - joint_last[2],
			joint_present[3] - joint_last[3],
			joint_present[4] - joint_last[4],
			joint_present[5] - joint_last[5];
		joint_speed = joint_speed / 0.001;	//除以采样时间
		double jacobian[36];
		GetSpacialJacobian(joint_present, jacobian);
		Matrix6d spacial_Jacobian;
		for (int i = 0; i < 16; i++)
		{
			spacial_Jacobian(i / 6, i % 6) = jacobian[i];
		}
		Vector6d Vst;
		Vst = spacial_Jacobian * joint_speed;
		xyz_speed[0] = Vst(0);
		xyz_speed[1] = Vst(1);
		xyz_speed[2] = Vst(2);
	}

	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

	OUTPUTS:    joint[6] 6个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	void robotBackward(const double* TransVector, double* joint)
	{
		// robotInit();
		Matrix4d gd;
		for (int i = 0; i < 16; i++)
		{
			gd(i / 4, i % 4) = TransVector[i];
		}

		Matrix4d gst1;
		gst1 = gd * inverseSE3(gst0);
		Vector4d p1, p2, p3, p;
		p1 << 0, 0, L1 + L2 + L3, 1;
		p2 << 0, 0, L1, 1;
		p3 << 0, 0, L1 + L2 + L3 + L4, 1;
		p << 1, 0, 0, 1;
		//利用 subproblem3 求解 theta3
		double delta1 = (gst1 * p1 - p2).norm();
		Vector3d r3(0, 0, L1 + L2);
		joint[2] = SubProblem3(r3, p1.head(3), p2.head(3), Ry, delta1);
		//利用 subproblem2 求解 theta1&2
		double th1, th2;
		Vector3d r12(0, 0, L1);
		Matrix4d exp_kesi3_theta3 = getSE3(-Ry.cross(r3), Ry, joint[2]);
		SubProblem2(r12, (exp_kesi3_theta3 * p1).head(3), (gst1 * p1).head(3), Rz, Ry, th1, th2);
		joint[0] = th1;
		joint[1] = th2;
		//利用 subproblem2 求解 theta4&5
		Matrix4d exp_kesi1_theta1 = getSE3(-Rz.cross(r12), Rz, joint[0]);
		Matrix4d exp_kesi2_theta2 = getSE3(-Ry.cross(r12), Ry, joint[1]);
		Matrix4d inv_1 = inverseSE3(exp_kesi1_theta1);
		Matrix4d inv_2 = inverseSE3(exp_kesi2_theta2);
		Matrix4d inv_3 = inverseSE3(exp_kesi3_theta3);
		Matrix4d gst2 = inv_3 * inv_2 * inv_1 * gst1;
		double th4, th5;
		SubProblem2(p1.head(3), p3.head(3), (gst2 * p3).head(3), Rz, Ry, th4, th5);
		joint[3] = th4;
		joint[4] = th5;
		//利用 subproblem1 求解theta6
		Vector3d r5(0, 0, L1 + L2 + L3);
		Matrix4d exp_kesi4_theta4 = getSE3(-Rz.cross(r12), Rz, joint[3]);
		Matrix4d exp_kesi5_theta5 = getSE3(-Ry.cross(r5), Ry, joint[4]);
		Matrix4d inv_4 = inverseSE3(exp_kesi4_theta4);
		Matrix4d inv_5 = inverseSE3(exp_kesi5_theta5);
		joint[5] = SubProblem1(p2.head(3), p.head(3), (inv_5 * inv_4 * gst2 * p).head(3), Rz);
	}

	/********************************************************************
	ABSTRACT:	机器人正运动学

	INPUTS:		q[6]: 6个关节角, 单位为弧度

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米

	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, double* TransVector)
	{
		// robotInit();
		Matrix4d gst;
		e1 = getSE3(kesi1.head(3), kesi1.tail(3), q[0]);
		e2 = getSE3(kesi2.head(3), kesi2.tail(3), q[1]);
		e3 = getSE3(kesi3.head(3), kesi3.tail(3), q[2]);
		e4 = getSE3(kesi4.head(3), kesi4.tail(3), q[3]);
		e5 = getSE3(kesi5.head(3), kesi5.tail(3), q[4]);
		e6 = getSE3(kesi6.head(3), kesi6.tail(3), q[5]);
		gst = e1 * e2 * e3 * e4 * e5 * e6 * gst0;
		for (int i = 0; i < 16; i++)
		{
			TransVector[i] = gst(i / 4, i % 4);
		}

	}

	/********************************************************************
	ABSTRACT:	Spacial Jacobian

	INPUTS:		q[6]: 6个关节角, 单位为弧度

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米

	RETURN:		<none>
	***********************************************************************/
	void GetSpacialJacobian(double* joint, double* jacobian)
	{
		// robotInit();
		Matrix3d joint0_SO3, joint1_SO3, joint2_SO3, joint3_SO3, joint4_SO3;
		joint0_SO3 = getSO3(Rz, joint[0]);
		joint1_SO3 = getSO3(Ry, joint[1]);
		joint2_SO3 = getSO3(Ry, joint[2]);
		joint3_SO3 = getSO3(Rz, joint[3]);
		joint4_SO3 = getSO3(Ry, joint[4]);

		Matrix4d joint0_SE3, joint1_SE3, joint2_SE3, joint3_SE3;
		joint0_SE3 = getSE3(-Rz.cross(q2), Rz, joint[0]);
		joint1_SE3 = getSE3(-Ry.cross(q2), Ry, joint[1]);
		joint2_SE3 = getSE3(-Ry.cross(q5), Ry, joint[2]);
		joint3_SE3 = getSE3(-Rz.cross(q5), Rz, joint[3]);

		Vector3d w1_prime, w2_prime, w3_prime, w4_prime, w5_prime, w6_prime;
		w1_prime << 0, 0, 1;
		w2_prime << -sin(joint[0]), cos(joint[0]), 0;
		w3_prime = joint0_SO3 * joint1_SO3 * Ry;
		w4_prime = joint0_SO3 * joint1_SO3 * joint2_SO3 * Rz;
		w5_prime = joint0_SO3 * joint1_SO3 * joint2_SO3 * joint3_SO3 * Ry;
		w6_prime = joint0_SO3 * joint1_SO3 * joint2_SO3 * joint3_SO3 * joint4_SO3 * Rz;

		Vector3d q1_prime, q2_prime, q3_prime, qw_prime;
		Vector4d q3_hat_prime, qw_hat_prime;
		q1_prime << 0, 0, L1;
		q2_prime << 0, 0, L1;
		q3_hat_prime = joint0_SE3 * joint1_SE3 * q3_bar;
		q3_prime << q3_hat_prime(0), q3_hat_prime(1), q3_hat_prime(2);
		qw_hat_prime = joint0_SE3 * joint1_SE3 * joint2_SE3 * joint3_SE3 * q5_bar;
		qw_prime << qw_hat_prime(0), qw_hat_prime(1), qw_hat_prime(2);

		Vector6d kesi1_prime, kesi2_prime, kesi3_prime, kesi4_prime, kesi5_prime, kesi6_prime;
		kesi1_prime << -w1_prime.cross(q1_prime), w1_prime;
		kesi2_prime << -w2_prime.cross(q2_prime), w2_prime;
		kesi3_prime << -w3_prime.cross(q3_prime), w3_prime;
		kesi4_prime << -w4_prime.cross(qw_prime), w4_prime;
		kesi5_prime << -w5_prime.cross(qw_prime), w5_prime;
		kesi6_prime << -w6_prime.cross(qw_prime), w6_prime;

		Matrix6d mJst;
		mJst << kesi1_prime, kesi2_prime, kesi3_prime, kesi4_prime, kesi5_prime, kesi6_prime;
		for (int i = 0; i < 36; i++)
		{
			jacobian[i] = mJst(i / 6, i % 6);
		}
	}
}
