#include "HLrobotconfig.h"


using namespace std;
using namespace Eigen;

#define Delta 0.000001

namespace HLRobot
{
	//初始化TransMatrix
	double mTransMatrix[16] {0};


	Vector3d wz = { 0,0,1 };
	Vector3d wy = { 0,1,0 };
	Vector3d q0 = { 0,0,0 };
	Vector3d q2 = { 0,0,0.491 };
	Vector3d q3 = { 0,0,0.941 };
	Vector3d q5 = { 0,0,1.391 };
	Vector3d q6 = { 0,0,1.475 };
	Vector3d v0 = { 0,0,0 };
	Vector3d v2 = { -0.491,0,0 };
	Vector3d v3 = { -0.941,0,0 };
	Vector3d v5 = { -1.391,0,0 };

	Vector4d p0 = { 0,0,0,1 };

	Vector3d Omega[6] = { wz,wy,wy,wz,wy,wz };
	Vector3d Q[6] = { q0,q2,q3,q0,q5,q0 };
	Vector3d V[6] = { v0,v2,v3,v0,v5,v0 };
	Matrix4d Trans;
	Matrix4d g0;
	Matrix4d xi[6];

	//只使用一种姿态
	bool mConfig[3] = { 1, 1, 1 };

	void Matrix2double(void)
	{
		for (int i = 0; i < 16; i++)
			mTransMatrix[i] = Trans(i / 4, i % 4);
	}
	void double2Matrix(void)
	{
		for (int i = 0; i < 16; i++)
			Trans(i / 4, i % 4) = mTransMatrix[i];
	}

	void hat(Vector3d w, Matrix3d& output)
	{
		output <<
			0, -w(2), w(1),
			w(2), 0, -w(0),
			-w(1), w(0), 0;
	}

	void hat(Vector3d v, Vector3d w, Matrix4d& output)
	{
		Matrix3d temp;
		hat(w, temp);
		output <<
			temp, v,
			0, 0, 0, 1;
	}

	void Init(void)
	{
		g0 <<
			-1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, 1, 1.475,
			0, 0, 0, 1;

		double2Matrix();

		for (int i = 0; i < 6; i++)
			hat(V[i], Omega[i], xi[i]);
	}

	void EXP(Vector3d w, double th, Matrix3d& output)
	{
		double s = sin(th);
		double c = cos(th);
		double v = 1 - c;

		output <<
			w(0) * w(0) * v + c, w(0)* w(1)* v - w(2) * s, w(0)* w(2)* v + w(1) * s,
			w(0)* w(1)* v + w(2) * s, w(1)* w(1)* v + c, w(1)* w(2)* v - w(0) * s,
			w(0)* w(2)* v - w(1) * s, w(1)* w(2)* v + w(0) * s, w(2)* w(2)* v + c;
	}

	void Joint(Vector3d v, Vector3d w, double theta, Matrix4d& output)
	{
		Matrix3d Temp, I;
		EXP(w, theta, Temp);
		I = MatrixXd::Identity(3, 3);
		if (w.norm() <= Delta)
		{
			output <<
				I, v* theta,
				0, 0, 0, 1;
		}
		else
		{
			output <<
				Temp, (I - Temp)* w.cross(v) + w * w.dot(v) * theta,
				0, 0, 0, 1;
		}
	}


	void posture2zyz(Matrix4d g, double& x, double& y, double& z, double& yaw, double& pitch, double& roll)
	{
		pitch = acos(g(2, 2)) / PI * 180;
		double p_ = (pitch + 0.0001) / 180 * PI;
		yaw = asin(g(2, 1) / sin(p_)) / PI * 180;
		roll = asin(g(1, 2) / sin(p_)) / PI * 180;
		x = g(0, 3);
		y = g(1, 3);
		z = g(2, 3);
	}

	void zyz2posture(double x, double y, double z, double yaw, double pitch, double roll, Matrix4d& g)
	{
		double a, b, c;
		a = yaw / 180 * PI;
		b = pitch / 180 * PI;
		c = roll / 180 * PI;
		g <<
			cos(a) * cos(b) * cos(c) - sin(a) * sin(c), -cos(a) * cos(b) * sin(c) - sin(a) * cos(c), cos(a)* sin(b), x,
			sin(a)* cos(b)* cos(c) + cos(a) * sin(c), -sin(a) * cos(b) * sin(c) + cos(a) * cos(c), sin(a)* sin(b), y,
			-sin(b) * cos(c), sin(b)* sin(c), cos(b), z,
			0, 0, 0, 1;
	}

	void subproblem1(Vector3d omega, Vector3d p, Vector3d q, Vector3d r, double& angle1)
	{
		Vector3d u = p - r;
		Vector3d v = q - r;
		Vector3d u_ = u - omega.dot(u) * omega;
		Vector3d v_ = v - omega.dot(v) * omega;
		double t = omega.dot(u_.cross(v_))/(u_.dot(v_));
		angle1 = atan2(omega.dot(u_.cross(v_)), (u_.dot(v_))) / PI * 180;
	}

	void subproblem2(Vector3d omega1, Vector3d omega2, Vector3d p, Vector3d q, Vector3d r, double& angle1, double& angle2)
	{
		Vector3d u = p - r;
		Vector3d v = q - r;
		double alpha, beta, gamma;
		alpha = (omega1.dot(omega2) * (omega2.dot(u)) - omega1.dot(v)) / (pow((omega1.dot(omega2)), 2) - 1);
		beta = (omega1.dot(omega2) * (omega1.dot(v)) - omega2.dot(u)) / (pow((omega1.dot(omega2)), 2) - 1);
		Vector3d w = omega1.cross(omega2);
		gamma = -sqrt((u.dot(u) - pow(alpha, 2) - pow(beta, 2) - 2 * alpha * beta * omega1.dot(omega2)) / (w.dot(w)));
		Vector3d z = alpha * omega1 + beta * omega2 + gamma * (omega1.cross(omega2));
		Vector3d c = r + z;
		subproblem1(omega2, p, c, r, angle2);
		subproblem1(omega1, c, q, r, angle1);
	}

	void subproblem3(Vector3d omega, Vector3d p, Vector3d q, Vector3d r, double delta, double& angle1)
	{
		Vector3d u = p - r;
		Vector3d v = q - r;
		Vector3d u_ = u - omega.dot(u) * omega;
		Vector3d v_ = v - omega.dot(v) * omega;
		double delta_2 = pow(delta, 2) - pow(omega.dot(p - q), 2);
		double th0;
		double t = omega.dot(u_.cross(v_)) / (u_.dot(v_));
		th0 = atan2(omega.dot(u_.cross(v_)), (u_.dot(v_))) / PI * 180;
		t = (u_.dot(u_) + v_.dot(v_) - delta_2) / (2 * u_.norm() * v_.norm());
		angle1 = th0 - acos(t) / PI * 180;
	}

	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
		zyz2posture(x / 1000, y / 1000, z / 1000, yaw, pitch, roll, Trans);
		Matrix2double();
	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, double &angle5, double &angle6)
	{
		double angle[6] = {0};
		robotBackward(mTransMatrix, mConfig, angle);
		angle1 = angle[0];
		angle2 = angle[1];
		angle3 = angle[2];
		angle4 = angle[3];
		angle5 = angle[4];
		angle6 = angle[5];
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6)
	{
		double Theta[6] = { angle1 * PI / 180,angle2 * PI / 180, angle3 * PI / 180, angle4 * PI / 180, angle5 * PI / 180, angle6 * PI / 180 };
		double T[16];
		robotForward(Theta, T, mConfig);
	}

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll)
	{
		Vector4d Pos = Trans * p0;
		x = Pos(0);
		y = Pos(1);
		z = Pos(2);

		Matrix3d R = Trans.block<3, 3>(0, 0);

		double alpha, beta, gamma;

		beta = atan2(sqrt(pow(R(2, 0), 2) + pow(R(2, 1), 2)), R(2, 2));
		double s = sin(beta);
		if (abs(s) <= Delta)
		{
			alpha = atan2(R(1, 0), R(1, 1));
			gamma = 0;
		}
		else
		{
			alpha = atan2(R(1, 2) / s, R(0, 2) / s);
			gamma = atan2(R(2, 1) / s, -R(2, 0) / s);
		}

		yaw = alpha;
		pitch = beta;
		roll = gamma;
	}


	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

				config[3]：姿态，六轴机器人对应有8种姿态（即对应的逆运动学8个解），为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

	OUTPUTS:    theta[6] 6个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	
	void robotBackward(const double* TransVector, bool* mconfig, double* theta)
	{
		double2Matrix();
		Matrix4d gd = Trans;
		Matrix4d g1 = gd * g0.inverse();
		Vector3d qw = q5;
		Vector3d pb = q2;
		Vector4d qw_, pb_;
		qw_ << qw, 1;
		pb_ << pb, 1;
		Vector4d w = g1 * qw_ - pb_;
		double delta = w.norm();
		subproblem3(Omega[2], qw, pb, Q[2], delta, theta[2]);
		Matrix4d e3;
		Joint(V[2], Omega[2], theta[2] / 180 * PI, e3);
		Vector4d qw_2 = e3 * qw_;
		subproblem2(Omega[0], Omega[1], qw_2.block<3, 1>(0, 0), (g1 * qw_).block<3, 1>(0, 0), Q[1], theta[0], theta[1]);
		Matrix4d e1,e2;
		Joint(V[0], Omega[0], theta[0] / 180 * PI, e1);
		Joint(V[1], Omega[1], theta[1] / 180 * PI, e2);
		Matrix4d g2 = e3.inverse() * e2.inverse() * e1.inverse() * g1;
		Vector3d p = q6;
		Vector4d p_;
		p_ << p, 1;
		subproblem2(Omega[3], Omega[4], p, (g2 * p_).block<3, 1>(0, 0), Q[4], theta[3], theta[4]);
		Matrix4d e4, e5;
		Joint(V[3], Omega[3], theta[3] / 180 * PI, e4);
		Joint(V[4], Omega[4], theta[4] / 180 * PI, e5);
		Matrix4d g3 = e5.inverse() * e4.inverse() * g2;
		p = { 1,0,0 };
		p_ << p, 1;
		subproblem1(Omega[5], p, (g3 * p_).block<3, 1>(0, 0), q0, theta[5]);
		
	}

	/********************************************************************
	ABSTRACT:	机器人正运动学
	
	INPUTS:		q[6]: 6个关节角, 单位为弧度
	
	OUTPUTS:	config[3]：姿态，六轴机器人对应有8种姿态，为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米
	
	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, double* TransVector, bool* mconfig)
	{
		Trans = MatrixXd::Identity(4, 4);
		Matrix4d Temp;

		for (int i = 0; i < 6; i++)
		{
			Joint(V[i], Omega[i], q[i], Temp);
			Trans = Trans * Temp;
		}

		Trans = Trans * g0;
		Matrix2double();
		TransVector = mTransMatrix;
	}
}
