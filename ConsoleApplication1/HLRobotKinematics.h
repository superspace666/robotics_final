#pragma once
#include "eigen3/Eigen/Dense"
#include <vector>
#include <iostream>

#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_
const double PI = 3.1415926;
const double degree2rad = 180 / PI;
const double L1 = 491;
const double L2 = 450;
const double L3 = 450;
const double L4 = 84;

namespace HLRobot
{
	//Init
	void robotInit();

	//Inverse kinematics solution
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll);
	void GetJointAngles(double& angle1, double& angle2, double& angle3, double& angle4, double& angle5, double& angle6);

	//Forward kinematics solution
	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6);
	void GetJointEndPos(double& x, double& y, double& z, double& yaw, double& pitch, double& roll);

	//Inverse kinematics and Forward kinematics method function
	void robotBackward(const double* TransVector, double* theta);
	void robotForward(const double* q, double* TransVector);

	//Jocobin solution
	void GetSpacialJacobian(double* joint, double* jacobian);
	void GetToolSpeed(double* joint_last, double* joint_present, double* xyz_vel);

}

#endif
