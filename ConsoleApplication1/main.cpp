#include <iostream>
#include <winsock.h>
#include <conio.h>
#include <vector>
#include "ftp/FtpControl.h"
#include "eigen3/Eigen/Core"
#include "MotionPlan.h"
#include "RobotCommunication.h"
#include "HLRobotKinematics.h"
// #include "CalVelocity.h"

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib, "WS2_32.lib")

using namespace std;

int main()
{
	string filename = "data.txt";
	string fileip = "192.168.10.101";


	Matrix<double, 6, 1> Point1, Point2, Point3, Point4, Point5, Point6, Point7, Point8, Point9, Point10;
	Matrix<double, 6, 1> SPoint1, SPoint2, SPoint3, SPoint4, SPoint5, SPoint6, SPoint7, SPoint8, SPoint9, SPoint10;

	//课设好使的点
	Point1 << 6.925, 32.778, 104.538, -0.563, 44.840, 157.393;
	SPoint1 << 6.925, 24.869, 102.116, -0.483, 55.171, 157.270;
	Point2 << -0.060, 31.162, 107.313, -0.732, 40.568, 156.021;
	SPoint2 << -0.060, 22.201, 104.425, -0.601, 52.415, 155.831;
	Point3 << -7.461, 31.563, 106.597, -0.533, 40.827, 148.469;
	SPoint3 << -7.461, 22.680, 103.719, -0.439, 52.588, 148.332;
	Point4 << -11.701, 28.711, 111.246, -0.433, 39.007, 144.162;
	SPoint4 << -11.701, 19.343, 108.234, -0.348, 51.386, 144.043;
	Point5 << -3.758, 27.776, 112.590, -0.661, 38.644, 152.284;
	SPoint5 << -3.758, 18.275, 109.510, -0.529, 51.224, 152.100;

	Point6 << 4.315, 27.682, 112.436, -0.872, 38.963, 160.518;
	SPoint6 << 3.848, 18.393, 109.251, -0.691, 51.431, 159.804;
	Point7 << 11.962, 28.875, 110.763, -1.047, 39.527, 168.293;
	SPoint7 << 11.764, 19.437, 107.941, -0.844, 51.782, 167.811;
	Point8 << 9.032, 25.389, 117.162, -1.042, 36.581, 165.394;
	SPoint8 << 9.032, 15.234, 114.047, -0.813, 49.850, 165.081;

	Point9 << 0.155, 24.600, 117.978, -0.806, 36.467, 156.329;
	SPoint9 << 0.155, 14.387, 114.764, -0.626, 49.894, 156.084;
	Point10 << -8.243, 24.964, 117.259, -0.559, 36.761, 147.731;
	SPoint10 << -8.243, 14.865, 114.048, -0.436, 50.070, 147.563;

	// HLRobot::robotInit();

	HLMotionPlan motion_plan;

	Matrix<double, 6, 1> homePoint, safePoint, warePoint, endPoint, startPoint, throwPoint;
	homePoint << -3.941, 21.234, 106.803, -0.528, 50.974, 171.917;
	safePoint << SPoint1; // 3.960, 21.946, 119.563, -0.884, 37.569, 180.188;
	// midPoint << -3.942, 22.921, -2.414, -0.549, 86.504, 171.948;
	warePoint << Point1;// 3.960, 24.284, 119.923, -0.950, 34.872, 180.261;
	startPoint << -3.932, 10.0, 133.893, -2.474, 70, 180.477;
	throwPoint << -3.93425, 10, 113.13, -1.98773, 48.76, 180.837;
	endPoint << -3.941, 10.0, 70, -0.528, -5, 181.917;
	VelConfig low_vel = { 50, 100, 50, 10, 20, 10, 50, 50, 50 };
	VelConfig mid_vel = { 50, 100, 50, 10, 20, 10, 100, 100, 100 };
	VelConfig high_vel = { 50, 100, 50, 10, 20, 10, 180, 1500, 1500 };
	vector<Matrix<double, 6, 1>> ctrlPoints = 
	{	homePoint,  SPoint1, Point1, SPoint1, startPoint, endPoint, homePoint, 
		SPoint2, Point2, SPoint2, startPoint, endPoint, homePoint,
		SPoint3, Point3, SPoint3, startPoint, endPoint, homePoint,
		SPoint4, Point4, SPoint4, startPoint, endPoint, homePoint,
		SPoint5, Point5, SPoint5, startPoint, endPoint, homePoint,
		SPoint6, Point6, SPoint6, startPoint, endPoint, homePoint,
		SPoint7, Point7, SPoint7, startPoint, endPoint, homePoint,
		SPoint8, Point8, SPoint8, startPoint, endPoint, homePoint,
		SPoint9, Point9, SPoint9, startPoint, endPoint, homePoint,
		SPoint10, Point10, SPoint10, startPoint, endPoint, homePoint
	};
	vector<int> plan_method = 
	{   
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, 
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT,
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT,
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT,
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT,
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT,
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT,
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT,
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT,
		PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT, PLAN_IN_JOINT
	};
	vector<VelConfig> vel_config = 
	{  
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel, 
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel,
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel,
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel,
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel,
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel,
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel,
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel,
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel,
		mid_vel, low_vel, low_vel, mid_vel, high_vel, mid_vel
	};

	motion_plan.SetSampleTime();
	motion_plan.SetCtrlPoints(ctrlPoints, plan_method, vel_config);
	motion_plan.GetPlanPoints(filename.data());

	RobotCommunication robo_com;
	robo_com.initialize();

	FtpControl::Upload(fileip.data(), "data", "data.txt", "data.txt");

	// HLRobot::SetRobotEndPos(homePoint.x, homePoint.y, homePoint.z, homePoint.yaw, homePoint.pitch, homePoint.roll);
	// HLRobot::GetJointAngles()

	robo_com.communicate("[5# LocationJ p1]", "[Define]", 100);
	// robo_com.communicate("[8# Robot.Frame 1,1]", "[Frame]", 200);
	robo_com.communicate("[5# p1=-3.941,21.234,106.803,-0.528,50.974,171.917]", "[Define]", 100);
	robo_com.communicate("[6# Move.Joint p1]", "[Move]", 2000);
	robo_com.communicate("[7# PPB.Enable 1,1]", "[PPB_Enable]", 200);
	robo_com.communicate("[9# PPB.ReadFile 1,/data/data.txt]", "[ReadFile]", 500);
	robo_com.communicate("[10# PPB.J2StartPoint 1,0,1]", "[ToStartPoint]", 1000);
	robo_com.communicate("[6# PPB.Run 1]", "[Run]", 100);

	// string test = robo_com.communicate("[10# Robot.Where 1]", "[Where]", 100); // the first read is empty

	// Sleep(700);
	// robo_com.communicate("[1# IO.Set DOUT(20101),0]", "[IO]", 50, true);
	// Sleep(3050);
	// robo_com.communicate("[1# IO.Set DOUT(20101),0]", "[IO]", 50, true);

	string fdb;
	Matrix<double, 6, 1> fdb_pos;
	double dis = 0.0;
	double Kt = 0.0;
	Matrix<double, 6, 1> last_pos;
	last_pos.setZero();
	while (true)
	{
		fdb = robo_com.communicate("[10# Robot.WhereAngle 1]", "[Where]", 0, false);
		if (fdb.length() != 96) continue;
		cout << fdb << endl;
		fdb_pos = RobotCommunication::decode_coord(fdb);
		dis = ((throwPoint(2) - fdb_pos(2)) + (throwPoint(4) - fdb_pos(4)));
		if (abs(fdb_pos(1) - 10) < 0.3 &&			// Joint 2 = 10.0 deg
			last_pos(2) - fdb_pos(2) > 5 &&		// Joint 3 decrease
			last_pos(4) - fdb_pos(4) > 5 &&		// Joint 5 decrease
			dis < 0 && dis > -35)						// threshold
		{
			Sleep((unsigned long int)(Kt * (35 - abs(dis))));
			robo_com.communicate("[1# IO.Set DOUT(20101),0]", "[IO]", 0, false);
			cout << fdb << endl;
			// robo_com.communicate("[1# IO.Set DOUT(20103),1]", "[IO]", 100, false);
		}
		else
		{
			robo_com.communicate("[1# IO.Set DOUT(20101),1]", "[IO]", 0, false);
		}
		last_pos = fdb_pos;
		//cout << tmp.x << "\t" << tmp.y << "\t" << tmp.z << "\t" << tmp.yaw << "\t" << tmp.pitch << "\t" << tmp.roll << endl;
	}


	return 0;
}