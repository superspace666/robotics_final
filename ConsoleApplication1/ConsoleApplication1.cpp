// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。

#include <string>
#include <fstream>
#include<iostream>
#include<winsock.h>
#include <conio.h>
#include "HLrobotconfig.h"
#include "MotionPlan.h"
#include "ftp/FtpControl.h"
#include "eigen3/Eigen/Core"

#pragma comment(lib,"ws2_32.lib")
using namespace std;

void initialization();
#pragma comment(lib, "WS2_32.lib")

int _main()
{   
	//定义长度变量
	int send_len = 0;
	int recv_len = 0;
	//定义发送缓冲区和接受缓冲区
    char send_buf[100] = {};
	char recv_buf[200] = {};
    string recvstr;
	//定义服务端套接字，接受请求套接字
	SOCKET s_server;
	//服务端地址客户端地址
	SOCKADDR_IN server_addr;
	initialization();
	//填充服务端信息
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
	//创建套接字
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}

	//登录
    send_len = send(s_server, "[0# System.Login 0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf,'\0',sizeof(recv_buf));
    Sleep(100);
	//使能
    send_len = send(s_server, "[0# Robot.PowerEnable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(2000);
	//停止
	send_len = send(s_server, "[0# System.Abort 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Abort]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);
	//启动
	send_len = send(s_server, "[0# System.Start 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Start]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(100);
	//auto
	send_len = send(s_server, "[0# System.Auto 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Automode]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);

	HLRobot::Init();
	ofstream outfile;
	outfile.open("data.txt");
	outfile.close();
	outfile.open("myDcr.txt");
	outfile.close();

	//起始点
	PosStruct Start;
	Start.x = 415.746; Start.y = -219.622; Start.z = 920.636;
	Start.yaw = -29.489; Start.pitch = 178.867; Start.roll = -33.751;

	//终止点
	PosStruct End;
	End.x = 555.893; End.y = 54.405; End.z = 920.636;
	End.yaw = -8.288; End.pitch = 178.867; End.roll = -33.751;

	//梯型速度规划
	CHLMotionPlan trajectory1;
	trajectory1.SetPlanPoints(Start, End);
	trajectory1.SetProfile(10, 10, 10);    //vel °/s， acc °/s.s, dec °/s.s
	trajectory1.SetSampleTime(0.001);      //s
	trajectory1.GetPlanPoints();           //关节空间梯形速度规划
	trajectory1.SetProfile(10, 10, 10);    //vel °/s， acc °/s.s, dec °/s.s
	trajectory1.GetPlanPoints_line();      //笛卡尔空间直线轨迹梯形速度规划

	FtpControl::Upload("192.168.10.101", "data", "data.txt", "joint.txt");
	//FtpControl::Upload("192.168.10.101", "data", "myDcr.txt", "car_zyz.txt");
	//PPB使能
	send_len = send(s_server, "[0# PPB.Enable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[PPB_Enable]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);
	//关节坐标系
	send_len = send(s_server, "[0# Robot.Frame 1,1]", 100, 0);
	//send_len = send(s_server, "[0# Robot.Frame 1,2]", 100, 0);//笛卡尔
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Frame2]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);
	//PPB读取data文件
	send_len = send(s_server, "[0# PPB.ReadFile 1,/data/joint.txt]", 100, 0);
	//send_len = send(s_server, "[0# PPB.ReadFile 1,/data/car_zyz.txt]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ReadFile]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	//到达起始点
	send_len = send(s_server, "[0# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ToStartPoint]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);
	//PPB运行
	send_len = send(s_server, "[0# PPB.Run 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Run]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

	//码垛
	//清空
	outfile.open("myDcr.txt");
	outfile.close();
	//P2->P1
	Start.x = 555.180; Start.y = 291.270; Start.z = 460.25;
	Start.yaw = -151.2; Start.pitch = 179.02; Start.roll = 81.866;
	End.x = 555.180; End.y = 291.270; End.z = 401.250;
	End.yaw = -151.2; End.pitch = 179.02; End.roll = 81.866;
	trajectory1.SetPlanPoints(Start, End);
	trajectory1.SetProfile(10, 10, 10);    //vel °/s， acc °/s.s, dec °/s.s
	trajectory1.SetSampleTime(0.001);
	trajectory1.GetPlanPoints_line();

	//FtpControl::Upload("192.168.10.101", "data", "myDcr.txt", "car_zyz.txt");

	//读文件
	send_len = send(s_server, "[0# PPB.ReadFile 1,/data/car_zyz.txt]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ReadFile]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	//到达起始点
	send_len = send(s_server, "[0# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ToStartPoint]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);
	//PPB运行
	send_len = send(s_server, "[0# PPB.Run 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Run]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

	//开启吸盘
	send_len = send(s_server, "[0# IO.Set DOUT(20101),1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Open]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

	//清空
	outfile.open("myDcr.txt");
	outfile.close();
	//P1->P2
	trajectory1.SetPlanPoints(End, Start);
	trajectory1.GetPlanPoints_line();
	//P2->P3
	Start.x = 555.180; Start.y = 291.270; Start.z = 460.25;
	Start.yaw = -151.2; Start.pitch = 179.02; Start.roll = 81.866;
	End.x = 355.180; End.y = 411.270; End.z = 460.25;
	End.yaw = -151.2; End.pitch = 179.02; End.roll = 81.866;
	trajectory1.SetPlanPoints(Start, End);
	trajectory1.GetPlanPoints_line();
	//P3->P4
	Start.x = 355.180; Start.y = 411.270; Start.z = 401.25;
	Start.yaw = -151.2; Start.pitch = 179.02; Start.roll = 81.866;
	trajectory1.SetPlanPoints(End, Start);
	trajectory1.GetPlanPoints_line();

	//FtpControl::Upload("192.168.10.101", "data", "myDcr.txt", "car_zyz.txt");

	//读文件
	send_len = send(s_server, "[0# PPB.ReadFile 1,/data/car_zyz.txt]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ReadFile]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	//到达起始点
	send_len = send(s_server, "[0# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ToStartPoint]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);
	//PPB运行
	send_len = send(s_server, "[0# PPB.Run 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Run]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

	//关闭吸盘
	send_len = send(s_server, "[0# IO.Set DOUT(20101),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Close]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

	//清空
	outfile.open("myDcr.txt");
	outfile.close();
	//P4->P1
	trajectory1.SetPlanPoints(Start, End);
	trajectory1.GetPlanPoints_line();

	//FtpControl::Upload("192.168.10.101", "data", "myDcr.txt", "car_zyz.txt");

	//读文件
	send_len = send(s_server, "[0# PPB.ReadFile 1,/data/car_zyz.txt]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ReadFile]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	//到达起始点
	send_len = send(s_server, "[0# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ToStartPoint]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);
	//PPB运行
	send_len = send(s_server, "[0# PPB.Run 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Run]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
	return 0;
}
void initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确！" << endl;
	}
}
