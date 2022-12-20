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
	memset(recv_buf, '\0', sizeof(recv_buf));
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
	PosStruct Start(632.182, 408.272, 551.192, -151.205, 179.020, 81.861);

	//终止点
	PosStruct End(632.182, 408.272, 462.25, -151.205, 179.020, 81.861);

	/*Start.SetPos(415.746, -219.622, 920.636, -29.489, 178.867, -33.751);
	End.SetPos(528.785, 215.597, 666.566, -8.273, 178.868, -33.734);*/

	Start.SetPos(643.118, 278.763, 531.499, -8.269, 178.868, -143.729);
	End.SetPos(653.118, 378.763, 471.499, -8.275, 178.868, -143.735);

	//梯型速度规划
	CHLMotionPlan trajectory1;
	trajectory1.SetPlanPoints(Start, End);
	trajectory1.SetSampleTime(0.001);      //s
	trajectory1.SetProfile(50, 50, 50);    //vel °/s， acc °/s.s, dec °/s.s
	trajectory1.GetPlanPoints_line();      //笛卡尔空间直线轨迹梯形速度规划

	//FtpControl::Upload("192.168.10.101", "data", "data.txt", "joint.txt");
	FtpControl::Upload("192.168.10.101", "data", "myDcr.txt", "car_zyz.txt");
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
	//send_len = send(s_server, "[0# PPB.ReadFile 1,/data/joint.txt]", 100, 0);
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
