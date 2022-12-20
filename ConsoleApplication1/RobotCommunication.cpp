#include<iostream>
#include<winsock.h>
#include <conio.h>
#include "ftp/FtpControl.h"
#include "eigen3/Eigen/Core"
#include "MotionPlan.h"
#include "RobotCommunication.h"

using namespace std;

RobotCommunication::RobotCommunication()
{
	// server info
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
}

RobotCommunication::RobotCommunication(string addr, int port)
{
	// server info
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr(addr.data());
	server_addr.sin_port = htons(port);
}

RobotCommunication::~RobotCommunication()
{
	closesocket(s_server);
	// release dll
	WSACleanup();
}

void RobotCommunication::connect_server()
{
	// initialize socket
	WORD w_req = MAKEWORD(2, 2);//°æ±¾ºÅ
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "[ComInit]\tFailed to intialize socket lib" << endl;
	}
	else {
		cout << "[ComInit]\tSocket lib initialize successfully" << endl;
	}
	// check version
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "[ComInit]\tSocket version mismatch" << endl;
		WSACleanup();
	}
	else {
		cout << "[ComInit]\tSocket version correct" << endl;
	}
	// create socket
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	cout << "[Server]\tWaiting to connect to server..." << endl;
	if (connect(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "[Server]\tFailed to connect to server" << endl;
		WSACleanup();
		exit(0);
	}
	else {
		cout << "[Server]\tConnected to server successfully" << endl;
	}
}

void RobotCommunication::initialize_command()
{
	// login
	communicate("[1# System.Login 0]", "[login]", 1200);
	// enable
	communicate("[2# Robot.PowerEnable 1,1]", "[enable]", 1200);
	// auto mode
	communicate("[3# System.Auto 1]", "[AutoMode]", 200);
	// system abort
	communicate("[3# System.Abort 1]", "[Abort]");
	// system start
	communicate("[4# System.Start 1]", "[Start]");
	// robot home
	communicate("[4# Robot.Home 1]", "[Home]");
	// robot frame
	communicate("[8# Robot.Frame 1,1]", "[Frame]", 200);
	// speed
	communicate("[6# System.Speed 100]", "[Speed]");
	// robot speed
	communicate("[6# Robot.Speed 1,100]", "[Speed]");

}

string RobotCommunication::communicate(const char* s_send, const string& log_header, int delay, bool if_print)
{
	char recv_buf[200] = {};
	int send_len = send(s_server, s_send, 100, 0);
	int recv_len = recv(s_server, recv_buf, 200, 0);
	if (if_print) cout << log_header << "\t" << recv_buf << endl;
	string recv(recv_buf);
	Sleep(delay);
	return recv;
}

void RobotCommunication::initialize()
{
	connect_server();
	initialize_command();
}

Matrix<double, 6, 1> RobotCommunication::decode_coord(const string& data)
{
	Matrix<double, 6, 1> pos;
	pos(0) = stod(data.substr(6, 9));
	pos(1) = stod(data.substr(16, 9));
	pos(2) = stod(data.substr(26, 9));
	pos(3) = stod(data.substr(36, 9));
	pos(4) = stod(data.substr(46, 9));
	pos(5) = stod(data.substr(56, 9));
	return pos;
}