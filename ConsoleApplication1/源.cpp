// ConsoleApplication1.cpp : �������̨Ӧ�ó������ڵ㡣

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
	//���峤�ȱ���
	int send_len = 0;
	int recv_len = 0;
	//���巢�ͻ������ͽ��ܻ�����
	char send_buf[100] = {};
	char recv_buf[200] = {};
	string recvstr;
	//���������׽��֣����������׽���
	SOCKET s_server;
	//����˵�ַ�ͻ��˵�ַ
	SOCKADDR_IN server_addr;
	initialization();
	//���������Ϣ
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
	//�����׽���
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "����������ʧ�ܣ�" << endl;
		WSACleanup();
	}
	else {
		cout << "���������ӳɹ���" << endl;
	}

	//��¼
	send_len = send(s_server, "[0# System.Login 0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(100);
	//ʹ��
	send_len = send(s_server, "[0# Robot.PowerEnable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(2000);
	//ֹͣ
	send_len = send(s_server, "[0# System.Abort 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Abort]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);
	//����
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

	//��ʼ��
	PosStruct Start(632.182, 408.272, 551.192, -151.205, 179.020, 81.861);

	//��ֹ��
	PosStruct End(632.182, 408.272, 462.25, -151.205, 179.020, 81.861);

	/*Start.SetPos(415.746, -219.622, 920.636, -29.489, 178.867, -33.751);
	End.SetPos(528.785, 215.597, 666.566, -8.273, 178.868, -33.734);*/

	Start.SetPos(643.118, 278.763, 531.499, -8.269, 178.868, -143.729);
	End.SetPos(653.118, 378.763, 471.499, -8.275, 178.868, -143.735);

	//�����ٶȹ滮
	CHLMotionPlan trajectory1;
	trajectory1.SetPlanPoints(Start, End);
	trajectory1.SetSampleTime(0.001);      //s
	trajectory1.SetProfile(50, 50, 50);    //vel ��/s�� acc ��/s.s, dec ��/s.s
	trajectory1.GetPlanPoints_line();      //�ѿ����ռ�ֱ�߹켣�����ٶȹ滮

	//FtpControl::Upload("192.168.10.101", "data", "data.txt", "joint.txt");
	FtpControl::Upload("192.168.10.101", "data", "myDcr.txt", "car_zyz.txt");
	//PPBʹ��
	send_len = send(s_server, "[0# PPB.Enable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[PPB_Enable]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);
	//�ؽ�����ϵ
	send_len = send(s_server, "[0# Robot.Frame 1,1]", 100, 0);
	//send_len = send(s_server, "[0# Robot.Frame 1,2]", 100, 0);//�ѿ���
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Frame2]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(200);
	//PPB��ȡdata�ļ�
	//send_len = send(s_server, "[0# PPB.ReadFile 1,/data/joint.txt]", 100, 0);
	send_len = send(s_server, "[0# PPB.ReadFile 1,/data/car_zyz.txt]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ReadFile]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	//������ʼ��
	send_len = send(s_server, "[0# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[ToStartPoint]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);
	//PPB����
	send_len = send(s_server, "[0# PPB.Run 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Run]" << "\t" << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

	closesocket(s_server);
	//�ͷ�DLL��Դ
	WSACleanup();
	return 0;
}
void initialization() {
	//��ʼ���׽��ֿ�
	WORD w_req = MAKEWORD(2, 2);//�汾��
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "��ʼ���׽��ֿ�ʧ�ܣ�" << endl;
	}
	else {
		cout << "��ʼ���׽��ֿ�ɹ���" << endl;
	}
	//���汾��
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "�׽��ֿ�汾�Ų�����" << endl;
		WSACleanup();
	}
	else {
		cout << "�׽��ֿ�汾��ȷ��" << endl;
	}
}
