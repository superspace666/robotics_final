#include "winsock.h"
#include "string"
#include "MotionPlan.h"

using namespace std;

class RobotCommunication
{
public:
	RobotCommunication();
	RobotCommunication(string addr, int port);
	~RobotCommunication();
	void initialize();
	string communicate(const char* s_send, const string& log_header, int delay = 100, bool if_print = true);
	static Matrix<double, 6, 1> decode_coord(const string& data);
private:
	void connect_server();
	void initialize_command();
	SOCKET s_server;
	SOCKADDR_IN server_addr;
};