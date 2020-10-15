#pragma once

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS

#include "Demo_setup.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>


//for TCP/IP Socket
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib") //for winsock2

#define MSG_WAITALL 0x8
#include <iostream>
using namespace std;



// Robot connection
//#define __Indy7__
//#define __IndyRP2__

#ifdef __Indy7__
//#define SERVER_IP "192.168.3.116"		// SCRC
//#define SERVER_IP "141.223.193.55"	// LAB INDY 2 (Junsu)
#define SERVER_IP "141.223.193.45"	// LAB INDY 1
//#define SERVER_IP "141.223.193.239"
//#define SERVER_IP "141.223.193.58"
#define ROBOT_NAME "NRMK-Indy7"

#elif defined(__IndyRP2__)
//#define SERVER_IP "141.223.193.55"	
#define SERVER_IP "192.168.0.19"		// SCRC
#define ROBOT_NAME "NRMK-IndyRP2"
#endif


#define SERVER_PORT 6067			//!!!edit

class CustomIndyDedicatedTCPTestClient //: public Poco::Runnable
{
public:
	enum Command : int
	{
		CMD_EMERGENCY_STOP = 1,
		CMD_RESET_ROBOT = 2,
		CMD_SET_SERVO = 3,
		CMD_SET_BRAKE = 4,
		CMD_STOP = 5,
		CMD_MOVE = 6,
		CMD_MOVE_HOME = 7,
		CMD_MOVE_ZERO = 8,
		CMD_JOINT_MOVE_TO = 9,
		CMD_JOINT_MOVE_BY = 10,
		CMD_TASK_MOVE_TO = 11,
		CMD_TASK_MOVE_BY = 12,
		CMD_START_CURRENT_PROGRAM = 14,
		CMD_PAUSE_CURRENT_PROGRAM = 15,
		CMD_RESUME_CURRENT_PROGRAM = 16,
		CMD_STOP_CURRENT_PROGRAM = 17,
		CMD_START_DEFAULT_PROGRAM = 18,

		CMD_IS_ROBOT_RUNNING = 30,	//refined ...
		CMD_IS_READY = 31,
		CMD_IS_EMG = 32,
		CMD_IS_COLLIDED = 33,
		CMD_IS_ERR = 34,
		CMD_IS_BUSY = 35,
		CMD_IS_MOVE_FINISEHD = 36,
		CMD_IS_HOME = 37,
		CMD_IS_ZERO = 38,
		CMD_IS_IN_RESETTING = 39,
		CMD_IS_DIRECT_TECAHING = 60,
		CMD_IS_TEACHING = 61,
		CMD_IS_PROGRAM_RUNNING = 62,
		CMD_IS_PROGRAM_PAUSED = 63,
		CMD_IS_CONTY_CONNECTED = 64,

		CMD_CHANGE_DIRECT_TEACHING = 80,
		CMD_FINISH_DIRECT_TEACHING = 81,	//... refined

		CMD_SET_DEFAULT_TCP = 100,
		CMD_RESET_DEFAULT_TCP = 101,
		CMD_SET_COMP_TCP = 102,
		CMD_RESET_COMP_TCP = 103,
		CMD_SET_REFFRAME = 104,
		CMD_RESET_REFFRAME = 105,
		CMD_SET_COLLISION_LEVEL = 106,
		CMD_SET_JOINT_BOUNDARY = 107,
		CMD_SET_TASK_BOUNDARY = 108,
		CMD_SET_JOINT_BLEND_RADIUS_LEVEL = 109,
		CMD_SET_TASK_BLEND_RADIUS_LEVEL = 110,
		CMD_SET_JOINT_WTIME = 111,
		CMD_SET_TASK_WTIME = 112,
		CMD_SET_TASK_CMODE = 113,
		CMD_SET_JOINT_BLEND_RADIUS = 116,
		CMD_SET_TASK_BLEND_RADIUS = 117,

		CMD_GET_DEFAULT_TCP = 200,
		CMD_GET_COMP_TCP = 201,
		CMD_GET_REFFRAME = 202,
		CMD_GET_COLLISION_LEVEL = 203,
		CMD_GET_JOINT_BOUNDARY = 204,
		CMD_GET_TASK_BOUNDARY = 205,
		CMD_GET_JOINT_BLEND_RADIUS = 206,
		CMD_GET_TASK_BLEND_RADIUS = 207,
		CMD_GET_JOINT_WTIME = 208,
		CMD_GET_TASK_WTIME = 209,
		CMD_GET_TASK_CMODE = 210,

		CMD_GET_RUNNING_TIME = 300,
		CMD_GET_CMODE = 301,
		CMD_GET_JOINT_STATE = 302,
		CMD_GET_JOINT_POSITION = 320,
		CMD_GET_JOINT_VELOCITY = 321,
		CMD_GET_TASK_POSITION = 322,
		CMD_GET_TASK_VELOCITY = 323,
		CMD_GET_TORQUE = 324,

		CMD_GET_LAST_EMG_INFO = 380,	//new

		CMD_GET_SMART_DI = 400,
		CMD_GET_SMART_DIS = 401,
		CMD_SET_SMART_DO = 402,
		CMD_SET_SMART_DOS = 403,
		CMD_GET_SMART_AI = 404,
		CMD_SET_SMART_AO = 405,

		CMD_GET_EXTIO_FTCAN_ROBOT_RAW = 420,
		CMD_GET_EXTIO_FTCAN_ROBOT_TRANS = 421,
		CMD_GET_EXTIO_FTCAN_CB_RAW = 422,
		CMD_GET_EXTIO_FTCAN_CB_TRANS = 423,

#ifdef TELEOPERATION
		CMD_TOGGLE_TELEOPERATION_MODE = 570,
		CMD_SET_REF_POSE = 571,
#endif
//DH_end

		CMD_FOR_EXTENDED = 800,
		CMD_FOR_STREAMING = 801,

		CMD_SEND_KEYCOMMAND = 9996,
		CMD_READ_MEMORY = 9997,
		CMD_WRITE_MEMORY = 9998,
		CMD_ERROR = 9999
	};


	enum {
		SIZE_HEADER = 52,
		SIZE_COMMAND = 4,
		SIZE_HEADER_COMMAND = 56,
		SIZE_DATA_MAX = 200,
		SIZE_DATA_ASCII_MAX = 32
	};

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
	struct HeaderCommandStruct
	{
		char robotName[20];
		char robotVersion[12];
		unsigned char stepInfo;
		unsigned char sof;		//source of Frame
		int invokeId;
		int dataSize;
		char reserved[10];
		int cmdId;
	};
#pragma pack(pop)   /* restore original alignment from stack */

	union HeaderCommand
	{
		unsigned char byte[SIZE_HEADER_COMMAND];
		HeaderCommandStruct val;
	};

	union Data
	{
		unsigned char byte[SIZE_DATA_MAX];
		char asciiStr[SIZE_DATA_ASCII_MAX + 1];
		char str[200];
		char charVal;
		bool boolVal;
		short shortVal;
		int intVal;
		float floatVal;
		double doubleVal;
		char char2dArr[2];
		char char3dArr[3];
		char char6dArr[6];
		char char7dArr[200];
		char charArr[200];
		int int2dArr[2];
		int int3dArr[3];
		int int6dArr[6];
		int int7dArr[6];
		int intArr[50];
		float float3dArr[3];
		float float6dArr[6];
		float float7dArr[7];
		float floatArr[50];
		double double3dArr[3];
		double double6dArr[6];
		double double7dArr[6];
		double doubleArr[25];
	};

	// Constructor and destructor
	CustomIndyDedicatedTCPTestClient();
	~CustomIndyDedicatedTCPTestClient();

	/*
	*@breif	Thread function
	*/
	CRITICAL_SECTION DCP_cs;
	void connect();
	void connect(const char* serverip, const char* robotname);
	void disconnect();
	void run();

	string getv_message() { return this->v_message; }
	char *getv_data() { return this->v_data; }
	bool getv_dataChanged() { return this->v_dataChanged; }
	void setv_inchar(char c) { this->v_inchar = c; }
	void setv_dataChanged(bool b) { this->v_dataChanged = b; }

private:
	bool v_working;
	int v_sockFd;
	int v_invokeId;
	char v_inchar;

	string v_message;
	char v_data[256];
	bool v_dataChanged = false;

	// Various functions
public:
	// Header and Data
	HeaderCommand header;
	HeaderCommand resHeader;
	Data data;
	Data extData;
	Data resData;

	unsigned char readBuff[1024];
	unsigned char writeBuff[1024];
	// Send and Response functions
	void SendHeader();
	void SendData();
	void SendExtData(int extDataSize);
	void ReadHeader();
	void ReadData();

	////// Cmd Transmittion
	void Run(int cmdID, int dataSize, int extDataSize);
	void Run(int cmdID, int dataSize);
	void Run(int cmdID);
	////// Functions
	//// Basic
	bool isReady();
	bool isMoveFinished();
	bool isProgramRunning();
	void GoHome();
	void GoZero();
	void StartDirectTeaching();
	void StopDirectTeaching();
	//// Info
	double *GetJointPos();
	double *GetTaskPos();
	double *GetJointVel();
	double *GetTaskVel();
	double *GetTorque();
	//// Move
	void MoveByJ(int numJoint, char dir, float dist);
	void MoveByT(double *coordinate);

	void MoveToT(float *arr);
	void MoveToJ(float *arr);
	
	//// For Extended
	void TrackTrajectory(std::string filename);

#ifdef TELEOPERATION
	void ToggleTeleoperationMode();
	void SetRefState(double *pos, double *vel);
#endif

}; /* end of class */
