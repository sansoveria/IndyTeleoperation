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


#define SERVER_PORT 6066			//!!!edit

class IndyDedicatedTCPTestClient //: public Poco::Runnable
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

//DH_start
#if defined(_DH_PROGRAM_SETUP)
		CMD_INIT_CUSTOM_PROGRAM = 500,
		CMD_ADD_MOVEHOME_CUSTOM_PROGRAM = 501,
		CMD_ADD_JOINTMOVETO_CUSTOM_PROGRAM = 502,
		CMD_ADD_TASKMOVETO_CUSTOM_PROGRAM = 503,
		CMD_ADD_WAIT_CUSTOM_PROGRAM = 504,
		CMD_ADD_DOCONTROL_CUSTOM_PROGRAM = 505,
		CMD_ADD_TOOLCOMMAND_CUSTOM_PROGRAM = 506,
		CMD_SET_CUSTOM_PROGRAM = 510,
		CMD_CLEAR_CUSTOM_PROGRAM = 511,
		CMD_ADD_TASKMOVEBY_CUSTOM_PROGRAM = 512,

		CMD_DEMO_PICK_PROGRAM1 = 520,
		CMD_DEMO_PICK_PROGRAM2 = 521,
		CMD_DEMO_PUT_PROGRAM1 = 522,
		CMD_DEMO_PUT_PROGRAM2 = 523,
		CMD_DEMO_GRIP_PROGRAM1 = 524,
		CMD_DEMO_GRIP_PROGRAM2 = 525,
		CMD_DEMO_SET_ASSEMBLE_PROGRAM = 526,
		CMD_DEMO_RUN_ASSEMBLE_PROGRAM = 527,
		CMD_DEMO_SET_AFTER_ASSEMBLE_ROBOT1_PROGRAM = 528,
		CMD_DEMO_SET_AFTER_ASSEMBLE_ROBOT2_PROGRAM = 529,
		CMD_DEMO_RUN_AFTER_ASSEMBLE_PROGRAM = 530,
#endif //_DH_PROGRAM_SETUP

#ifdef TASK_SPACE_CONTROL_OFFLINE
		CMD_TRACK_GIVEN_POINT_LIST = 550,
#endif

#ifdef NULL_MOTION
		CMD_START_NULL_MOTION = 551,
		CMD_STOP_NULL_MOTION = 552,
#endif

#ifdef TASK_SPACE_CONTROL_OFFLINE
		CMD_INIT_POINT_LIST = 560,
		CMD_ADD_POINT_TO_POINT_LIST = 561,
		CMD_DELETE_LAST_POINT = 562,
		CMD_SAVE_POINT_LIST = 563,
#endif

#ifdef TASK_SPACE_CONTROL
		CMD_SET_FDCC_GAINS = 520,
		CMD_TOGGLE_CMODE_ADMITTANCE_CONTROL = 521,
		CMD_SET_COMPLIANCE_PARAMETERS = 522,
#endif 

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
	IndyDedicatedTCPTestClient();
	~IndyDedicatedTCPTestClient();

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


/* references (custom functions) */
/*
//// DH_start
#if defined(_DH_PROGRAM_SETUP)
void InitProgram();
void AddProgramMoveHome();
void AddProgramTaskMoveTo(double *p, int velLevel = 5);
void AddProgramTaskMoveBy(double *p, int velLevel = 5);
void AddProgramJointMoveTo(double *q, int velLevel = 5);
void AddProgramWait(double time);
void AddProgramDOControl(int idx, bool on);
//void AddProgramToolCommand(int idx, int* cmd);
void AddProgramToolCommand(int toolId, int command);
void SetProgram();
void StartProgram();
void PauseProgram();
void ResumeProgram();
void ClearProgram();
void StopProgram();

void RunDemoPickProgram(
	double* pre_pick_cover,
	double* pickcover_above,
	double* pickcover,
	double* lift_up,
	double* updown_cover,
	double waiting_time,
	int motion_velocity,
	int DO_port_num,
	int tool_index,
	int tool_open
);
void RunDemoPutProgram(
	double* placecover,
	double* liftdown,
	double* move_side,
	double* liftup,
	double waiting_time,
	int motion_velocity,
	int DO_port_num
);
void RunDemoGripProgram(
	double* pre_pickcover_holder,
	double* liftdown_to_grip,
	double* liftup,
	double* pre_assemble,
	double waiting_time,
	int motion_velocity,
	int tool_index,
	int tool_open,
	int tool_close
);
void SetDemoAssembleProgram(
	double* pre_assemble_1,
	double* pre_assemble_2,
	int motion_velocity
);
void RunDemoAssembleProgram();
void SetDemoRobot1AfterAssembleProgram(
	double* put_pose,
	double waiting_time,
	int motion_velocity,
	int tool_index,
	int tool_open
);
void SetDemoRobot2AfterAssembleProgram(
	double* put_pose,
	double waiting_time_1,
	double waiting_time_2,
	int motion_velocity,
	int tool_index,
	int tool_open
);
void RunDemoAfterAssembleProgram();

bool WaitForProgramFinish();
#endif //_DH_PROGRAM_SETUP
//// DH_end

#ifdef NULL_MOTION
bool StartNullMotion(double amp);

bool IndyDedicatedTCPTestClient::StopNullMotion();
#endif

#ifdef TASK_SPACE_CONTROL_OFFLINE
bool SetOperationalMotionTargetPointListFile(std::string filename, double velocity, double joint_accel);
bool InitPointList();
bool AddCurrentConfigToPointList();
bool DeleteLastPoint();
bool SavePointList(std::string filename);
#endif

#ifdef TASK_SPACE_CONTROL
void SetFDCCGains(double k, double kd, double psi_width = 1.0 / 25.0, double fp_thres = 100.0);
void ToggleAdmittanceControl();
void SetComplianceParameters(double M, double K);
#endif
*/
