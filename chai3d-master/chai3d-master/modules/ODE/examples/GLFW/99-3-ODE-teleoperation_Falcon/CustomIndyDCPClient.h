#pragma once

#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
	#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
	#define _CRT_SECURE_NO_WARNINGS
#endif 

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

#ifndef SERVER_PORT
	#define SERVER_PORT 6067			//!!!edit
#endif

class CustomIndyDedicatedTCPTestClient //: public Poco::Runnable
{
public:
	//enum {
	//	SIZE_HEADER = 52,
	//	SIZE_COMMAND = 4,
	//	SIZE_HEADER_COMMAND = 56,
	//	SIZE_DATA_MAX = 200,
	//	SIZE_DATA_ASCII_MAX = 32
	//};

	enum {
		SIZE_USER_INPUT = 57,
		SIZE_INDY_STATE = 61,
	};

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
	struct CustomUserInputStruct{					// 57 byte
		int invokeId;								// message ID (4 byte)
		int targetPos[6];							// master device position in 10um and Euler angle in 0.001rad (24 byte)
		int targetVel[6];							// master device velocity in 10um and Euler angle in 0.001rad (24 byte)
		int passivityPort;							// reserved for passivity port (4 byte)
		unsigned char sof;							// source of Frame (1 byte)
	};

	struct CustomIndyStateStruct{					// 57 byte
		int invokeId;								// message ID which is same with userinput (4 byte)
		int indyPos[6];								// Indy position in 10um and Euler angle in 0.001rad (24 byte)
		int force[3];								// force value in mN (12 byte)
		int torque[3];								// torque value in mNm (12 byte)
		int cmode;									// Indy cmode (4 byte)
		int damping;								// user feedback damping coefficient (4 byte)
		unsigned char sof;							// source of Frame (1 byte)
	};
#pragma pack(pop)   /* restore original alignment from stack */

	union CustomUserInput
	{
		unsigned char byte[SIZE_USER_INPUT];
		CustomUserInputStruct val;
	};

	union CustomIndyState
	{
		unsigned char byte[SIZE_INDY_STATE];
		CustomIndyStateStruct val;
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
	CustomUserInput userInput;
	CustomIndyState indyState;

	unsigned char readBuff[1024];
	unsigned char writeBuff[1024];
	// Send and Response functions
	void SendUserInput();
	void ReadIndyState();

	////// Cmd Transmittion
	bool Run();
	void SendIndyCommandAndReadState(double * masterPos, double * masterVel, double passivityPort, double * indyPos, double * forceTorque, int& cmode, double& damping);

}; /* end of class */
