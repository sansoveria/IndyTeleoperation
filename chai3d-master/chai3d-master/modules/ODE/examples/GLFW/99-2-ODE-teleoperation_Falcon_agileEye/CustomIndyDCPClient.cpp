#include "pch.h"
#include "CustomIndyDCPClient.h"
#include <stdio.h>



#define SOF_SERVER  0x12
#define SOF_CLIENT  0x34


CustomIndyDedicatedTCPTestClient::CustomIndyDedicatedTCPTestClient() :v_sockFd(-1), v_invokeId(0), v_working(true)
{
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	// userInput setting
	userInput.val.sof = SOF_CLIENT;

	InitializeCriticalSection(&DCP_cs);
};

CustomIndyDedicatedTCPTestClient::~CustomIndyDedicatedTCPTestClient()
{
	WSACleanup();
	DeleteCriticalSection(&DCP_cs);
};


void CustomIndyDedicatedTCPTestClient::connect()
{
	connect(SERVER_IP, ROBOT_NAME);
}

void CustomIndyDedicatedTCPTestClient::connect(const char* serverip, const char* robotname) {

	userInput.val.sof = SOF_CLIENT;

	struct sockaddr_in server_addr;
	v_sockFd = socket(AF_INET, SOCK_STREAM, 0);
	if (v_sockFd < 0) {
		v_message = ("Socket opening Error\n");
	}

	server_addr.sin_addr.s_addr = inet_addr(serverip);
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(SERVER_PORT);

	if (::connect(v_sockFd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
		v_message = ("Socket connection Failed\n");
		closesocket(v_sockFd);
		v_sockFd = -1;
	}
	else
	{
		v_message = ("Socket connection Success\n");
	}
}

void CustomIndyDedicatedTCPTestClient::disconnect()
{
	closesocket(v_sockFd);
	v_message = ("Socket Disconnected\n");
}

void CustomIndyDedicatedTCPTestClient::SendUserInput()
{
	// Send header
	memcpy(writeBuff, userInput.byte, SIZE_USER_INPUT);
	if (send(v_sockFd, (const char*)writeBuff, SIZE_USER_INPUT, 0) == -1)
	{
		// send error
		closesocket(v_sockFd);
		v_sockFd = -1;
	}

}

void CustomIndyDedicatedTCPTestClient::ReadIndyState()
{
	int len = 0;
	int cur = 0;
	// Read header
	while (true)
	{
		len = recv(v_sockFd, (char*)readBuff + cur, SIZE_INDY_STATE - cur, MSG_WAITALL);
		if (len == 0)
		{
			//Poco::Thread::sleep(100);
			//Sleep(100);
		}
		else if (len > 0)
		{
			cur += len;
			if (cur == SIZE_INDY_STATE) break;
		}
		else
		{
			// Recieving error
			closesocket(v_sockFd);
			v_sockFd = -1;
			break;
		}
	}
	if (v_sockFd == -1) return;
	memcpy(indyState.byte, readBuff, SIZE_INDY_STATE);
	//message = ("CustomIndyDedicatedTCPTestClient : Read response header \n");
}

bool CustomIndyDedicatedTCPTestClient::Run()
{
	userInput.val.invokeId = v_invokeId++;

	// Communication
	if (v_sockFd >= 0)
	{
		SendUserInput();
		ReadIndyState();
	}

	// Integrity check
	return (userInput.val.invokeId == indyState.val.invokeId && indyState.val.sof == SOF_SERVER);
}

void CustomIndyDedicatedTCPTestClient::SendIndyCommandAndReadState(double * masterPos, double * masterVel, double passivityPort, double * indyPos, double * forceTorque, int& cmode) {
	// masterPos, indyPos: position in m, axisangle in rad
	// passivityPort: reserved for later
	// cmode: current Indy cmode (ex - 0: stationary, 1: joint move, ..., 20: teleoperation mode) 

	EnterCriticalSection(&DCP_cs);
	// prepare command
	userInput.val.passivityPort = (int)(passivityPort * 1000.0);
	userInput.val.targetPos[0] = (int)(masterPos[0] * 100000.0);
	userInput.val.targetPos[1] = (int)(masterPos[1] * 100000.0);
	userInput.val.targetPos[2] = (int)(masterPos[2] * 100000.0);
	userInput.val.targetPos[3] = (int)(masterPos[3] * 1000.0);
	userInput.val.targetPos[4] = (int)(masterPos[4] * 1000.0);
	userInput.val.targetPos[5] = (int)(masterPos[5] * 1000.0);
	userInput.val.targetVel[0] = (int)(masterVel[0] * 100000.0);
	userInput.val.targetVel[1] = (int)(masterVel[1] * 100000.0);
	userInput.val.targetVel[2] = (int)(masterVel[2] * 100000.0);
	userInput.val.targetVel[3] = (int)(masterVel[3] * 1000.0);
	userInput.val.targetVel[4] = (int)(masterVel[4] * 1000.0);
	userInput.val.targetVel[5] = (int)(masterVel[5] * 1000.0);
	
	// Run TCP communication
	if (Run()) {
		indyPos[0] = (double)indyState.val.indyPos[0] / 100000.0;
		indyPos[1] = (double)indyState.val.indyPos[1] / 100000.0;
		indyPos[2] = (double)indyState.val.indyPos[2] / 100000.0;
		indyPos[3] = (double)indyState.val.indyPos[3] / 1000.0;
		indyPos[4] = (double)indyState.val.indyPos[4] / 1000.0;
		indyPos[5] = (double)indyState.val.indyPos[5] / 1000.0;
		forceTorque[0] = (double)indyState.val.force[0] / 1000.0;
		forceTorque[1] = (double)indyState.val.force[1] / 1000.0;
		forceTorque[2] = (double)indyState.val.force[2] / 1000.0;
		forceTorque[3] = (double)indyState.val.torque[0] / 1000.0;
		forceTorque[4] = (double)indyState.val.torque[1] / 1000.0;
		forceTorque[5] = (double)indyState.val.torque[2] / 1000.0;
		cmode = indyState.val.cmode;
	}
	else {
		indyPos[0] = 0.0;
		indyPos[1] = 0.0;
		indyPos[2] = 0.0;
		indyPos[3] = 0.0;
		indyPos[4] = 0.0;
		indyPos[5] = 0.0;
		forceTorque[0] = 0.0;
		forceTorque[1] = 0.0;
		forceTorque[2] = 0.0;
		forceTorque[3] = 0.0;
		forceTorque[4] = 0.0;
		forceTorque[5] = 0.0;
		cmode = -1;
	}
	
	LeaveCriticalSection(&DCP_cs);
}

