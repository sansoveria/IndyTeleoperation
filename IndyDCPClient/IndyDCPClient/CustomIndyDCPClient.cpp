#include "pch.h"
#include "CustomIndyDCPClient.h"
#include <stdio.h>



#define SOF_SERVER  0x12
#define SOF_CLIENT  0x34


CustomIndyDedicatedTCPTestClient::CustomIndyDedicatedTCPTestClient() :v_sockFd(-1), v_invokeId(0), v_working(true)
{
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	// Header setting
	strcpy(header.val.robotName, ROBOT_NAME);
	header.val.robotVersion[0] = '\0';
	header.val.stepInfo = 0x02;
	header.val.sof = SOF_CLIENT;

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

	// Header setting
	strcpy(header.val.robotName, robotname);
	header.val.robotVersion[0] = '\0';
	header.val.stepInfo = 0x02;
	header.val.sof = SOF_CLIENT;

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

void CustomIndyDedicatedTCPTestClient::SendHeader()
{
	// Send header
	memcpy(writeBuff, header.byte, SIZE_HEADER_COMMAND);
	if (send(v_sockFd, (const char*)writeBuff, SIZE_HEADER_COMMAND, 0) == -1)
	{
		// send error
		closesocket(v_sockFd);
		v_sockFd = -1;
	}

}

void CustomIndyDedicatedTCPTestClient::SendData()
{
	// Send data
	if (header.val.dataSize > 0)
	{
		memcpy(writeBuff, &data, header.val.dataSize);
		if (send(v_sockFd, (const char*)writeBuff, header.val.dataSize, 0) == -1)
		{
			// send error
			closesocket(v_sockFd);
			v_sockFd = -1;
		}
	}
}

void CustomIndyDedicatedTCPTestClient::SendExtData(int extDataSize)
{
	// Send data
	if (extDataSize > 0)
	{
		memcpy(writeBuff, &extData, extDataSize);
		if (send(v_sockFd, (const char*)writeBuff, extDataSize, 0) == -1)
		{
			// send error
			closesocket(v_sockFd);
			v_sockFd = -1;
		}
	}
}

void CustomIndyDedicatedTCPTestClient::ReadHeader()
{
	int len = 0;
	int cur = 0;
	// Read header
	while (true)
	{
		len = recv(v_sockFd, (char*)readBuff + cur, SIZE_HEADER_COMMAND - cur, MSG_WAITALL);
		if (len == 0)
		{
			//Poco::Thread::sleep(100);
			Sleep(100);
		}
		else if (len > 0)
		{
			cur += len;
			if (cur == SIZE_HEADER_COMMAND) break;
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
	memcpy(resHeader.byte, readBuff, SIZE_HEADER_COMMAND);
	//message = ("CustomIndyDedicatedTCPTestClient : Read response header \n");
}
void CustomIndyDedicatedTCPTestClient::ReadData()
{
	int len = 0;
	int cur = 0;
	if (resHeader.val.dataSize > 0)
	{
		// Read data
		while (true)
		{
			len = recv(v_sockFd, (char*)readBuff + cur, resHeader.val.dataSize - cur, MSG_WAITALL);
			if (len == 0)
			{
				Sleep(100);
				// Poco::Thread::sleep(100);
			}
			else if (len > 0)
			{
				cur += len;
				if (cur == resHeader.val.dataSize) break;
			}
			else
			{
				// Recieving error (error code: errno)
				closesocket(v_sockFd);
				v_sockFd = -1;
				break;
			}
		}

		if (v_sockFd == -1) return;
		memcpy(resData.byte, readBuff, resHeader.val.dataSize);
		//printf("CustomIndyDedicatedTCPTestClient : Read response data\n");
	}
}

void CustomIndyDedicatedTCPTestClient::Run(int cmdID, int dataSize, int extDataSize)
{
	// Perform socket communication
	header.val.invokeId = v_invokeId++;

	header.val.cmdId = cmdID;
	header.val.dataSize = dataSize;


	if (v_sockFd >= 0)
	{
		SendHeader();
		SendData();
		if (header.val.cmdId == CMD_FOR_EXTENDED && extDataSize > 0)
			SendExtData(extDataSize);

		ReadHeader();
		ReadData();
	}
}

void CustomIndyDedicatedTCPTestClient::Run(int cmdID, int dataSize)
{
	// Perform socket communication
	header.val.invokeId = v_invokeId++;

	header.val.cmdId = cmdID;
	header.val.dataSize = dataSize;


	if (v_sockFd >= 0)
	{
		SendHeader();
		SendData();

		ReadHeader();
		ReadData();
	}
}

void CustomIndyDedicatedTCPTestClient::Run(int cmdID)
{
	// Perform socket communication
	header.val.invokeId = v_invokeId++;

	header.val.cmdId = cmdID;
	header.val.dataSize = 0;


	if (v_sockFd >= 0)
	{
		SendHeader();
		SendData();

		ReadHeader();
		ReadData();
	}
}

double *CustomIndyDedicatedTCPTestClient::GetJointPos() {
	EnterCriticalSection(&DCP_cs);

	// Get joint position
	Run(CMD_GET_JOINT_POSITION, 0);

	if (header.val.cmdId == resHeader.val.cmdId
		&& header.val.invokeId == resHeader.val.invokeId
		&& resHeader.val.sof == SOF_SERVER)
	{
		// Process success: return resHeader.val.dataSize
		return &resData.double6dArr[0];
	}
	double zeros[6] = { 0 };
	return zeros;
	LeaveCriticalSection(&DCP_cs);
}

double *CustomIndyDedicatedTCPTestClient::GetJointVel() {
	EnterCriticalSection(&DCP_cs);

	// Get Joint Velocity (Angular)
	Run(CMD_GET_JOINT_VELOCITY, 0);

	if (header.val.cmdId == resHeader.val.cmdId
		&& header.val.invokeId == resHeader.val.invokeId
		&& resHeader.val.sof == SOF_SERVER)
	{
		// Process success: return resHeader.val.dataSize
		return &resData.double6dArr[0];
	}
	double zeros[6] = { 0 };
	return zeros;
	LeaveCriticalSection(&DCP_cs);
}

double *CustomIndyDedicatedTCPTestClient::GetTaskPos() {
	EnterCriticalSection(&DCP_cs);

	// Get Task position
	Run(CMD_GET_TASK_POSITION, 0);

	if (header.val.cmdId == resHeader.val.cmdId
		&& header.val.invokeId == resHeader.val.invokeId
		&& resHeader.val.sof == SOF_SERVER)
	{
		// Process success: return resHeader.val.dataSize
		return &resData.double6dArr[0];
	}
	double zeros[6] = { 0 };
	return zeros;
	LeaveCriticalSection(&DCP_cs);
}

double *CustomIndyDedicatedTCPTestClient::GetTaskVel() {
	EnterCriticalSection(&DCP_cs);

	// Get Task Velocity
	Run(CMD_GET_TASK_VELOCITY, 0);

	if (header.val.cmdId == resHeader.val.cmdId
		&& header.val.invokeId == resHeader.val.invokeId
		&& resHeader.val.sof == SOF_SERVER)
	{
		// Process success: return resHeader.val.dataSize
		return &resData.double6dArr[0];
	}
	double zeros[6] = { 0 };
	return zeros;
	LeaveCriticalSection(&DCP_cs);
}

double *CustomIndyDedicatedTCPTestClient::GetTorque() {
	EnterCriticalSection(&DCP_cs);
	// Get Task Velocity
	Run(CMD_GET_TORQUE, 0);

	if (header.val.cmdId == resHeader.val.cmdId
		&& header.val.invokeId == resHeader.val.invokeId
		&& resHeader.val.sof == SOF_SERVER)
	{
		// Process success: return resHeader.val.dataSize
		return &resData.double6dArr[0];
	}
	double zeros[6] = { 0 };
	return zeros;
	LeaveCriticalSection(&DCP_cs);
}

bool CustomIndyDedicatedTCPTestClient::isReady() {
	EnterCriticalSection(&DCP_cs);
	bool res = false;
	Run(CMD_IS_READY, 0);

	if (header.val.cmdId == resHeader.val.cmdId
		&& header.val.invokeId == resHeader.val.invokeId
		&& resHeader.val.sof == SOF_SERVER)
	{
		// Process success: return resHeader.val
		res = resData.boolVal;
	}
	LeaveCriticalSection(&DCP_cs);
	return res;
}

bool CustomIndyDedicatedTCPTestClient::isMoveFinished() {
	EnterCriticalSection(&DCP_cs);
	bool res = false;

	Run(CMD_IS_MOVE_FINISEHD, 0);

	if (header.val.cmdId == resHeader.val.cmdId
		&& header.val.invokeId == resHeader.val.invokeId
		&& resHeader.val.sof == SOF_SERVER)
	{
		// Process success: return resHeader.val
		res = resData.boolVal;
	}

	LeaveCriticalSection(&DCP_cs);
	return res;
}

void CustomIndyDedicatedTCPTestClient::GoHome(){
	EnterCriticalSection(&DCP_cs);
	// Go Home command		
	Run(CMD_MOVE_HOME, 0);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::GoZero(){
	EnterCriticalSection(&DCP_cs);
	// Go Zero command		
	Run(CMD_MOVE_ZERO, 0);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::StartDirectTeaching() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_CHANGE_DIRECT_TEACHING, 0);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::StopDirectTeaching() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_FINISH_DIRECT_TEACHING, 0);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::MoveByJ(int numJoint, char dir, float dist){
	EnterCriticalSection(&DCP_cs);
	double joints[6];
	for (int i = 0; i < 6; i++) {
		if (i == numJoint) {
			joints[i] = dist;
		}
		else {
			joints[i] = 0;
		}
	}

	double _dir;
	dir == 'P' ? _dir = 1 : _dir = -1;

	header.val.cmdId = CMD_JOINT_MOVE_BY;
	header.val.dataSize = sizeof(double)*6;

	for (int i = 0; i < 6; i++) {
		data.double6dArr[i] = joints[i] * _dir;
	}

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::MoveByT(double* coordinate)
{
	EnterCriticalSection(&DCP_cs);
	
	header.val.cmdId = CMD_TASK_MOVE_BY;
	header.val.dataSize = sizeof(double) * 6;

	for (int i = 0; i < 6; i++) {
		data.double6dArr[i] = coordinate[i];
	}

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::MoveToT(float *arr){
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_TASK_MOVE_TO;
	header.val.dataSize = sizeof(double)*6;

	for (int i = 0; i < 6; i++) {
		data.double6dArr[i] = arr[i];
	}

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::MoveToJ(float *arr){
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_JOINT_MOVE_TO;
	header.val.dataSize = sizeof(double)*6;

	for (int i = 0; i < 6; i++) {
		data.double6dArr[i] = arr[i];
	}

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::TrackTrajectory(std::string filename) {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_FOR_EXTENDED;
	header.val.dataSize = sizeof(int) * 2;
	
	int charLength = filename.length();

	data.intArr[0] = 4;
	data.intArr[1] = charLength+1;

	for (int i = 0; i < charLength; i++) {
		extData.charArr[i] = filename.c_str()[i];
	}
	extData.charArr[charLength] = '\0';

	Run(header.val.cmdId, header.val.dataSize, charLength+1);
	LeaveCriticalSection(&DCP_cs);
}

#ifdef TELEOPERATION
void CustomIndyDedicatedTCPTestClient::ToggleTeleoperationMode() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_TOGGLE_TELEOPERATION_MODE, 0);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::SetRefState(double *pos, double *vel) {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_SET_REF_POSE;
	header.val.dataSize = sizeof(double) * 12;

	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = pos[i];
	}
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i+6] = vel[i];
	}

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}
#endif
