#include "pch.h"
#include "NRMK_IndyDCPClient.h"
#include <stdio.h>



#define SOF_SERVER  0x12
#define SOF_CLIENT  0x34


IndyDedicatedTCPTestClient::IndyDedicatedTCPTestClient() :v_sockFd(-1), v_invokeId(0), v_working(true)
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

IndyDedicatedTCPTestClient::~IndyDedicatedTCPTestClient()
{
	WSACleanup();
	DeleteCriticalSection(&DCP_cs);
};


void IndyDedicatedTCPTestClient::connect()
{
	connect(SERVER_IP, ROBOT_NAME);
}

void IndyDedicatedTCPTestClient::connect(const char* serverip, const char* robotname) {

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

void IndyDedicatedTCPTestClient::disconnect()
{
	closesocket(v_sockFd);
	v_message = ("Socket Disconnected\n");
}

void IndyDedicatedTCPTestClient::SendHeader()
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

void IndyDedicatedTCPTestClient::SendData()
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

void IndyDedicatedTCPTestClient::SendExtData(int extDataSize)
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

void IndyDedicatedTCPTestClient::ReadHeader()
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
	//message = ("IndyDedicatedTCPTestClient : Read response header \n");
}
void IndyDedicatedTCPTestClient::ReadData()
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
		//printf("IndyDedicatedTCPTestClient : Read response data\n");
	}
}

void IndyDedicatedTCPTestClient::Run(int cmdID, int dataSize, int extDataSize)
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

void IndyDedicatedTCPTestClient::Run(int cmdID, int dataSize)
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

void IndyDedicatedTCPTestClient::Run(int cmdID)
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

double *IndyDedicatedTCPTestClient::GetJointPos() {
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

double *IndyDedicatedTCPTestClient::GetJointVel() {
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

double *IndyDedicatedTCPTestClient::GetTaskPos() {
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

double *IndyDedicatedTCPTestClient::GetTaskVel() {
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

double *IndyDedicatedTCPTestClient::GetTorque() {
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

bool IndyDedicatedTCPTestClient::isReady() {
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

bool IndyDedicatedTCPTestClient::isMoveFinished() {
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

void IndyDedicatedTCPTestClient::GoHome(){
	EnterCriticalSection(&DCP_cs);
	// Go Home command		
	Run(CMD_MOVE_HOME, 0);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::GoZero(){
	EnterCriticalSection(&DCP_cs);
	// Go Zero command		
	Run(CMD_MOVE_ZERO, 0);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::StartDirectTeaching() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_CHANGE_DIRECT_TEACHING, 0);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::StopDirectTeaching() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_FINISH_DIRECT_TEACHING, 0);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::MoveByJ(int numJoint, char dir, float dist){
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

void IndyDedicatedTCPTestClient::MoveByT(double* coordinate)
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

void IndyDedicatedTCPTestClient::MoveToT(float *arr){
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_TASK_MOVE_TO;
	header.val.dataSize = sizeof(double)*6;

	for (int i = 0; i < 6; i++) {
		data.double6dArr[i] = arr[i];
	}

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::MoveToJ(float *arr){
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_JOINT_MOVE_TO;
	header.val.dataSize = sizeof(double)*6;

	for (int i = 0; i < 6; i++) {
		data.double6dArr[i] = arr[i];
	}

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::TrackTrajectory(std::string filename) {
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
void IndyDedicatedTCPTestClient::ToggleTeleoperationMode() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_TOGGLE_TELEOPERATION_MODE, 0);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::SetRefState(double *pos, double *vel) {
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

/* References (Custom functions) */
/*
//DH_start
#if defined(_DH_PROGRAM_SETUP)
void IndyDedicatedTCPTestClient::InitProgram() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_INIT_CUSTOM_PROGRAM, 0);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::AddProgramMoveHome() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_ADD_MOVEHOME_CUSTOM_PROGRAM, 0);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::AddProgramJointMoveTo(double *q, int velLevel) {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_ADD_JOINTMOVETO_CUSTOM_PROGRAM;
	header.val.dataSize = sizeof(double) * 6 + sizeof(int);

	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = q[i];
	}
	data.intArr[2 * 6] = velLevel;

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::AddProgramTaskMoveTo(double *p, int velLevel) {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_ADD_TASKMOVETO_CUSTOM_PROGRAM;
	header.val.dataSize = sizeof(double) * 6 + sizeof(int);

	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = p[i];
	}
	data.intArr[2 * 6] = velLevel;

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::AddProgramTaskMoveBy(double *p, int velLevel) {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_ADD_TASKMOVEBY_CUSTOM_PROGRAM;
	header.val.dataSize = sizeof(double) * 6 + sizeof(int);

	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = p[i];
	}
	data.intArr[2 * 6] = velLevel;

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::AddProgramWait(double time) {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_ADD_WAIT_CUSTOM_PROGRAM;
	header.val.dataSize = 8;

	data.doubleVal = time;

	Run(CMD_ADD_WAIT_CUSTOM_PROGRAM, 8);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::AddProgramDOControl(int idx, bool on) {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_ADD_DOCONTROL_CUSTOM_PROGRAM;
	header.val.dataSize = 5;

	data.intVal = idx;
	data.str[4] = on;
	Run(CMD_ADD_DOCONTROL_CUSTOM_PROGRAM, 8);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::AddProgramToolCommand(int toolId, int command) {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_ADD_TOOLCOMMAND_CUSTOM_PROGRAM;
	header.val.dataSize = 2 * sizeof(int);
	data.intArr[0] = toolId;
	data.intArr[1] = command;
	Run(header.val.cmdId, header.val.dataSize);
	
	//header.val.dataSize = 11 * sizeof(int);

	//data.intArr[0] = idx;
	//for (int i = 0; i < 10; i++) {
	//	data.intArr[i + 1] = cmd[i];
	//}
	//Run(CMD_ADD_TOOLCOMMAND_CUSTOM_PROGRAM, 11 * sizeof(int));
	
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::SetProgram() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_SET_CUSTOM_PROGRAM, 0);
	cout << "SetProgram" << endl;
	cout << header.val.cmdId << ',' << header.val.invokeId << ',' << SOF_SERVER << ',' << endl;
	cout << resHeader.val.cmdId << ',' << resHeader.val.invokeId << ',' << resHeader.val.sof << ',' << endl;
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::StartProgram() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_START_CURRENT_PROGRAM, 0);
	cout << "StartProgram" << endl;
	cout << header.val.cmdId << ',' << header.val.invokeId << ',' << SOF_SERVER << ',' << endl;
	cout << resHeader.val.cmdId << ',' << resHeader.val.invokeId << ',' << resHeader.val.sof << ',' << endl;
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::PauseProgram() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_PAUSE_CURRENT_PROGRAM, 0);
}

void IndyDedicatedTCPTestClient::ResumeProgram() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_RESUME_CURRENT_PROGRAM, 0);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::ClearProgram() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_CLEAR_CUSTOM_PROGRAM, 0);
	LeaveCriticalSection(&DCP_cs);
}
void IndyDedicatedTCPTestClient::StopProgram() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_STOP_CURRENT_PROGRAM, 0);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::RunDemoPickProgram(
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
){	
	// PickProgram1
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = pre_pick_cover[i];
		data.doubleArr[i + 6] = pickcover_above[i];
		data.doubleArr[i + 12] = pickcover[i];
	}
	data.doubleArr[18] = waiting_time;

	data.intArr[2 * 19] = motion_velocity;
	data.intArr[2 * 19 + 1] = DO_port_num;
	data.intArr[2 * 19 + 2] = tool_index;
	data.intArr[2 * 19 + 3] = tool_open;

	header.val.cmdId = CMD_DEMO_PICK_PROGRAM1;
	header.val.dataSize = (sizeof(double) * 19 + sizeof(int) * 4);

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);

	// PickProgram2
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = lift_up[i];
		data.doubleArr[i+6] = updown_cover[i];
	}

	data.intArr[2 * 12] = motion_velocity;

	header.val.cmdId = CMD_DEMO_PICK_PROGRAM2;
	header.val.dataSize = (sizeof(double) * 12 + sizeof(int) * 1);

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::RunDemoPutProgram(
	double* placecover,
	double* liftdown,
	double* move_side,
	double* liftup,
	double waiting_time,
	int motion_velocity,
	int DO_port_num
) {
	// PutProgram1
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = placecover[i];
		data.doubleArr[i + 6] = liftdown[i];
	}
	data.doubleArr[12] = waiting_time;

	data.intArr[2 * 13] = motion_velocity;
	data.intArr[2 * 13 + 1] = DO_port_num;

	header.val.cmdId = CMD_DEMO_PUT_PROGRAM1;
	header.val.dataSize = (sizeof(double) * 13 + sizeof(int) * 2);

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);

	// PutProgram2
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = move_side[i];
		data.doubleArr[i + 6] = liftup[i];
	}
	data.intArr[2 * 12] = motion_velocity;

	header.val.cmdId = CMD_DEMO_PUT_PROGRAM2;
	header.val.dataSize = (sizeof(double) * 12 + sizeof(int) * 1);

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::RunDemoGripProgram(
	double* pre_pickcover_holder,
	double* liftdown_to_grip,
	double* liftup,
	double* pre_assemble,
	double waiting_time,
	int motion_velocity,
	int tool_index,
	int tool_open,
	int tool_close
) {
	// PutProgram1
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = pre_pickcover_holder[i];
		data.doubleArr[i + 6] = liftdown_to_grip[i];
	}
	data.doubleArr[12] = waiting_time;

	data.intArr[2 * 13] = motion_velocity;
	data.intArr[2 * 13 + 1] = tool_index;
	data.intArr[2 * 13 + 2] = tool_open;
	data.intArr[2 * 13 + 3] = tool_close;

	header.val.cmdId = CMD_DEMO_GRIP_PROGRAM1;
	header.val.dataSize = (sizeof(double) * 13 + sizeof(int) * 4);

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);

	// PutProgram2
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = liftup[i];
		data.doubleArr[i + 6] = pre_assemble[i];
	}
	data.intArr[2 * 12] = motion_velocity;

	header.val.cmdId = CMD_DEMO_GRIP_PROGRAM2;
	header.val.dataSize = (sizeof(double) * 12 + sizeof(int) * 1);

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::SetDemoAssembleProgram(
	double* pre_assemble_1, 
	double* pre_assemble_2, 
	int motion_velocity
) {
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = pre_assemble_1[i];
		data.doubleArr[i + 6] = pre_assemble_2[i];
	}
	data.intArr[2 * 12] = motion_velocity;

	header.val.cmdId = CMD_DEMO_SET_ASSEMBLE_PROGRAM;
	header.val.dataSize = (sizeof(double) * 12 + sizeof(int) * 1);

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::RunDemoAssembleProgram() {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_DEMO_RUN_ASSEMBLE_PROGRAM;
	header.val.dataSize = 0;

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::SetDemoRobot1AfterAssembleProgram(
	double* put_pose,
	double waiting_time,
	int motion_velocity,
	int tool_index,
	int tool_open
) {
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = put_pose[i];
	}
	data.doubleArr[6] = waiting_time;

	data.intArr[2 * 7] = motion_velocity;
	data.intArr[2 * 7 + 1] = tool_index;
	data.intArr[2 * 7 + 2] = tool_open;

	header.val.cmdId = CMD_DEMO_SET_AFTER_ASSEMBLE_ROBOT1_PROGRAM;
	header.val.dataSize = (sizeof(double) * 7 + sizeof(int) * 3);

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::SetDemoRobot2AfterAssembleProgram(
	double* put_pose,
	double waiting_time_1,
	double waiting_time_2,
	int motion_velocity,
	int tool_index,
	int tool_open
) {
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		data.doubleArr[i] = put_pose[i];
	}
	data.doubleArr[6] = waiting_time_1;
	data.doubleArr[7] = waiting_time_2;

	data.intArr[2 * 8] = motion_velocity;
	data.intArr[2 * 8 + 1] = tool_index;
	data.intArr[2 * 8 + 2] = tool_open;

	header.val.cmdId = CMD_DEMO_SET_AFTER_ASSEMBLE_ROBOT2_PROGRAM;
	header.val.dataSize = (sizeof(double) * 8 + sizeof(int) * 3);

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::RunDemoAfterAssembleProgram() {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_DEMO_RUN_AFTER_ASSEMBLE_PROGRAM;
	header.val.dataSize = 0;

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

bool IndyDedicatedTCPTestClient::isProgramRunning() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_IS_PROGRAM_RUNNING, 0);
	bool res = true;
	if (header.val.cmdId == resHeader.val.cmdId
		&& header.val.invokeId == resHeader.val.invokeId
		&& resHeader.val.sof == SOF_SERVER)
	{
		// Process success: return resHeader.val
		res = resData.boolVal;
	}
	cout << "IsProgramRunning" << endl;
	cout << header.val.cmdId << ',' << header.val.invokeId << ',' << SOF_SERVER << ',' << endl;
	cout << resHeader.val.cmdId << ',' << resHeader.val.invokeId << ',' << resHeader.val.sof << ',' << endl;
	LeaveCriticalSection(&DCP_cs);
	return res;
}

bool IndyDedicatedTCPTestClient::WaitForProgramFinish(){	
	while (true){
		EnterCriticalSection(&DCP_cs);
		if (!isProgramRunning())
			break;
		
		LeaveCriticalSection(&DCP_cs);
		
		Sleep(50);
	}
	LeaveCriticalSection(&DCP_cs);	
	return true;
}
#endif //_DH_PROGRAM_SETUP

#ifdef NULL_MOTION
bool IndyDedicatedTCPTestClient::StartNullMotion(double amp) {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_START_NULL_MOTION;
	header.val.dataSize = sizeof(double);
	data.doubleVal = amp;

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
	return true;
}

void IndyDedicatedTCPTestClient::StopNullMotion() {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_STOP_NULL_MOTION;
	header.val.dataSize = 0;

	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
	return true;
}
#endif

#ifdef TASK_SPACE_CONTROL_OFFLINE
bool IndyDedicatedTCPTestClient::SetOperationalMotionTargetPointListFile(std::string filename, double velocity, double joint_accel) {
	EnterCriticalSection(&DCP_cs);
	int char_length = filename.length();
	data.doubleArr[0] = velocity;
	data.doubleArr[1] = joint_accel;
	data.intArr[4] = char_length;
	for (int i = 0; i < char_length; i++) {
		data.charArr[20 + i] = filename.c_str()[i];
	}

	header.val.cmdId = CMD_TRACK_GIVEN_POINT_LIST;
	header.val.dataSize = sizeof(double) * 2 + sizeof(int) * 1 + sizeof(char)*char_length;
	Run(header.val.cmdId, header.val.dataSize);

	LeaveCriticalSection(&DCP_cs);
	return true;
}

bool IndyDedicatedTCPTestClient::InitPointList() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_INIT_POINT_LIST, 0);
	LeaveCriticalSection(&DCP_cs);

	return true;
}

bool IndyDedicatedTCPTestClient::AddCurrentConfigToPointList() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_ADD_POINT_TO_POINT_LIST, 0);
	LeaveCriticalSection(&DCP_cs);

	return true;
}

bool IndyDedicatedTCPTestClient::DeleteLastPoint() {
	EnterCriticalSection(&DCP_cs);
	Run(CMD_DELETE_LAST_POINT, 0);
	LeaveCriticalSection(&DCP_cs);

	return true;
}

bool IndyDedicatedTCPTestClient::SavePointList(std::string filename) {
	EnterCriticalSection(&DCP_cs);
	int char_length = filename.length();
	data.intArr[0] = char_length;
	for (int i = 0; i < char_length; i++) {
		data.charArr[4 + i] = filename.c_str()[i];
	}

	header.val.cmdId = CMD_SAVE_POINT_LIST;
	header.val.dataSize = sizeof(double) * 1 + sizeof(int) * 1 + sizeof(char)*char_length;
	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);

	return true;
}
#endif

#ifdef TASK_SPACE_CONTROL
void IndyDedicatedTCPTestClient::SetFDCCGains(double k, double kd, double psi_width, double fp_thres) {
	EnterCriticalSection(&DCP_cs);
	data.doubleArr[0] = k;
	data.doubleArr[1] = kd;
	data.doubleArr[2] = psi_width;
	data.doubleArr[3] = fp_thres;
	header.val.cmdId = CMD_SET_FDCC_GAINS;
	header.val.dataSize = sizeof(double) * 4;
	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::ToggleAdmittanceControl() {
	EnterCriticalSection(&DCP_cs);
	header.val.cmdId = CMD_TOGGLE_CMODE_ADMITTANCE_CONTROL;
	header.val.dataSize = 0;
	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}

void IndyDedicatedTCPTestClient::SetComplianceParameters(double M, double K) {
	EnterCriticalSection(&DCP_cs);
	data.doubleArr[0] = M;
	data.doubleArr[1] = K;
	header.val.cmdId = CMD_SET_COMPLIANCE_PARAMETERS;
	header.val.dataSize = sizeof(double) * 2;
	Run(header.val.cmdId, header.val.dataSize);
	LeaveCriticalSection(&DCP_cs);
}
#endif
//DH_end
*/