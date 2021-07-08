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

	server_ip = "192.168.1.11";

	time_last_update = std::chrono::system_clock::now();
	for (int i = 0; i < 6; i++) {
		pose_last[i] = 0.0;
		velocity_last[i] = 0.0;
	}
	pose_updated = false;
	min_time_interval = 1.0 / 10.0;
	cut_off_freq = 1.0;

	InitializeCriticalSection(&DCP_cs);
};

CustomIndyDedicatedTCPTestClient::CustomIndyDedicatedTCPTestClient(const char* ip) :v_sockFd(-1), v_invokeId(0), v_working(true)
{
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	// userInput setting
	userInput.val.sof = SOF_CLIENT;

	server_ip = ip;

	time_last_update = std::chrono::system_clock::now();
	for (int i = 0; i < 6; i++) {
		pose_last[i] = 0.0;
		velocity_last[i] = 0.0;
	}
	pose_updated = false;
	min_time_interval = 1.0 / 10.0;
	cut_off_freq = 1.0;

	InitializeCriticalSection(&DCP_cs);
};

CustomIndyDedicatedTCPTestClient::~CustomIndyDedicatedTCPTestClient()
{
	WSACleanup();
	DeleteCriticalSection(&DCP_cs);
};

void CustomIndyDedicatedTCPTestClient::connect()
{
	connect(server_ip.c_str(), ROBOT_NAME);
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
	server_addr.sin_port = htons(CUSTOM_SERVER_PORT);

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

void CustomIndyDedicatedTCPTestClient::send_message()
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

void CustomIndyDedicatedTCPTestClient::recv_message()
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

bool CustomIndyDedicatedTCPTestClient::check_indy_state()
{
	if (userInput.val.invokeId != indyState.val.invokeId) {
		printf("Header check fail (invokeId): user_input {%d}, indy_state {%d}\r\n", userInput.val.invokeId, indyState.val.invokeId);
		return false;
	}
	else if (indyState.val.sof != SOF_SERVER) {
		printf("Header check fail (sof): Request %d, Response %d\r\n", SOF_SERVER, indyState.val.sof);
		return false;
	}
	else{
		return true;
	}
}

bool CustomIndyDedicatedTCPTestClient::communicate(int cmd)
{
	userInput.val.invokeId = v_invokeId++;
	userInput.val.command = cmd;
	userInput.val.sof = SOF_CLIENT;

	// Communication
	if (v_sockFd >= 0) {
		send_message();
		recv_message();
	}

	// Integrity check
	return check_indy_state();
}

void CustomIndyDedicatedTCPTestClient::ceiling(Eigen::Vector3d input, Eigen::Matrix3d& res) {
	res(0, 0) = 0.0;
	res(0, 1) = -input(2);
	res(0, 2) = input(1);
	res(1, 0) = input(2);
	res(1, 1) = 0.0;
	res(1, 2) = -input(0);
	res(2, 0) = -input(1);
	res(2, 1) = input(0);
	res(2, 2) = 0.0;
}

void CustomIndyDedicatedTCPTestClient::dexp(Eigen::Vector3d input, Eigen::Matrix3d& res) {
	Eigen::Matrix3d ceiling_, identity_;
	Eigen::Matrix3d comp1, comp2;
	double alpha_, beta_, inputNorm_;
	ceiling(input, ceiling_);
	identity_ = Eigen::Matrix3d::Identity();

	inputNorm_ = input.norm();
	if (inputNorm_ == 0.0) {
		res = identity_;
	}
	else {
		alpha_ = sin(inputNorm_ / 2.0) * cos(inputNorm_ / 2.0) / inputNorm_ * 2.0;
		beta_ = sin(inputNorm_ / 2.0) * sin(inputNorm_ / 2.0) / inputNorm_ * 2.0 / inputNorm_ * 2.0;

		comp1 = ceiling_;
		comp2 = ceiling_;
		comp1 *= 0.5 * beta_;
		comp2 *= 1.0 / inputNorm_ * (1.0 - alpha_);
		res = identity_ + comp1 + comp2 * ceiling_;
	}
}

void CustomIndyDedicatedTCPTestClient::rotation_from_axis_angle(Eigen::Vector3d input, Eigen::Matrix3d& res) {
	if (input.norm() > 0) {
		Eigen::AngleAxisd aa(input.norm(), input / input.norm());
		res = aa.toRotationMatrix();
	}
	else {
		res = Eigen::Matrix3d::Identity();
	}
}

void CustomIndyDedicatedTCPTestClient::axis_angle_from_rotation(Eigen::Matrix3d input, Eigen::Vector3d& res) {
	Eigen::AngleAxisd aa(input);
	res = aa.angle() * aa.axis();
}

void CustomIndyDedicatedTCPTestClient::estimate_velocity(double* pose, double* velocity) {
	std::chrono::system_clock::time_point time_curr = std::chrono::system_clock::now();
	std::chrono::duration<double> time_delta = time_curr - time_last_update;
	double dT = time_delta.count();
	if (dT > min_time_interval || !pose_updated) {
		time_last_update = time_curr;
		Eigen::Vector3d axis_angle;
		for (int i = 0; i < 3; i++) {
			pose_last[i] = pose[i];
			pose_last[i + 3] = 0.0;
			velocity_last[i] = 0.0;
			velocity_last[i + 3] = 0.0;
			velocity[i] = velocity_last[i];
			velocity[i + 3] = velocity_last[i + 3];
			axis_angle[i] = pose[i + 3];
		}
		rotation_from_axis_angle(axis_angle, R0);
		pose_updated = true;
	}
	else {
		double w = 2.0 * M_PI * cut_off_freq;
		Eigen::Vector3d linear_velocity;
		Eigen::Vector3d axis_angle;
		for (int i = 0; i < 3; i++) {
			linear_velocity[i] = 1.0 / (2.0 + dT * w) * ((2.0 - dT * w) * velocity_last[i] + 2.0 * w * (pose[i] - pose_last[i]));
			axis_angle[i] = pose[i + 3];
		}

		// convert xidot into w(angular velocity)
		Eigen::Matrix3d R_curr;
		rotation_from_axis_angle(axis_angle, R_curr);
		Eigen::Vector3d xi, xidot;
		axis_angle_from_rotation(R0.transpose() * R_curr, xi);
		for (int i = 0; i < 3; i++) {
			xidot[i] = 1.0 / (2.0 + dT * w) * ((2.0 - dT * w) * velocity_last[i + 3] + 2.0 * w * (xi[i] - pose_last[i + 3]));
			pose_last[i] = pose[i];
			pose_last[i + 3] = xi[i];
			velocity_last[i] = linear_velocity[i];
			velocity_last[i + 3] = xidot[i];
		}
		Eigen::Matrix3d dexp_xi;
		dexp(xi, dexp_xi);
		Eigen::Vector3d angular_velocity = dexp_xi.transpose() * xidot;
		time_last_update = time_curr;
		for (int i = 0; i < 3; i++) {
			velocity[i] = linear_velocity[i];
			velocity[i + 3] = angular_velocity[i];
		}
	}
}

void CustomIndyDedicatedTCPTestClient::get_robot_joint_state(int& cmode, double* joint_config) {
	double dummy1[6];
	double dummy2[6];
	get_robot_joint_state(cmode, joint_config, dummy1, dummy2);
}

void CustomIndyDedicatedTCPTestClient::get_robot_joint_state(int& cmode, double* joint_config, double* joint_velocity) {
	double dummy[6];
	get_robot_joint_state(cmode, joint_config, joint_velocity, dummy);
}

void CustomIndyDedicatedTCPTestClient::get_robot_joint_state(int& cmode, double* joint_config, double* joint_velocity, double* external_torque) {
	EnterCriticalSection(&DCP_cs);
	if (communicate(CUSTOM_TCP_COMMAND_GET_ROBOT_JOINT_STATE)) {
		cmode = indyState.val.indyCMode;
		for (int i = 0; i < 6; i++) {
			joint_config[i] = indyState.val.indyPos[i];
			joint_velocity[i] = indyState.val.indyVel[i];
			external_torque[i] = indyState.val.forceTorque[i];
		}
	}
	else {
		cmode = -1;
		for (int i = 0; i < 6; i++) {
			joint_config[i] = 0.0;
			joint_velocity[i] = 0.0;
			external_torque[i] = 0.0;
		}
	}
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::get_robot_task_state(int& cmode, double* task_pose) {
	double dummy1[6];
	double dummy2[6];
	get_robot_task_state(cmode, task_pose, dummy1, dummy2);
}

void CustomIndyDedicatedTCPTestClient::get_robot_task_state(int& cmode, double* task_pose, double* task_velocity) {
	double dummy[6];
	get_robot_task_state(cmode, task_pose, task_velocity, dummy);
}

void CustomIndyDedicatedTCPTestClient::get_robot_task_state(int& cmode, double* task_pose, double* task_velocity, double* external_wrench) {
	EnterCriticalSection(&DCP_cs);
	if (communicate(CUSTOM_TCP_COMMAND_GET_ROBOT_JOINT_STATE)) {
		cmode = indyState.val.indyCMode;
		for (int i = 0; i < 6; i++) {
			task_pose[i] = indyState.val.indyPos[i];
			task_velocity[i] = indyState.val.indyVel[i];
			external_wrench[i] = indyState.val.forceTorque[i];
		}
	}
	else {
		cmode = -1;
		for (int i = 0; i < 6; i++) {
			task_pose[i] = 0.0;
			task_velocity[i] = 0.0;
			external_wrench[i] = 0.0;
		}
	}
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::reset_robot() {
	EnterCriticalSection(&DCP_cs);
	communicate(CUSTOM_TCP_COMMAND_RESET_ROBOT);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::reset_ft_bias() {
	EnterCriticalSection(&DCP_cs);
	communicate(CUSTOM_TCP_COMMAND_RESET_FT_BIAS);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::start_direct_teaching() {
	EnterCriticalSection(&DCP_cs);
	communicate(CUSTOM_TCP_COMMAND_START_DIRECT_TEACHING);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::stop_direct_teaching() {
	EnterCriticalSection(&DCP_cs);
	communicate(CUSTOM_TCP_COMMAND_STOP_DIRECT_TEACHING);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::start_teleoperation() {
	EnterCriticalSection(&DCP_cs);
	pose_updated = false;
	communicate(CUSTOM_TCP_COMMAND_START_TELEOPERATION);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::stop_teleoperation() {
	EnterCriticalSection(&DCP_cs);
	communicate(CUSTOM_TCP_COMMAND_STOP_TELEOPERATION);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::go_home() {
	EnterCriticalSection(&DCP_cs);
	communicate(CUSTOM_TCP_COMMAND_GO_HOME);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::go_zero() {
	EnterCriticalSection(&DCP_cs);
	communicate(CUSTOM_TCP_COMMAND_GO_ZERO);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::joint_move_to(double * target_config, bool deg=false) {
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		if (deg) {
			userInput.val.targetPos[i] = (int)(target_config[i]/180.0*M_PI * 100000.0);
		}
		else {
			userInput.val.targetPos[i] = (int)(target_config[i] * 100000.0);
		}
		userInput.val.targetVel[i] = 0.0;
		userInput.val.targetAcc[i] = 0.0;
	}
	communicate(CUSTOM_TCP_COMMAND_JOINT_MOVE_TO);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::task_move_to(double* target_pose) {
	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		userInput.val.targetPos[i] = (int)(target_pose[i] * 100000.0);
		userInput.val.targetVel[i] = 0.0;
		userInput.val.targetAcc[i] = 0.0;
	}
	communicate(CUSTOM_TCP_COMMAND_TASK_MOVE_TO);
	LeaveCriticalSection(&DCP_cs);
}

void CustomIndyDedicatedTCPTestClient::update_teleoperation_command(double* target_pose, int& cmode, double& curr_time_constant, double* task_pose, double* task_velocity, double* external_wrench)
{
	// targe_pose, indyPos: [position in m, axisangle in rad]
	// target_velocity, indyVel: [velocity in m / s, angular velocity in rad / s]
	// target_acceleration : [acceleration in m / s2, angular acceleration in rad / s2]
	// cmode : current Indy cmode(ex - 0: stationary, 1 : joint move, ..., 20 : teleoperation mode)

	update_teleoperation_command(target_pose, 10.0, cmode, curr_time_constant, task_pose, task_velocity, external_wrench);
}

void CustomIndyDedicatedTCPTestClient::update_teleoperation_command(double* target_pose, double time_constant, int& cmode, double& curr_time_constant, double* task_pose, double* task_velocity, double* external_wrench)
{
	// targe_pose, indyPos: [position in m, axisangle in rad]
	// target_velocity, indyVel: [velocity in m / s, angular velocity in rad / s]
	// target_acceleration : [acceleration in m / s2, angular acceleration in rad / s2]
	// cmode : current Indy cmode(ex - 0: stationary, 1 : joint move, ..., 20 : teleoperation mode)

	double target_velocity[6];
	estimate_velocity(target_pose, target_velocity);

	update_teleoperation_command(target_pose, target_velocity, time_constant, cmode, curr_time_constant, task_pose, task_velocity, external_wrench);
}

void CustomIndyDedicatedTCPTestClient::update_teleoperation_command(double* target_pose, double* target_velocity, double time_constant, int& cmode, double& curr_time_constant, double* task_pose, double* task_velocity, double* external_wrench)
{
	// targe_pose, indyPos: [position in m, axisangle in rad]
	// target_velocity, indyVel: [velocity in m / s, angular velocity in rad / s]
	// target_acceleration : [acceleration in m / s2, angular acceleration in rad / s2]
	// cmode : current Indy cmode(ex - 0: stationary, 1 : joint move, ..., 20 : teleoperation mode)

	double target_acceleration[6];
	for (int i = 0; i < 6; i++) {
		target_acceleration[i] = 0.0;
	}

	update_teleoperation_command(target_pose, target_velocity, target_acceleration, time_constant, cmode, curr_time_constant, task_pose, task_velocity, external_wrench);
}


void CustomIndyDedicatedTCPTestClient::update_teleoperation_command(double* target_pose, double* target_velocity, double* target_acceleration, double time_constant, int& cmode, double& curr_time_constant, double* task_pose, double* task_velocity, double* external_wrench)
{
	// targe_pose, indyPos: [position in m, axisangle in rad]
	// target_velocity, indyVel: [velocity in m / s, angular velocity in rad / s]
	// target_acceleration : [acceleration in m / s2, angular acceleration in rad / s2]
	// cmode : current Indy cmode(ex - 0: stationary, 1 : joint move, ..., 20 : teleoperation mode)

	EnterCriticalSection(&DCP_cs);
	for (int i = 0; i < 6; i++) {
		userInput.val.targetPos[i] = (int)(target_pose[i] * 100000.0);
		userInput.val.targetVel[i] = (int)(target_velocity[i] * 100000.0);
		userInput.val.targetAcc[i] = 0;
	}
	userInput.val.targetTimeConstant = (int)(time_constant * 100000.0);
	if (communicate(CUSTOM_TCP_COMMAND_UPDATE_TELEOPERATION_TRAJECTORY)) {
		cmode = indyState.val.indyCMode;
		curr_time_constant = indyState.val.targetTimeConstant;
		for (int i = 0; i < 6; i++) {
			task_pose[i] = (double)indyState.val.indyPos[i] / 100000.0;
			task_velocity[i] = (double)indyState.val.indyVel[i] / 100000.0;
			external_wrench[i] = (double)indyState.val.forceTorque[i] / 100000.0;
		}
	}
	else {
		cmode = -1;
		curr_time_constant = 0.0;
		for (int i = 0; i < 6; i++) {
			task_pose[i] = 0.0;
			task_velocity[i] = 0.0;
			external_wrench[i] = 0.0;
		}
	}
	LeaveCriticalSection(&DCP_cs);
}
