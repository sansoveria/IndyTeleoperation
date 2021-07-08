#pragma once

#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
	#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
	#define _CRT_SECURE_NO_WARNINGS
#endif 

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <chrono>
#include <math.h>

#include <winsock2.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib") //for winsock2

#include "Eigen/Eigen"


#define MSG_WAITALL 0x8
#include <iostream>
using namespace std;

#ifndef ROBOT_NAME
#define ROBOT_NAME "NRMK-Indy7"
#endif

#define CUSTOM_SERVER_PORT 6067			//!!!edit

#define CUSTOM_TCP_COMMAND_GET_ROBOT_JOINT_STATE				0
#define CUSTOM_TCP_COMMAND_GET_ROBOT_TASK_STATE					1

#define CUSTOM_TCP_COMMAND_RESET_ROBOT							10
#define CUSTOM_TCP_COMMAND_RESET_FT_BIAS						11

#define CUSTOM_TCP_COMMAND_START_DIRECT_TEACHING				21
#define CUSTOM_TCP_COMMAND_STOP_DIRECT_TEACHING					22
#define CUSTOM_TCP_COMMAND_START_TELEOPERATION					23
#define CUSTOM_TCP_COMMAND_STOP_TELEOPERATION					24

#define CUSTOM_TCP_COMMAND_GO_HOME								30
#define CUSTOM_TCP_COMMAND_GO_ZERO								31
#define CUSTOM_TCP_COMMAND_JOINT_MOVE_TO						32
#define CUSTOM_TCP_COMMAND_TASK_MOVE_TO							33
#define CUSTOM_TCP_COMMAND_UPDATE_TELEOPERATION_TRAJECTORY		34



class CustomIndyDedicatedTCPTestClient //: public Poco::Runnable
{
public:
	enum {
		SIZE_USER_INPUT = 85,
		SIZE_INDY_STATE = 85,
	};

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
	struct CustomUserInputStruct {					// 85 byte
		unsigned int invokeId;						// message ID (4 byte)
		unsigned int command;						// command (4 byte)
		int targetTimeConstant;						// tracking error decaying rate * 100000 (4 byte)
		int targetPos[6];							// target position in 10um and axis angle in 10^-5 rad (24 byte)
		int targetVel[6];							// target velocity in 10um and angular velocity in 10^-5 rad/s (24 byte)
		int targetAcc[6];							// target acceleration in 10um and angular acceleration in 10^-5 rad/s2 (24 byte)
		unsigned char sof;							// source of Frame (1 byte)
	};

	struct CustomIndyStateStruct {					// 85 byte
		unsigned int invokeId;						// message ID which is same with userinput (4 byte)
		unsigned int indyCMode;						// current indy control mode (4 byte)
		int targetTimeConstant;						// tracking error decaying rate * 100000 (4 byte)
		int indyPos[6];								// current indy position in 10um and axis angle in 10^-5 rad (24 byte)
		int indyVel[6];								// current indy velocity in 10um and angular velocity in 10^-5 rad/s (24 byte)
		int forceTorque[6];							// force value in 10 uN and torque in uNm (24 byte)
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
	CustomIndyDedicatedTCPTestClient(const char* serverip);
	~CustomIndyDedicatedTCPTestClient();

	/*
	*@breif	Thread function
	*/
	CRITICAL_SECTION DCP_cs;
	void connect();
	void connect(const char* serverip, const char* robotname);
	void disconnect();

	void set_velocity_cutoff_freq(double cutoff_freq) { cut_off_freq = cutoff_freq; }
	void set_min_time_interval(double min_interval) { min_time_interval = min_interval; }

	void get_robot_joint_state(int& cmode, double* joint_config);
	void get_robot_joint_state(int& cmode, double* joint_config, double* joint_velocity);
	void get_robot_joint_state(int& cmode, double* joint_config, double* joint_velocity, double* external_torque);

	void get_robot_task_state(int& cmode, double* task_pose);
	void get_robot_task_state(int& cmode, double* task_pose, double* task_velocity);
	void get_robot_task_state(int& cmode, double* task_pose, double* task_velocity, double* external_wrench);

	void reset_robot();
	void reset_ft_bias();

	void start_direct_teaching();
	void stop_direct_teaching();
	void start_teleoperation();
	void stop_teleoperation();

	void go_home();
	void go_zero();
	void joint_move_to(double* target_config, bool deg);
	void task_move_to(double* target_pose);

	void update_teleoperation_command(double* target_pose, int& cmode, double& curr_time_constant, double* task_pose, double* task_velocity, double* external_wrench);
	void update_teleoperation_command(double* target_pose, double time_constant, int& cmode, double& curr_time_constant, double* task_pose, double* task_velocity, double* external_wrench);
	void update_teleoperation_command(double* target_pose, double* target_velocity, double time_constant, int& cmode, double& curr_time_constant, double* task_pose, double* task_velocity, double* external_wrench);
	void update_teleoperation_command(double* target_pose, double* target_velocity, double* target_acceleration, double time_constant, int& cmode, double& curr_time_constant, double* task_pose, double* task_velocity, double* external_wrench);


private:
	string server_ip;
	bool v_working;
	int v_sockFd;
	int v_invokeId;
	char v_inchar;

	string v_message;
	char v_data[256];
	bool v_dataChanged = false;

	// state variables
	std::chrono::system_clock::time_point time_last_update;
	double pose_last[6];
	double velocity_last[6];
	bool pose_updated;
	double min_time_interval;
	double cut_off_freq;
	Eigen::Matrix3d R0;

	// Header and Data
	CustomUserInput userInput;
	CustomIndyState indyState;

	unsigned char readBuff[1024];
	unsigned char writeBuff[1024];

	// Send and Response functions
	string getv_message() { return this->v_message; }
	char* getv_data() { return this->v_data; }
	bool getv_dataChanged() { return this->v_dataChanged; }
	void setv_inchar(char c) { this->v_inchar = c; }
	void setv_dataChanged(bool b) { this->v_dataChanged = b; }

	void send_message();
	void recv_message();
	bool check_indy_state();
	bool communicate(int cmd);

	void ceiling(Eigen::Vector3d xi, Eigen::Matrix3d& res);
	void dexp(Eigen::Vector3d input, Eigen::Matrix3d& res);
	void axis_angle_from_rotation(Eigen::Matrix3d input, Eigen::Vector3d& res);
	void rotation_from_axis_angle(Eigen::Vector3d input, Eigen::Matrix3d& res);
	void estimate_velocity(double* pose, double* velocity);
}; /* end of class */