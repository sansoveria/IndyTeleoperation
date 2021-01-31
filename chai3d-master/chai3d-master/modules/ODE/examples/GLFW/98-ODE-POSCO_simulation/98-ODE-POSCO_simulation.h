#pragma once
#define SIMULATION
#define FENCE

#ifndef SIMULATION
#include <winsock2.h>
#include <windows.h>
#endif
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#include "CODE.h"
#include "CODEWorldFurnace.h"		// customized ODE world for furnace
#include "CODEUtils.h"
//------------------------------------------------------------------------------
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <ctime>

#ifndef SIMULATION
#include "NRMK_IndyDCPClient.h"
#include "Demo_setup.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// INDY SETTINGS
//------------------------------------------------------------------------------
#ifndef SIMULATION
double pRef1[6] = { 0.605, 0.076, 0.221, 0.0, 180.000, 0.000, };
double pdotRef1[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double pRef2[6] = { 0.455, 0.076, 0.221, 0.0, 180.000, 0.000, };
double pdotRef2[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
IndyDedicatedTCPTestClient indyTCP;
int indy_cmode = 0;
#endif

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
	C_STEREO_DISABLED:            Stereo is disabled
	C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
	C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
	C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
string resourceRoot;
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------

cWorld* world;
cCamera* camera;
cSpotLight* light;
cHapticDeviceHandler* handler;
shared_ptr<cGenericHapticDevice> hapticDevice;

// a label to display the rate [Hz] at which the simulation is running
cPanel* panel;
cLabel* labelRates;
cLabel* labelStatus;
cLabel* labelSuccess;
cLabel* labelFailure;
cLabel* labelForce;
cLabel* labelTorque;
cLevel* levelForce;
cLevel* levelTorque;

double workspaceScaleFactor = 100.0;

#define TIMESTEP		0.001

// stiffness of virtual spring
double linGain = 0.01;
double angGain = 0.0;
double linG;
double angG;
double linStiffness = 2000;
double angStiffness = 50;
double cube_size = 0.4;
#define NUM_STONE           1
int num_stone_remain, num_success, num_failure;
cVector3d stone_init_pos;
cMatrix3d stone_init_rot;
cVector3d tool_init_pos;
cMatrix3d tool_init_rot;
cVector3d starting_pos;
cMaterial matBlue, matRed;

bool MISSION_SUCCEEDED, MISSION_FAILED, MISSION_STARTED;

cLowPassFilter LPFSensor;
cFilteredDerivative devicePosDerivative;
cFilteredDerivative toolPosDerivative;
cFilteredDerivative toolRotDerivative;

//---------------------------------------------------------------------------
// STATE VARIABLES
//---------------------------------------------------------------------------
cVector3d robot_force, robot_torque, render_force;
cVector3d posDevice, posTool, posStone;
cMatrix3d rotDevice, rotTool, rotStone;
cVector3d velDevice;
double timeSystem;
int loopIdx;
int collisionSet;

//---------------------------------------------------------------------------
// LOGGING BUFFERS
//---------------------------------------------------------------------------
#define LOGGING_BUFFER_SIZE							60000 // 60*1000
#define LOGGING_DIRECTORY							"../../../logging_data/data/"

bool SAVE_LOG;

double logTime[LOGGING_BUFFER_SIZE];					// 1
double logPosDevice[LOGGING_BUFFER_SIZE][3];			// 2-4
double logPosTool[LOGGING_BUFFER_SIZE][3];				// 8-10
double logPosStone[LOGGING_BUFFER_SIZE][3];				// 14-16
double logRotDevice[LOGGING_BUFFER_SIZE][3];			// 5-7
double logRotTool[LOGGING_BUFFER_SIZE][3];				// 11-13
double logRotStone[LOGGING_BUFFER_SIZE][3];				// 17-19
double logForceRobot[LOGGING_BUFFER_SIZE][3];			// 20-22
double logTorqueRobot[LOGGING_BUFFER_SIZE][3];			// 23-25
int logCollisionSet[LOGGING_BUFFER_SIZE];				// 26
double logForceFeedback[LOGGING_BUFFER_SIZE][3];		// 27-29

//---------------------------------------------------------------------------
// ODE MODULE VARIABLES
//---------------------------------------------------------------------------

// ODE world
cODEWorldFurnace* ODEWorld;

// World ground
cODEGenericBody* ODEGround;

// ODE objects (static)
cODEGenericBody* ODEFence;				// collision bit 0
cODEGenericBody* ODEFence_side1;		// collision bit 1
cODEGenericBody* ODEFence_side2;		// collision bit 2
cODEGenericBody* ODELane1;				// collision bit 3
cODEGenericBody* ODELane2;				// collision bit 4
cODEGenericBody* ODESidewalk1;			// collision bit 5
cODEGenericBody* ODESidewalk2;			// collision bit 6
cODEGenericBody* ODELava;				// collision bit 7
cODEGenericBody* ODEFurnace;			// collision bit 8

// ODE objects (dynamic)
cODEGenericBody* ODETool;				
cODEGenericBody* ODEToolHandle;			
cODEGenericBody* ODEStone[NUM_STONE];	// collision bit 10 -


// Meshes for visualization
//cMesh* cursor_handle = new cMesh();
cMesh* cursor_endtip = new cMesh();
cMesh* starting_pad = new cMesh();

cMesh* tool_mesh = new cMesh();
cMesh* tool_handle_mesh = new cMesh();
#ifdef FENCE
cMesh* fence_mesh = new cMesh();
cMesh* fence_side1_mesh = new cMesh();
cMesh* fence_side2_mesh = new cMesh();
#endif
cMesh* lane1_mesh = new cMesh();
cMesh* lane2_mesh = new cMesh();
cMesh* sidewalk1_mesh = new cMesh();
cMesh* sidewalk2_mesh = new cMesh();
cMesh* furnace_mesh = new cMesh();
cMesh* ground = new cMesh();

// Virtual FT sensor
dJointGroupID sensorGroup;
dJointID FTSensor;
dJointFeedback FTSensorMeasure;

//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation and util thread has terminated
bool simulationFinished = false;
bool utilityThreadFinished = false;

// a frequency counter
cFrequencyCounter freqCounterGraphics;
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// util thread (data acquisition / Indy control)
cThread* utilThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width and height of window
int width = 0;
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);
void errorCallback(int error, const char* a_description);
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

void updateGraphics(void);
void updateHaptics(void);
void updateUtils(void);

// this function closes the application
void close(void);
