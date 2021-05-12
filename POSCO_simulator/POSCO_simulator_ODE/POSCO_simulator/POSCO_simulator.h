#pragma once

//---------------------------------------------------------------------------
#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

#include "chai3d.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CODE.h"

#include "definitions.h"

#ifdef USE_INDY
#include "cIndyFurnace.h"
#else
#include "cODEFurnace.h"
#include "cBulletFurnace.h"

#include "CSerialComm.h"
#endif
#include "CCustomFurnaceDevice.h"
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// CONTROL VARIABLES
//---------------------------------------------------------------------------
#ifdef USE_INDY
    double workspaceScaleFactor = 5.0;
#else
    double workspaceScaleFactor = 50.0;
#endif
double linGain = 0.05;
double angGain = 0.03;
double linG;
double angG;
double linStiffness = 4000;
double angStiffness = 30;
double cameraMovementStepSize = 0.05;
double cameraRotationStepSize = 1.0;
double motionMovementStepSize = 0.01;
double motionRotationStepSize = M_PI/180.0*1.0;

cVector3d inputCursorPosition;
cMatrix3d inputCursorRotation;

//---------------------------------------------------------------------------
// SYSTEM VARIABLES
//---------------------------------------------------------------------------

// User interface
shared_ptr<cCustomFurnaceDevice> hapticDevice;
bool isHapticDeviceAvailable = false;
bool userInterfaceCommunicating = false;
bool userInterfaceThreadFinished = false;
bool isButtonPressed = false;
cFrequencyCounter freqCounterUserInterface;
cThread* userInterfaceThread;
cVector3d renderForce, renderTorque;
cVector3d deviceZeroPosition;
cMatrix3d deviceZeroRotation;


// World 
#if defined(USE_INDY)
cIndyFurnace* workspace;
#else
#if defined(USE_ODE)
cODEFurnace* workspace;
#elif defined(USE_BULLET)
cBulletFurnace* workspace;
#endif
#endif

bool worldRunning = false;
bool worldThreadFinished = true;
cFrequencyCounter freqCounterWorld;
cThread* worldThread;
cVector3d sensorForce, sensorTorque;

bool teleoperationMode = false;
bool commandActivate = false;

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;
bool fullscreen = true;
bool mirroredDisplay = false;

GLFWwindow* window = NULL;
int width = 0;
int height = 0;
int swapInterval = 1;
cFrequencyCounter freqCounterGraphics;
cPanel* panel;
cLabel* labelRates;
cLabel* labelStatus;
cLabel* labelSuccess;
cLabel* labelFailure;
cLabel* labelForce;
cLabel* labelTorque;
//cLevel* levelForce;
//cLevel* levelTorque;
cScope* scopeForce;
cScope* scopeTorque;

#ifndef USE_INDY
    cPanel* panelMessage;
    cLabel* labelMessage;
#endif
//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------


void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);
void errorCallback(int error, const char* a_description);
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);
void updateGraphics(void);
void updateUserInterface(void);
void updateWorld(void);

void close(void);

