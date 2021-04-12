#pragma once

//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"
#include "cFurnace.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// CONTROL VARIABLES
//---------------------------------------------------------------------------

#define INITIAL_CURSOR_POS_X    1.5
#define INITIAL_CURSOR_POS_Y    0.2
#define INITIAL_CURSOR_POS_Z    2.5

double workspaceScaleFactor = 30.0;
double linGain = 0.05;
double angGain = 0.03;
double linG;
double angG;
double linStiffness = 4000;
double angStiffness = 30;
double cameraMovementStepSize = 0.05;
double cameraRotationStepSize = 1.0;

cVector3d keyInputCursorPosition;
cMatrix3d keyInputCursorRotation;

//---------------------------------------------------------------------------
// SYSTEM VARIABLES
//---------------------------------------------------------------------------

cFurnace* simEnvironment;

// variables for haptic rendering (window)
bool simulationRunning = false;
bool simulationFinished = true;
cFrequencyCounter freqCounterHaptics;
cThread* hapticsThread;
bool teleoperationMode = false;

cHapticDeviceHandler* handler;
shared_ptr<cGenericHapticDevice> hapticDevice;

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Actifve stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;
bool fullscreen = false;
bool mirroredDisplay = false;

GLFWwindow* window = NULL;
int width = 0;
int height = 0;
int swapInterval = 1;
cFrequencyCounter freqCounterGraphics;
cLabel* labelRates;
//cPanel* panelRates;


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------


void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);
void errorCallback(int error, const char* a_description);
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);
void updateGraphics(void);
void updateHaptics(void);

void close(void);

