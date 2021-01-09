#include <winsock2.h>
#include <windows.h>
//------------------------------------------------------------------------------
#include "NRMK_IndyDCPClient.h"
#include "CustomIndyDCPClient.h"
#include "Demo_setup.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#include "CODE.h"
//------------------------------------------------------------------------------

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

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight* light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// workspace scale factor
double workspaceScaleFactor = 30.0;

// Device state variables
cVector3d posDevice;
cVector3d posDevicePrev;
cMatrix3d rotDevice;
cVector3d linVelDevice;
cVector3d linVelDevicePrev;
cVector3d robot_force, robot_torque;

cMesh* handle = new cMesh();

double cube_size = 0.4;

//------------------------------------------------------------------------------
// INDY SETTINGS
//------------------------------------------------------------------------------
double pRef1[6] = { 0.605, 0.076, 0.221, 0.0, 180.000, 0.000, };
double pdotRef1[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double pRef2[6] = { 0.455, 0.076, 0.221, 0.0, 180.000, 0.000, };
double pdotRef2[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
IndyDedicatedTCPTestClient indyTCP;
CustomIndyDedicatedTCPTestClient customTCP;
int indy_cmode = 0;
double alpha = 0.0;
bool BUTTON_PUSHED = false;
bool MOVE_ROBOT = false;

//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------
// flag to indicate if the haptic teleoperation currently running
bool teleoperationRunning = false;

// flag to indicate if the haptic teleoperation has terminated
bool teleoperationFinished = true;

// a frequency counter to measure the teleoperation haptic rate
cFrequencyCounter freqCounterGraphics;
cFrequencyCounter freqCounterHaptics;
cFrequencyCounter freqCounterIndy;

cLabel* labelRates;

// threads
cThread* hapticsThread;
cThread* indyThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);
void updateHaptics(void);
void updateIndy(void);

// this function closes the application
void close(void);


//===========================================================================
/*
DEMO:    99-ODE-teleoperation.cpp

This example illustrates the use of the ODE framework for simulating
haptic interaction with dynamic bodies. In this scene we create 4
cubic meshes that we individually attach to ODE bodies. One of the blocks
is attached to the haptic device through a virtual spring.
*/
//===========================================================================

int main(int argc, char* argv[])
{
	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------

	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "CHAI3D" << endl;
	cout << "Demo: 04-ODE-tool" << endl;
	cout << "Copyright 2003-2016" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	cout << "[h] - Display help menu" << endl;
	cout << "[m] - toggle teleoperation mode" << endl;
	cout << "[n] - move robot" << endl;
	cout << "[a] - test1" << endl;
	cout << "[s] - test2" << endl;
	cout << "[w] - starting pose" << endl;
	cout << "[z] - raise external force (sim)" << endl;
	cout << "[x] - reduce external force (sim)" << endl;
	cout << "[r] - reset ft sensor bias" << endl;
	cout << "[k] - raise alpha" << endl;
	cout << "[l] - reduce alpha" << endl;
	cout << "[f] - toggle full screen\n" << endl;
	cout << "[q] - Exit application\n" << endl;
	cout << endl << endl;


	//-----------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//-----------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "IndyTeleoperation", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

	// initialize GLEW library
#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif

	//-----------------------------------------------------------------------
	// 3D - SCENEGRAPH
	//-----------------------------------------------------------------------
	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setWhite();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// set the near and far clipping planes of the camera
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.02);
	camera->setStereoFocalLength(2.0);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a light source
	light = new cSpotLight(world);

	// attach light to camera
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// position the light source
	light->setLocalPos(1.0, -1.0, 3.0);

	// define the direction of the light beam
	light->setDir(0, 0, -1.0);

	// set uniform concentration level of light 
	light->setSpotExponent(10.0);

	// enable this light source to generate shadows
	light->setShadowMapEnabled(true);

	// set the resolution of the shadow map
	//light->m_shadowMap->setQualityLow();
	light->m_shadowMap->setQualityHigh();

	// set light cone half angle
	light->setCutOffAngleDeg(45);

	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	cFontPtr font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	labelRates->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(labelRates);

	//-----------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();


	//////////////////////////////////////////////////////////////////////////
	// ODE WORLD
	//////////////////////////////////////////////////////////////////////////

	// stiffness properties
	double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

	//////////////////////////////////////////////////////////////////////////
	// HANDLE
	//////////////////////////////////////////////////////////////////////////
	cCreateSphere(handle, cube_size / 4.0);
	world->addChild(handle);

	cMaterial mat_handle;
	mat_handle.setYellowLight();
	handle->setMaterial(mat_handle);

	//-----------------------------------------------------------------------
	// START Indy Communication
	//-----------------------------------------------------------------------
	indyTCP.connect();
	customTCP.connect();

	// teleoperation in now running
	teleoperationRunning = true;

	// create a thread which starts the main haptics rendering loop
	indyThread = new cThread();
	hapticsThread = new cThread();
	indyThread->start(updateIndy, CTHREAD_PRIORITY_GRAPHICS);
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// setup callback when application exits
	atexit(close);

	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);

		// render graphics
		updateGraphics();

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		freqCounterGraphics.signal(1);
	}

	// close window
	glfwDestroyWindow(window);

	// terminate GLFW library
	glfwTerminate();

	indyTCP.disconnect();
	customTCP.disconnect();

	// exit
	return 0;
}

void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////
	// Frequency
	labelRates->setText("Graphic freq./Haptic freq./Indy freq. \r\n" + cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " 
		+ cStr(freqCounterHaptics.getFrequency(), 0) + " Hz / " 
		+ cStr(freqCounterIndy.getFrequency(), 0) + " Hz");
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), height * 0.05);
	labelRates->setFontScale(1.0 / 500.0 * height);

	/////////////////////////////////////////////////////////////////////
	// UPDATE CAMERA VIEW
	/////////////////////////////////////////////////////////////////////

	// position and orient the camera
	camera->set(cVector3d(5.0, -1.0, 2.0),    // camera position (eye)
		cVector3d(0.0, -1.0, -0.5),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

	
	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// wait until all GL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//---------------------------------------------------------------------------

cMatrix3d rotateY(double theta) {
	cMatrix3d offset(cos(theta / 180.0 * M_PI), 0.0, sin(theta / 180.0 * M_PI), 0.0, 1.0, 0.0, -sin(theta / 180.0 * M_PI), 0.0, cos(theta / 180.0 * M_PI));
	return offset;
}

#define TIME_STEP					0.001	// s
#define LOW_PASS_FILTER_FREQ		100		// Hz
void updateHaptics() {
	// start haptic device
	hapticDevice->open();

	// teleoperation clock
	cPrecisionClock hapticClock;
	hapticClock.start(true);

	int count = 0;
	// main haptic teleoperation loop
	while (teleoperationRunning)
	{
		// retrieve teleoperation time and compute next interval
		double time = hapticClock.getCurrentTimeSeconds();
		if (time < 0.001) {
			continue;
		}

		// update frequency counter
		freqCounterHaptics.signal(1);

		//cout << "[haptic time]" << time << endl;
		hapticClock.reset();
		hapticClock.start();

		// update position and orientation of tool
		hapticDevice->getPosition(posDevice);
		hapticDevice->getRotation(rotDevice);

		// attach cursor (show device position)
		handle->setLocalPos(posDevice);
		handle->setLocalRot(rotDevice);

		// calculate linear velocity
		double tc = 1.0 / (2.0 * M_PI * LOW_PASS_FILTER_FREQ);
		linVelDevice = (2.0 * tc - TIME_STEP) / (2.0 * tc + TIME_STEP) * linVelDevicePrev + 2.0 / (2.0 * tc + TIME_STEP) * (posDevice - posDevicePrev);
		posDevice.copyto(posDevicePrev);
		linVelDevice.copyto(linVelDevicePrev);

		// button handling
		bool button;
		hapticDevice->getUserSwitch(0, button);
		if (button) {
			BUTTON_PUSHED = true;
		}
		else {
			alpha = 0.0;
			BUTTON_PUSHED = false;
			//cout << "[Button] " << button << endl;
		}

		// render feedback force
		cVector3d damping_force = -3*linVelDevice;
		cVector3d render_force = 0.02*robot_force + damping_force;
		hapticDevice->setForce(render_force);
		count++;
	}
	// exit haptics thread
	teleoperationFinished = true;
}

//---------------------------------------------------------------------------

void updateIndy() {
	// teleoperation clock
	cPrecisionClock indyClock;
	indyClock.start(true);

	int count = 0;
	// main haptic teleoperation loop
	while (teleoperationRunning)
	{
		// update frequency counter
		freqCounterIndy.signal(1);

		// retrieve teleoperation time and compute next interval
		double time = indyClock.getCurrentTimeSeconds();
		//if (time < 0.001) {
		//	continue;
		//}
		//cout << "[indy time]" << time << endl;
		//indyClock.reset();
		//indyClock.start();


		// update position and orientation of tool
		cVector3d pos;
		cMatrix3d rot;
		posDevice.copyto(pos);
		rotDevice.copyto(rot);
			
		cMatrix3d tool_offset(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0);
		//cMatrix3d tool_offset(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
		//cMatrix3d tool_offset(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
		rot = cMul(rot, tool_offset);

		// scale position of device
		pos.mul(2.0);

		// attach cursor (show device position)
		handle->setLocalPos(pos);
		handle->setLocalRot(rot);

		if (indy_cmode == 20) {
			cMatrix3d indy_offset1(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0);
			cMatrix3d indy_offset2(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);
			cMatrix3d indy_offset3(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);
			rot = cMul(rotateY(-90), rot);
			rot = cMul(rot, rotateY(-90.0));
			rot.trans();

			Eigen::Matrix3d rotDeviceEigen;
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					rotDeviceEigen(i, j) = rot(i, j);
				}
			}
			/*
			rotDeviceEigen(0, 0) = rotDevice(0, 0);
			rotDeviceEigen(0, 1) = rotDevice(0, 1);
			rotDeviceEigen(0, 2) = rotDevice(0, 2);
			rotDeviceEigen(1, 0) = rotDevice(1, 0);
			rotDeviceEigen(1, 1) = rotDevice(1, 1);
			rotDeviceEigen(1, 2) = rotDevice(1, 2);
			rotDeviceEigen(2, 0) = rotDevice(2, 0);
			rotDeviceEigen(2, 1) = rotDevice(2, 1);
			rotDeviceEigen(2, 2) = rotDevice(2, 2);
			*/
			cVector3d euler = rotDeviceEigen.eulerAngles(2, 1, 0);

			double p[6];
			double v[6] = { 0.0 };
			p[0] = -pos.x() + 0.80;
			p[1] = -pos.y();
			p[2] = pos.z() + 0.20;
			p[3] = euler(0) * 180.0 / M_PI;
			p[4] = euler(1) * 180.0 / M_PI;
			p[5] = euler(2) * 180.0 / M_PI;
			//p[0] = 0.454;
			//p[1] = -0.186;
			//p[2] = 0.416;
			//p[3] = 178;
			//p[4] = 109;
			//p[5] = 178;
			double* fext;
			if (MOVE_ROBOT) {
				//fext = customTCP.SetRefState(p, v, alpha);
				fext = customTCP.SetRefState(p, v);
			}
			else {
				fext = customTCP.GetExtWrench();
			}
			// convert force
			robot_force(0) = -fext[0];
			robot_force(1) = -fext[1];
			robot_force(2) = fext[2];
			robot_torque = cVector3d(0, 0, 0);


		}

		if ((int)(time*1000.0) % 1000 == 0) {
			indy_cmode = indyTCP.GetCMode();
			cout << "INDY CMODE: " << indy_cmode << endl;
		}

		count++;

	}
	// exit haptics thread
	teleoperationFinished = true;
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// option - toggle fullscreen
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

	// go home
	else if (a_key == GLFW_KEY_H)
	{
		float joint[6] = { 0.0f, -30.0f, -105.0f, 0.0f, 45.0f, 0.0f };
		indyTCP.MoveToJ(joint);
	}

	else if (a_key == GLFW_KEY_M)
	{
		indyTCP.ToggleTeleoperationMode();
		indy_cmode = indyTCP.GetCMode();
		cout << "indy_cmode: " << indy_cmode << endl;
	}

	// test motion 1
	else if (a_key == GLFW_KEY_A)
	{
		double* fext;
		fext = customTCP.SetRefState(pRef2, pdotRef2, alpha);
		printf("fext: ");
		for (int i = 0; i < 6; i++) {
			printf("%.5f, ", fext[i]);
		}
		printf("\n");
	}

	// test motion 2
	else if (a_key == GLFW_KEY_S)
	{
		double* fext;
		fext = customTCP.SetRefState(pRef1, pdotRef1, alpha);
		printf("fext: ");
		for (int i = 0; i < 6; i++) {
			printf("%.5f, ", fext[i]);
		}
		printf("\n");
	}

	// starting pose
	else if (a_key == GLFW_KEY_W)
	{
		float j_home[6] = { 0.0, -15.0, -90.0, 0.0, 15.0, 0.0 };
		indyTCP.MoveToJ(j_home);
		while (!indyTCP.isMoveFinished()) {
			int a = 0;
		}
	}

	else if (a_key == GLFW_KEY_Z)
	{
		customTCP.RaiseExtWrench();
	}

	else if (a_key == GLFW_KEY_X)
	{
		customTCP.ReduceExtWrench();
	}

	else if (a_key == GLFW_KEY_N)
	{
		if (MOVE_ROBOT) {
			MOVE_ROBOT = false;
			cout << "Robot does not move" << endl;
		}
		else {
			MOVE_ROBOT = true;
			cout << "Robot moves" << endl;
		}
	}

	else if (a_key == GLFW_KEY_R)
	{
		indyTCP.ResetFTBias();
	}

	else if (a_key == GLFW_KEY_K)
	{
		if (BUTTON_PUSHED) {
			if (alpha + 0.01 < 1.0)	alpha = alpha + 0.01;
		}
	}

	else if (a_key == GLFW_KEY_L)
	{
		if (BUTTON_PUSHED) {
			if (alpha - 0.01 > 0.0)	alpha = alpha - 0.01;
		}
	}
}

//---------------------------------------------------------------------------

void close(void)
{
	// stop the teleoperation
	teleoperationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!teleoperationFinished) { cSleepMs(100); }

	// close haptic device
	hapticDevice->close();

	// delete resources
	delete hapticsThread;
	delete indyThread;
	delete world;
	delete handler;
}

//---------------------------------------------------------------------------