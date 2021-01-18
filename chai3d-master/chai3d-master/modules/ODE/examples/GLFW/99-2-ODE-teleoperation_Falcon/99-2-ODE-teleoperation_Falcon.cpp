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
enum {
	MANIPULATION_MODE = 0,
	MOBILE_MODE = 1, 
	RECOVERY_MODE = 2, 
};

cWorld* world;
cCamera* camera;
cSpotLight* light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> Falcon;
shared_ptr<cGenericHapticDevice> AgileEye;

// workspace scale factor
double workspaceScaleFactor = 10.0;

// Device state variables
cVector3d posMaster, posSlave;
cVector3d posOrigin;
cMatrix3d rotMaster, rotSlave;
cVector3d sensorForce, sensorTorque, renderForce, renderTorque;
int controlMode = RECOVERY_MODE;
int countRecoveryMode = 0;
bool manipulationReady = false;
bool button1Clicked = false;
bool button2Clicked = false;


cMesh* cursor = new cMesh();
cMesh* tool = new cMesh();
cMesh* originWorld = new cMesh();
cMesh* originFalcon = new cMesh();
cMesh* slave = new cMesh();
cMesh* ground = new cMesh();
double toolLength = 0.5;
double toolRadius = 0.01;

//------------------------------------------------------------------------------
// INDY SETTINGS
//------------------------------------------------------------------------------
//double pRef1[6] = { 0.605, 0.076, 0.221, 0.0, 180.000, 0.000, };
//double pdotRef1[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//double pRef2[6] = { 0.455, 0.076, 0.221, 0.0, 180.000, 0.000, };
//double pdotRef2[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float defaultPose[6] = { 0.0f, -30.0f, -105.0f, 0.0f, 45.0f, 0.0f };

IndyDedicatedTCPTestClient indyTCP;
CustomIndyDedicatedTCPTestClient customTCP;
int indy_cmode = 0;
double alpha = 0.0;

//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------
// flag to indicate if the haptic teleoperation currently running
bool teleoperationRunning = false;

// flag to indicate if the haptic teleoperation has terminated
bool falconThreadFinished = false;
bool agileEyeThreadFinished = false;
bool indyThreadFinished = false;

// a frequency counter to measure the teleoperation haptic rate
cFrequencyCounter freqCounterGraphics;
cFrequencyCounter freqCounterFalcon;
cFrequencyCounter freqCounterAgileEye;
cFrequencyCounter freqCounterIndy;

cLabel* labelRates;

// threads
cThread* falconThread;
cThread* agileEyeThread;
cThread* indyThread;

// a handle to window display context
GLFWwindow* window = NULL;
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
void updateFalcon(void);
void updateAgileEye(void);
void updateIndy(void);

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
	cout << "[h] - Move to home position" << endl;
	cout << "[c] - Calibrate device" << endl;
	cout << "[m] - Toggle teleoperation mode" << endl;
	cout << "[r] - Reset ft sensor bias" << endl;
	cout << "[f] - Toggle full screen\n" << endl;
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
	glfwSetWindowPos(window, x, y);
	glfwSetKeyCallback(window, keyCallback);
	glfwSetWindowSizeCallback(window, windowSizeCallback);
	glfwMakeContextCurrent(window);
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
	// HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();
	int numDevice = handler->getNumDevices();
	if (numDevice != 2) {
		std::cout << "[HapticDeviceHandler] Detected device number is not matched. Quit process. (numDevice: " << numDevice << ")" << endl;
		return 0;
	}
	handler->getDevice(Falcon, 0);
	handler->getDevice(AgileEye, 1);

	// start haptic device
	if (!Falcon->open()) {
		std::cout << "Failed to open Falcon. Quit process" << std::endl;
		Falcon->close();
		return 0;
	}
	if (!AgileEye->open()) {
		std::cout << "Failed to open AgileEye. Quit process" << std::endl;
		Falcon->close();
		AgileEye->close();
		return 0;
	}

	// connect Indy DCP / TCP communication
	indyTCP.connect();
	customTCP.connect();

	posOrigin.set(0.626, -0.186, 0.388);

	//-----------------------------------------------------------------------
	// 3D - SCENEGRAPH
	//-----------------------------------------------------------------------
	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setBlack();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);
	camera->setClippingPlanes(0.01, 10.0);
	camera->setStereoMode(stereoMode);
	camera->setStereoEyeSeparation(0.02);
	camera->setStereoFocalLength(2.0);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a light source
	light = new cSpotLight(world);
	world->addChild(light);
	light->setEnabled(true);
	light->setLocalPos(0, 0, 5);
	light->setDir(0, 0, -1.0);

	// set uniform concentration level of light 
	light->setSpotExponent(1.0);
	light->setShadowMapEnabled(true);
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

	// world origin
	cCreateBox(originWorld, 0.01, 0.01, 0.01);
	world->addChild(originWorld);
	originWorld->setShowFrame(true);

	// falcon origin
	cCreateBox(originFalcon, 0.01, 0.01, 0.01);
	world->addChild(originFalcon);
	//originFalcon->setShowFrame(true);
	originFalcon->setLocalPos(posOrigin);

	// ground
	cCreatePlane(ground, 3.0, 3.0);
	world->addChild(ground);
	ground->setShowFrame(true);

	// master cursor
	cCreateBox(cursor, 0.01, 0.01, 0.01);
	world->addChild(cursor);
	cursor->setShowFrame(true);

	cCreateCylinder(tool, toolLength, toolRadius);
	world->addChild(tool);
	//device->setShowFrame(true);

	// slave
	cCreateBox(slave, 0.1, 0.1, 0.1);
	world->addChild(slave);
	slave->setShowFrame(true);

	cMaterial mat_tool, mat_ground, mat_slave;
	mat_tool.setRedCrimson();
	mat_ground.setWhite();
	mat_slave.setBlueAqua();

	tool->setMaterial(mat_tool);
	cursor->setMaterial(mat_tool);
	originWorld->setMaterial(mat_tool);
	originFalcon->setMaterial(mat_tool);
	slave->setMaterial(mat_slave);
	ground->setMaterial(mat_ground);

	//-----------------------------------------------------------------------
	// START Device Communication (Falcon, AgileEye, Indy)
	//-----------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop and teleoperation loop
	teleoperationRunning = true;
	indyThread = new cThread();
	indyThread->start(updateIndy, CTHREAD_PRIORITY_GRAPHICS);
	agileEyeThread = new cThread();
	agileEyeThread->start(updateAgileEye, CTHREAD_PRIORITY_HAPTICS);
	falconThread = new cThread();
	falconThread->start(updateFalcon, CTHREAD_PRIORITY_HAPTICS);


	// setup callback when application exits
	atexit(close);

	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	while (!glfwWindowShouldClose(window))
	{
		glfwGetWindowSize(window, &width, &height);
		updateGraphics();
		glfwSwapBuffers(window);
		glfwPollEvents();
		freqCounterGraphics.signal(1);
	}

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

void updateGraphics(void)
{
	// Frequency widget update
	std::string strControlMode;
	if (controlMode == MANIPULATION_MODE) {
		strControlMode = "Manipulation Mode";
	}
	if (controlMode == MOBILE_MODE) {
		strControlMode = "Mobile Mode";
	}
	if (controlMode == RECOVERY_MODE) {
		strControlMode = "Recovery Mode";
	}

	labelRates->setText(
		"[Graphic freq./Falcon freq./Agile eye freq./Indy freq./Control mode/Indy cmode/manipulationReady] \r\n"
		+ cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / "
		+ cStr(freqCounterFalcon.getFrequency(), 0) + " Hz / "
		+ cStr(freqCounterAgileEye.getFrequency(), 0) + " Hz / "
		+ cStr(freqCounterIndy.getFrequency(), 0) + " Hz / "
		+ strControlMode + " / "
		+ cStr(indy_cmode) + " / "
		+ cStr(manipulationReady)
	);
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), height * 0.05);
	labelRates->setFontScale(1.0 / 500.0 * height);

	// update camera view
	camera->set(cVector3d(-2.0, 0.0, 2.0),    // camera position (eye)
		cVector3d(2.0, 0.0, 0.0),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

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

cVector3d axisAngle(cMatrix3d rot) {
	double a;
	double angle, axis1, axis2, axis3;
	a = sqrt((rot(2, 1) - rot(1, 2)) * (rot(2, 1) - rot(1, 2)) + (rot(0, 2) - rot(2, 0)) * (rot(0, 2) - rot(2, 0)) + (rot(1, 0) - rot(0, 1)) * (rot(1, 0) - rot(0, 1)));
	angle = acos((rot(0,0) + rot(1,1) + rot(2,2) - 1) / 2.0);
	axis1 = (rot(2,1) - rot(1,2)) / a;
	axis2 = (rot(0,2) - rot(2,0)) / a;
	axis3 = (rot(1,0) - rot(0,1)) / a;

	cVector3d res(angle * axis1, angle * axis2, angle * axis3);
	return res;
}

void updateFalcon () {
	// teleoperation clock
	cPrecisionClock falconClock;
	falconClock.start(true);


	int count = 0;
	double falconTimeStep = 0.001;
	countRecoveryMode = 0;
	// main haptic teleoperation loop
	while (teleoperationRunning)
	{
		// retrieve teleoperation time and compute next interval
		double time = falconClock.getCurrentTimeSeconds();
		if (time < falconTimeStep) {
			continue;
		}

		// update frequency counter
		freqCounterFalcon.signal(1);

		falconClock.reset();
		falconClock.start();

		// update position and orientation of tool
		cVector3d posFalcon;
		Falcon->getPosition(posFalcon);
		posMaster = posOrigin + workspaceScaleFactor*cVector3d(-posFalcon(0), -posFalcon(1), posFalcon(2));
		cursor->setLocalPos(posMaster);			// for axis check
		tool->setLocalPos(posMaster);

		// button handling
		bool button1, button2;
		Falcon->getUserSwitch(1, button1);
		Falcon->getUserSwitch(2, button2);
		if (button1) {
			button1Clicked = true;			
		}
		else {
			if (button1Clicked) {
				// button up
				printf("Button1 clicked \n");
				switch (controlMode) {
				case MANIPULATION_MODE:
				case RECOVERY_MODE:
					if (indy_cmode == 20) indyTCP.ToggleTeleoperationMode();					
					manipulationReady = false;
					controlMode = MOBILE_MODE;
					break;
				case MOBILE_MODE:
					countRecoveryMode = 0;
					controlMode = RECOVERY_MODE;
					break;
				}
			}
			button1Clicked = false;
		}

		if (button2) {
			button2Clicked = true;
		}
		else {
			if (button2Clicked) {
				// button up
				// Do something
			}
			button2Clicked = false;
		}

		cVector3d posError, rotErrorAxisAngle;
		cMatrix3d rotMasterTrans, rotError;
		renderForce = cVector3d(0.0, 0.0, 0.0);
		double linearStiffness, rotationalStiffness;
		switch (controlMode) {
		case MANIPULATION_MODE:
			linearStiffness = 100.0;
			rotationalStiffness = 0.5;
			posError = posSlave - posMaster;
			renderForce = linearStiffness * cVector3d(-posError(0), -posError(1), posError(2));
			renderForce += 1.0 * cVector3d(-sensorForce(0), -sensorForce(1), sensorForce(2));

			if (renderForce.length() > 4.0) {
				printf("Falcon saturated (force norm: %.5f)\n", renderForce.length());
				renderForce = renderForce / renderForce.length() * 4.0;				
			}

			rotMaster.transr(rotMasterTrans);
			rotError = cMatrix3d(-1, 0, 0, 0, -1, 0, 0, 0, 1) * rotMasterTrans * rotSlave * cMatrix3d(-1, 0, 0, 0, -1, 0, 0, 0, 1);
			rotErrorAxisAngle = axisAngle(rotError);
			renderTorque = rotationalStiffness * rotErrorAxisAngle;
			renderTorque += 0.1 * cVector3d(-sensorTorque(0), -sensorTorque(1), sensorTorque(2));

			if (renderTorque.length() > 0.19) {
				printf("Agile Eye saturated (torque norm: %.5f)\n", renderTorque.length());
				renderTorque = renderTorque / renderTorque.length() * 0.19;				
			}
			
			printf("%.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n", sensorForce(0), sensorForce(1), sensorForce(2), sensorTorque(0), sensorTorque(1), sensorTorque(2));
			break;

		case MOBILE_MODE:	
			linearStiffness = 1000.0;
			rotationalStiffness = 0.0;
			posError = cVector3d(0.0, 0.0, 0.0) - posFalcon;
			renderForce = linearStiffness * posError;

			if (renderForce.length() > 2.0) {
				printf("Falcon saturated (force norm: %.5f)\n", renderForce.length());
				renderForce = renderForce / renderForce.length() * 2.0;
			}

			renderTorque.set(0, 0, 0);
			
			/*// torque test
			rotMaster.transr(rotMasterTrans);
			rotError = cMatrix3d(-1, 0, 0, 0, -1, 0, 0, 0, 1) * rotMasterTrans * cMatrix3d(-1, 0, 0, 0, -1, 0, 0, 0, 1);
			rotErrorAxisAngle = axisAngle(rotError);
			renderTorque = 0.5 * rotErrorAxisAngle;

			if (renderTorque.length() > 0.19) {
				renderTorque = renderTorque / renderTorque.length() * 0.19;
				printf("Agile Eye saturated (torque norm: %.5f,\terror norm: %.5f)\n", renderTorque.length(), rotErrorAxisAngle.length());
			}*/

			if (posError.length() > 0.01) {
				posOrigin += falconTimeStep * 30.0 * (posError.length() - 0.01) * cVector3d(posError(0), posError(1), -posError(2)) / posError.length();
				originFalcon->setLocalPos(posOrigin);
			}
			break;

		case RECOVERY_MODE:
			if (countRecoveryMode > 3000) {
				linearStiffness = 1500.0;
				rotationalStiffness = 0.5;
			}
			else {
				linearStiffness = 1500.0 / 3000.0 * countRecoveryMode;
				rotationalStiffness = 0.5 / 3000.0 * countRecoveryMode;
				//printf("%d \n", countRecoveryMode);
			}
			posError = posSlave- posMaster;
			renderForce = linearStiffness * cVector3d(-posError(0), -posError(1), posError(2));

			if (renderForce.length() > 2.0) {
				printf("Falcon saturated (force norm: %.5f)\n", renderForce.length());
				renderForce = renderForce / renderForce.length() * 2.0;
			}

			rotMaster.transr(rotMasterTrans);
			rotError = cMatrix3d(-1, 0, 0, 0, -1, 0, 0, 0, 1) * rotMasterTrans * rotSlave * cMatrix3d(-1, 0, 0, 0, -1, 0, 0, 0, 1);
			rotErrorAxisAngle = axisAngle(rotError);
			renderTorque = rotationalStiffness * rotErrorAxisAngle;

			if (renderTorque.length() > 0.19) {
				printf("Agile Eye saturated (torque norm: %.5f)\n", renderTorque.length());
				renderTorque = renderTorque / renderTorque.length() * 0.19;
			}

			if (posError.length() < 0.005 && rotErrorAxisAngle.length() < 5.0*M_PI/360.0) manipulationReady = true;
			else manipulationReady = false;

			countRecoveryMode++;
			break;
		}
		Falcon->setForce(renderForce + cVector3d(0.0, 0.0, 4.0));

		count++;
	}

	// exit haptics thread
	falconThreadFinished = true;
}

void updateAgileEye() {
	// teleoperation clock
	cPrecisionClock agileEyeClock;
	agileEyeClock.start(true);


	int count = 0;
	// main haptic teleoperation loop
	while (teleoperationRunning)
	{
		// retrieve teleoperation time and compute next interval
		double time = agileEyeClock.getCurrentTimeSeconds();
		if (time < 0.002) {
			continue;
		}

		// update frequency counter
		freqCounterAgileEye.signal(1);

		agileEyeClock.reset();
		agileEyeClock.start();

		// update position and orientation of tool
		cMatrix3d rotAgileEye;
		AgileEye->getRotation(rotAgileEye);
		rotMaster = cMatrix3d(-1, 0, 0, 0, -1, 0, 0, 0, 1) * rotAgileEye * cMatrix3d(-1, 0, 0, 0, -1, 0, 0, 0, 1);
		//rotMaster = rotAgileEye;

		// attach cursor (show device position)
		cursor->setLocalRot(rotMaster);
		tool->setLocalRot(rotMaster * cMatrix3d(0, 0, -1, 0, 1, 0, 1, 0, 0));

		AgileEye->setForceAndTorque(cVector3d(0.0, 0.0, 0.0), renderTorque);

		count++;
	}

	// exit haptics thread
	agileEyeThreadFinished = true;
}

void updateIndy() {
	// teleoperation clock
	cPrecisionClock indyClock;
	indyClock.start(true);

	int count = 0;
	// main haptic teleoperation loop
	while (teleoperationRunning)
	{
		// retrieve teleoperation time and compute next interval
		double time = indyClock.getCurrentTimeSeconds();
		if (time < 0.002) {
			continue;
		}

		// update frequency counter
		freqCounterIndy.signal(1);

		indyClock.reset();
		indyClock.start();

		// update position and orientation of Indy
		cVector3d posIndy;
		cMatrix3d rotIndy;
		double masterPos[6], indyPos[6], indyForceTorque[6];
		double passivityPort = 0.0;
		int cmode;
		cVector3d masterRot; 
		if (indy_cmode == 20) {
			masterRot = axisAngle(rotMaster * cMatrix3d(0, 0, 1, 0, 1, 0, -1, 0, 0));
			masterPos[0] = posMaster(0);
			masterPos[1] = posMaster(1);
			masterPos[2] = posMaster(2);
			masterPos[3] = masterRot(0);
			masterPos[4] = masterRot(1);
			masterPos[5] = masterRot(2);
		}
		else {
			masterRot = axisAngle(rotSlave * cMatrix3d(0, 0, 1, 0, 1, 0, -1, 0, 0));
			masterPos[0] = posSlave(0);
			masterPos[1] = posSlave(1);
			masterPos[2] = posSlave(2);
			masterPos[3] = masterRot(0);
			masterPos[4] = masterRot(1);
			masterPos[5] = masterRot(2);
		}


		customTCP.SendIndyCommandAndReadState(masterPos, passivityPort, indyPos, indyForceTorque, cmode);
		//if (controlMode == MANIPULATION_MODE) {
		//	for (int i = 0; i < 6; i++) {
		//		printf("%.5f, ", masterPos[i]);
		//	}
		//	printf("\n");
		//	for (int i = 0; i < 6; i++) {
		//		printf("%.5f, ", indyPos[i]);
		//	}
		//	printf("\n");
		//}

		double axis1, axis2, axis3, angle;
		posIndy(0) = indyPos[0];
		posIndy(1) = indyPos[1];
		posIndy(2) = indyPos[2];
		angle = sqrt(indyPos[3] * indyPos[3] + indyPos[4] * indyPos[4] + indyPos[5] * indyPos[5]);
		axis1 = indyPos[3];
		axis2 = indyPos[4];
		axis3 = indyPos[5];
		if (angle > 0) {
			rotIndy.setAxisAngleRotationRad(cVector3d(axis1, axis2, axis3), angle);
		}
		else {
			rotIndy.set(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
		}

		sensorForce.set(indyForceTorque[0], indyForceTorque[1], indyForceTorque[2]);
		sensorTorque.set(indyForceTorque[3], indyForceTorque[4], indyForceTorque[5]);

		indy_cmode = cmode;

		posSlave = posIndy;
		rotSlave = rotIndy * cMatrix3d(0, 0, -1, 0, 1, 0, 1, 0, 0);

		// attach cursor (show device position)
		slave->setLocalPos(posSlave);
		slave->setLocalRot(rotSlave);

		AgileEye->setForceAndTorque(cVector3d(0.0, 0.0, 0.0), renderTorque);

		count++;
	}

	// exit haptics thread
	indyThreadFinished = true;
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

	GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(monitor);

	switch (a_key) {

	// option - quit
	case GLFW_KEY_ESCAPE:
	case GLFW_KEY_Q:
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
		break;

	// option - toggle fullscreen
	case GLFW_KEY_F:		
		fullscreen = !fullscreen;
		if (fullscreen){
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		break;

	// option - calibration agile eye
	case GLFW_KEY_C:
		AgileEye->calibrate();
		break;

	// option - go home
	case GLFW_KEY_H:
		indyTCP.GoHome();
		break;

	// option - start Indy teleoperation mode
	case GLFW_KEY_M:
		if (manipulationReady && indy_cmode != 20) {
			indyTCP.ToggleTeleoperationMode();
			controlMode = MANIPULATION_MODE;
		}

		break;

	// option - reset Indy and FT sensor
	case GLFW_KEY_R:
		indyTCP.ResetRobot();
		indyTCP.ResetFTBias();
		Sleep(3000);
		indyTCP.MoveToJ(defaultPose);
		break;
	}
}

//---------------------------------------------------------------------------

void close(void)
{
	if (indy_cmode == 20) {
		indyTCP.ToggleTeleoperationMode();
		indyTCP.MoveToJ(defaultPose);
	}

	// stop the teleoperation
	teleoperationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!falconThreadFinished) {
		printf("Waiting for Falcon Thread\n");
		cSleepMs(100); 
	}
	while (!agileEyeThreadFinished) {
		printf("Waiting for AgileEye Thread\n");
		cSleepMs(100); 
	}
	while (!indyThreadFinished) { 
		printf("Waiting for Indy Thread\n");
		cSleepMs(100); 
	}

	// close haptic device
	Falcon->close();
	AgileEye->close();

	// disconnect Indy DCP, TCP communication
	indyTCP.disconnect();
	customTCP.disconnect();

	// delete resources
	delete falconThread;
	delete agileEyeThread;
	// delete indyThread;
	delete world;
	delete handler;
}

//---------------------------------------------------------------------------