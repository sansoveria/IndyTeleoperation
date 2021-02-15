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
	IDLE_MODE = 0,
	VELOCITY_MODE = 1,
	POSITION_MODE = 2,
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
cVector3d posOrigin, posFalconRef;
cMatrix3d rotMaster, rotSlave;
cVector3d sensorForce, sensorTorque, renderForce, renderTorque;
double masterPosePrev[6] = { 0.0 };
int currentControlMode = IDLE_MODE;
bool controlModeSwitch = true;
//bool isButton1Clicked = false;
//bool isButton2Clicked = false;
bool isButtonClicked = false;

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
#define USE_INDY
float defaultPose[6] = { 0.0f, -30.0f, -105.0f, 0.0f, 45.0f, -90.0f };

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
bool indyThreadFinished = false;

// a frequency counter to measure the teleoperation haptic rate
cFrequencyCounter freqCounterGraphics;
cFrequencyCounter freqCounterFalcon;
cFrequencyCounter freqCounterIndy;

cLabel* labelRates;

// threads
cThread* falconThread;
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
void updateIndy(void);

void close(void);

cMatrix3d rotateY(double theta) {
	cMatrix3d offset(cos(theta / 180.0 * M_PI), 0.0, sin(theta / 180.0 * M_PI), 0.0, 1.0, 0.0, -sin(theta / 180.0 * M_PI), 0.0, cos(theta / 180.0 * M_PI));
	return offset;
}

cVector3d axisAngle(cMatrix3d rot) {
	double a;
	double angle, axis1, axis2, axis3;
	a = sqrt((rot(2, 1) - rot(1, 2)) * (rot(2, 1) - rot(1, 2)) + (rot(0, 2) - rot(2, 0)) * (rot(0, 2) - rot(2, 0)) + (rot(1, 0) - rot(0, 1)) * (rot(1, 0) - rot(0, 1)));
	//assert(a != 0);
	if (a == 0) {
		printf("[axisAngle] a = 0\n");
		return cVector3d(0, 0, 0);
	}

	angle = acos((rot(0, 0) + rot(1, 1) + rot(2, 2) - 1) / 2.0);
	axis1 = (rot(2, 1) - rot(1, 2)) / a;
	axis2 = (rot(0, 2) - rot(2, 0)) / a;
	axis3 = (rot(1, 0) - rot(0, 1)) / a;

	cVector3d res(angle * axis1, angle * axis2, angle * axis3);
	return res;
}

class cFilteredDerivative {
public:
	cFilteredDerivative() {
		setParam(0.002, 50.0);
		_xPrev = 0.0;
		_yPrev = 0.0;
	}

	~cFilteredDerivative() {}

	void setParam(double dT, double cutOffFreq) {
		_dT = dT;
		_cutOffFreq = cutOffFreq;
	}

	double filter(double x) {
		double w = 2.0 * M_PI * _cutOffFreq;
		w = (2.0 / _dT) * ::tan(w * _dT / 2.0);   // warping to match cut-off frequency

		double y = (2.0 - w * _dT) / (2.0 + w * _dT) * _yPrev + 2.0 * w / (2.0 + w * _dT) * x - 2.0 * w / (2.0 + w * _dT) * _xPrev;

		_xPrev = x;
		_yPrev = y;

		return y;
	}

	double filter(double x, double dT) {
		double w = 2.0 * M_PI * _cutOffFreq;
		w = (2.0 / dT) * ::tan(w * dT / 2.0);   // warping to match cut-off frequency

		double y = (2.0 - w * dT) / (2.0 + w * dT) * _yPrev + 2.0 * w / (2.0 + w * dT) * x - 2.0 * w / (2.0 + w * dT) * _xPrev;

		_xPrev = x;
		_yPrev = y;

		return y;
	}

	void reset(double x) {
		_yPrev = 0.0;
		_xPrev = x;
	}

private:
	double _dT, _cutOffFreq;
	double _yPrev;
	double _xPrev;
};


class cIndyTransform {
public:
	cIndyTransform()
	{
		_localTransformation.identity();
		for (int i = 0; i < 6; i++) {
			_poseCommandInIndyCoordinate[i] = 0.0;
			_velocityCommandInIndyCoordinate[i] = 0.0;
			_derivative[i].setParam(0.002, 50.0);
		}
	}

	~cIndyTransform() {}

	void setParam(double dT, double cutOffFreq, cMatrix3d localTransform) {
		for (int i = 0; i < 6; i++) {
			_derivative[i].setParam(dT, cutOffFreq);
		}
		_localTransformation = localTransform;
	}

	void updateCommand(cVector3d posInMasterSpace, cMatrix3d rotInMasterSpace, double dT, bool INIT_FILTER) {
		cMatrix3d rotMatInIndySpace = rotInMasterSpace * cMatrix3d(0, 0, 1, 0, 1, 0, -1, 0, 0);
		cVector3d rotVecInIndySpace = axisAngle(rotMatInIndySpace);

		_poseCommandInIndyCoordinate[0] = posInMasterSpace(0);
		_poseCommandInIndyCoordinate[1] = posInMasterSpace(1);
		_poseCommandInIndyCoordinate[2] = posInMasterSpace(2);
		_poseCommandInIndyCoordinate[3] = rotVecInIndySpace(0);
		_poseCommandInIndyCoordinate[4] = rotVecInIndySpace(1);
		_poseCommandInIndyCoordinate[5] = rotVecInIndySpace(2);

		if (INIT_FILTER) {
			for (int i = 0; i < 6; i++) {
				_derivative[i].reset(_poseCommandInIndyCoordinate[i]);
			}
		}
		cVector3d xidot, rotationalVelocity;
		cMatrix3d dexpXidot, dexpXidotTranspose;
		_velocityCommandInIndyCoordinate[0] = _derivative[0].filter(_poseCommandInIndyCoordinate[0], dT);
		_velocityCommandInIndyCoordinate[1] = _derivative[1].filter(_poseCommandInIndyCoordinate[1], dT);
		_velocityCommandInIndyCoordinate[2] = _derivative[2].filter(_poseCommandInIndyCoordinate[2], dT);
		xidot(0) = _derivative[3].filter(_poseCommandInIndyCoordinate[3], dT);
		xidot(1) = _derivative[4].filter(_poseCommandInIndyCoordinate[4], dT);
		xidot(2) = _derivative[5].filter(_poseCommandInIndyCoordinate[5], dT);
		//printf("%.5f\t%.5f\t%.5f\n", _poseCommandInIndyCoordinate[3], _poseCommandInIndyCoordinate[4], _poseCommandInIndyCoordinate[5]);
		//printf("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n", _velocityCommandInIndyCoordinate[0], _velocityCommandInIndyCoordinate[1], _velocityCommandInIndyCoordinate[2], xidot(0), xidot(1), xidot(2));
		dexp(xidot, dexpXidot);
		dexpXidot.transr(dexpXidotTranspose);
		rotationalVelocity = dexpXidotTranspose * xidot;
		_velocityCommandInIndyCoordinate[3] = rotationalVelocity(0);
		_velocityCommandInIndyCoordinate[4] = rotationalVelocity(1);
		_velocityCommandInIndyCoordinate[5] = rotationalVelocity(2);
	}

	void getPoseCommand(double* command) {
		for (int i = 0; i < 6; i++) {
			command[i] = _poseCommandInIndyCoordinate[i];
		}
	}

	void getVelocityCommand(double* command) {
		for (int i = 0; i < 6; i++) {
			command[i] = _velocityCommandInIndyCoordinate[i];
		}
	}

private:
	void ceiling(cVector3d input, cMatrix3d& res) {
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

	void dexp(cVector3d input, cMatrix3d& res) {
		cMatrix3d ceiling_, identity_;
		cMatrix3d comp1, comp2;
		double alpha_, beta_, inputNorm_;
		ceiling(input, ceiling_);
		identity_.identity();

		inputNorm_ = input.length();
		if (inputNorm_ == 0.0) {
			res.identity();
		}
		else {
			alpha_ = sin(inputNorm_ / 2.0) * cos(inputNorm_ / 2.0) / inputNorm_ * 2.0;
			beta_ = sin(inputNorm_ / 2.0) * sin(inputNorm_ / 2.0) / inputNorm_ * 2.0 / inputNorm_ * 2.0;
		
			comp1 = ceiling_;
			comp2 = ceiling_;
			comp1 *= 0.5 * beta_;
			comp2 *= 1.0 / inputNorm_ * (1.0 - alpha_);
			res.identity();
			res.add(comp1);
			res.add(comp2 * ceiling_);
		}		
	}

	cMatrix3d _localTransformation;
	cFilteredDerivative _derivative[6];

	double _poseCommandInIndyCoordinate[6];
	double _velocityCommandInIndyCoordinate[6];
};