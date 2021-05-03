#pragma once
#pragma once

//---------------------------------------------------------------------------
#ifndef CINDYFURNACE
#define CINDYFURNACE

#ifdef USE_INDY
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CGenericFurnace.h"
#include "utils.h"
//---------------------------------------------------------------------------
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <algorithm>            // std::min, std::max
//---------------------------------------------------------------------------
#include "CustomIndyDCPClient.h"
#include "NRMK_IndyDCPClient.h"
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------

#define END_EFFECTOR_TOOL_LENGTH        1.0

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

	void setParam(double dT, double cutOffFreq, cMatrix3d globalTransform, cMatrix3d localTransform) {
		for (int i = 0; i < 6; i++) {
			_derivative[i].setParam(dT, cutOffFreq);
		}
		_localTransformation = localTransform;
		_globalTransformation = globalTransform;
	}

	cMatrix3d getLocalTransformation() {
		return _localTransformation;
	}

	cMatrix3d getGlobalTransformation() {
		return _globalTransformation;
	}

	void initFilters() {
		for (int i = 0; i < 6; i++) {
			_derivative[i].reset(_poseCommandInIndyCoordinate[i]);
		}
	}

	void updateCommand(cVector3d posInMasterSpace, cMatrix3d rotInMasterSpace, double dT, bool INIT_FILTER) {
		cMatrix3d rotMatInIndySpace = cMul(_globalTransformation, cMul(rotInMasterSpace ,_localTransformation));
		cVector3d rotVecInIndySpace = axisAngle(rotMatInIndySpace);
		cVector3d posInIndySpace = cMul(_globalTransformation, posInMasterSpace);
		_poseCommandInIndyCoordinate[0] = posInIndySpace(0);
		_poseCommandInIndyCoordinate[1] = posInIndySpace(1);
		_poseCommandInIndyCoordinate[2] = posInIndySpace(2);
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

	cVector3d axisAngle(cMatrix3d rot) {
		double a;
		double angle, axis1, axis2, axis3;
		a = sqrt((rot(2, 1) - rot(1, 2)) * (rot(2, 1) - rot(1, 2)) + (rot(0, 2) - rot(2, 0)) * (rot(0, 2) - rot(2, 0)) + (rot(1, 0) - rot(0, 1)) * (rot(1, 0) - rot(0, 1)));
		//assert(a != 0);
		if (a == 0) {
			//printf("[axisAngle] a = 0\n");
			return cVector3d(0, 0, 0);
		}

		angle = acos((rot(0, 0) + rot(1, 1) + rot(2, 2) - 1) / 2.0);
		axis1 = (rot(2, 1) - rot(1, 2)) / a;
		axis2 = (rot(0, 2) - rot(2, 0)) / a;
		axis3 = (rot(1, 0) - rot(0, 1)) / a;

		cVector3d res(angle * axis1, angle * axis2, angle * axis3);
		return res;
	}

	cMatrix3d _localTransformation, _globalTransformation;
	cFilteredDerivative _derivative[6];

	double _poseCommandInIndyCoordinate[6];
	double _velocityCommandInIndyCoordinate[6];
};

class cIndyFurnace : cGenericFurnace {
public:
    cIndyFurnace();
    virtual ~cIndyFurnace();

    bool updateShadowMaps(const bool mirror_x, const bool mirror_y);
    void renderView(const int window_width,
        const int window_height,
        const cEyeMode eyeMode = C_STEREO_LEFT_EYE,
        const bool defaultBuffer = true);
    void computeGlobalPositions(const bool frameOnly = true,
        const cVector3d& globalPos = cVector3d(0.0, 0.0, 0.0),
        const cMatrix3d& globalRot = cIdentity3d());
    void updateDynamics(double interval);
    void addVisualComponent(cGenericObject* object);
    void updateUserCommand(cVector3d position, cMatrix3d rotation, bool activateCommand = false);
    void getCameraPose(cVector3d& eye, cVector3d& target);
    void getEndEffectorPose(cVector3d& position, cMatrix3d& rotation);
    void updateCameraPose(cVector3d eye, cVector3d target);
    void moveCamera(cVector3d step);
    void rotateCamera(double yawStep, double pitchStep);
    void switchCameraMode();
    void setEndEffectorStiffness(double posStiffness_, double rotStiffness_);
    void setEndEffectorDamping(double posDamping_, double rotDamping_);
    void setEndEffectorPose(cVector3d position, cMatrix3d rotation);
    void getForceTorqueSensorValue(cVector3d& sensorForce, cVector3d& sensorTorque);

	int getRobotControlMode();
    void moveRobotHome();
	void moveRobotZero();
    void toggleDirectTeachingMode();
	void startTeleoperationMode();
	void quitTeleoperationMode();
	void test();

private:
    void _setWorld();
    void _setIndy();
    void _setRealsense();
	void _createCursor();

public:     // member
    cVector3d renderForce, renderTorque;
    double maxForce, maxTorque;

private:    // member
    //! ODE world
    cWorld* _renderingWorld;

    //! visualization
    cVector3d _cameraEye, _cameraTarget;
    cCamera* _camera;
    cDirectionalLight* _light;
    cSpotLight* _lightIron1;
    cSpotLight* _lightIron2;
    cSpotLight* _lightIron3;
    cSpotLight* _lightIron4;

    cMesh* _cursor;
    cMesh* _indy;

    //! states
    cVector3d _indyPosition;
    cMatrix3d _indyRotation;

    cVector3d _sensorForce;
    cVector3d _sensorTorque;

    //! control variables
    double _workspaceScaleFactor = 30.0;
    double _linStiffness, _angStiffness, _linDamping, _angDamping;

    //! members for visual feedback
    rs2::pointcloud _realSensePointCloudConverter;
    rs2::points _points;
    rs2::pipeline* _realSensePipeline;

    cBitmap *_monitor2D;
    cMultiPoint* _pointCloud;
    cImagePtr _colorImage;

    bool _renderPointCloud = false;

    //! members for Indy7 control
    float _defaultPose[6] = { 0.0f, -45.0f, -135.0f, 0.0f, 75.0f, 0.0f };

    IndyDedicatedTCPTestClient _indyTCP;
    CustomIndyDedicatedTCPTestClient _customTCP;
	cIndyTransform _indyCommand;
    int _indyCmode = 0;
};

#endif

#endif	//	CFURNACE