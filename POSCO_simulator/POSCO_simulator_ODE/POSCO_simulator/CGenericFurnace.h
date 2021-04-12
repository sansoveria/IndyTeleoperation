#pragma once

//---------------------------------------------------------------------------
#ifndef CGENERICFURNACE
#define CGENERICFURNACE
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CODE.h"
#include "utils.h"

using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------

class cGenericFurnace {
public:
    cGenericFurnace() {}
    virtual ~cGenericFurnace() {}

    virtual bool updateShadowMaps(const bool mirror_x, const bool mirror_y) { return true; }
    virtual void renderView(const int window_width,
        const int window_height,
        const cEyeMode eyeMode = C_STEREO_LEFT_EYE,
        const bool defaultBuffer = true) {}
    virtual void computeGlobalPositions(const bool frameOnly = true,
        const cVector3d& globalPos = cVector3d(0.0, 0.0, 0.0),
        const cMatrix3d& globalRot = cIdentity3d()) {}
    virtual void updateDynamics(double interval) {}
    virtual void addVisualComponent(cGenericObject* object) {}
    virtual void updateUserCommand(cVector3d position, cMatrix3d rotation) {}
    virtual void getCameraPose(cVector3d& eye, cVector3d& target) {}
    virtual void updateCameraPose(cVector3d eye, cVector3d target) {}
    virtual void moveCamera(cVector3d step) {}
    virtual void rotateCamera(double yawStep, double pitchStep) {}
    virtual void setEndEffectorStiffness(double posStiffness_, double rotStiffness_) {}
    virtual void setEndEffectorDamping(double posDamping_, double rotDamping_) {}
    virtual void setEndEffectorPose(cVector3d position, cMatrix3d rotation) {}
    virtual void getForceTorqueSensorValue(cVector3d& sensorForce, cVector3d& sensorTorque) {}


public:     // member
    cVector3d renderForce, renderTorque;


private:    // member
    //! control variables
    double _workspaceScaleFactor = 30.0;
    double _linStiffness, _angStiffness, _linDamping, _angDamping;
    double _contactTriangleSize = 0.01;

};
#endif	//	CFURNACE