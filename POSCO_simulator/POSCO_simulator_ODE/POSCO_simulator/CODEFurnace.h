#pragma once

//---------------------------------------------------------------------------
#ifndef CODEFURNACE
#define CODEFURNACE
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CODE.h"
#include "CGenericFurnace.h"
#include "CCustomODEWorld.h"
#include "utils.h"

using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------

#define END_EFFECTOR_TOOL_LENGTH        1.0

class cODEFurnace: cGenericFurnace{
public:
    cODEFurnace();
    virtual ~cODEFurnace();

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
    void updateCameraPose(cVector3d eye, cVector3d target);
    void moveCamera(cVector3d step);
    void rotateCamera(double yawStep, double pitchStep);
    void setEndEffectorStiffness(double posStiffness_, double rotStiffness_);
    void setEndEffectorDamping(double posDamping_, double rotDamping_);
    void setEndEffectorPose(cVector3d position, cMatrix3d rotation);
    void getForceTorqueSensorValue(cVector3d& sensorForce, cVector3d& sensorTorque);


private: 
    void _setWorld();
    void _createStaticObjects();
    void _createEndEffector();
    void _createDynamicObjects();


public:     // member
    cVector3d renderForce, renderTorque;


private:    // member

    //! ODE world
    cWorld* _renderingWorld;
    cCustomODEWorld* _furnaceWorld;

    //! visualization
    cVector3d _cameraEye, _cameraTarget;
    cCamera* _camera;
    cDirectionalLight* _light;
    cSpotLight* _lightIron1;
    cSpotLight* _lightIron2;
    cSpotLight* _lightIron3;
    cSpotLight* _lightIron4;

    //! user cursor (tool)
    cODEGenericBody* _endEffectorTool;
    cMesh* _endEffectorToolMesh;
    cMesh* _endEffectorCursorMesh;
    cMatrix3d localToolCoordRotation;

    cODEGenericBody* _depositedIron;
    cMultiMesh* _depositedIronMesh;

    dJointGroupID jointGroup;
    dJointID jointConstraint;

        //! static objects (environment)
    cODEGenericBody* _wall;
    cODEGenericBody* _splashCover;
    cODEGenericBody* _outlet;
    cODEGenericBody* _fence;
    cODEGenericBody* _pouringIron;
    cODEGenericBody* _flowingIron;

    cMultiMesh* _wallMesh;
    cMultiMesh* _splashCoverMesh;
    cMultiMesh* _outletMesh;
    cMultiMesh* _fenceMesh;
    cMultiMesh* _pouringIronMesh;
    cMesh* _flowingIronMesh;
   
    //! control variables
    double _workspaceScaleFactor = 30.0;
    double _linStiffness, _angStiffness, _linDamping, _angDamping;
    double _contactTriangleSize = 0.01;

};
#endif	//	CFURNACE