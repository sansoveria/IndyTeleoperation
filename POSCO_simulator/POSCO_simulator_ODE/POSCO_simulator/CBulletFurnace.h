#pragma once

//---------------------------------------------------------------------------
#ifndef CFURNACE
#define CFURNACE
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CBullet.h"
#include "CCustomBulletWorld.h"
#include "utils.h"
using namespace chai3d;
//---------------------------------------------------------------------------

class cBulletFurnace{
public:
    cBulletFurnace();
    ~cBulletFurnace();

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
    void updateUserCommand(cVector3d position, cMatrix3d rotation);
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


public:     // member
    cVector3d renderForce, renderTorque;


private:    // member

    //! bullet world
    cCustomBulletWorld* _furnaceWorld;

    //! visualization
    cVector3d _cameraEye, _cameraTarget;
    cCamera* _camera;
    cDirectionalLight* _light;
    cSpotLight* _lightIron1;
    cSpotLight* _lightIron2;
    cSpotLight* _lightIron3;
    cSpotLight* _lightIron4;

    //! user cursor (tool)
    cBulletCylinder* _endEffectorTool;
    cBulletCylinder* _endEffectorCursor;
    //btFixedConstraint* _endEffectorJoint;
    btGeneric6DofSpring2Constraint* _endEffectorJoint;
    btJointFeedback* _endEffectorForceTorqueSensor;
    cMaterial* _matEndEffectorSafe;
    cMaterial* _matEndEffectorDang;

    //! static objects (environment)
	cBulletMultiMesh* _wall;
	cBulletMultiMesh* _splashCover;
	cBulletMultiMesh* _outlet;
	cBulletMultiMesh* _fence;
	cBulletMultiMesh* _pouringIron;
	//cBulletMultiMesh* _flowingIron;
    cBulletSphere* _origin;
    cBulletSphere* _cursor;

    //! dynamic objects (target object)
    //cBulletMultiMesh* _depositedIron1, _depositedIron2, _depositedIron3;

    //cBulletStaticPlane* bulletInvisibleWall1;
    //cBulletStaticPlane* bulletGround;
    
    //! control variables
    double _workspaceScaleFactor = 30.0;
    double _linStiffness, _angStiffness, _linDamping, _angDamping;
    double _contactTriangleSize = 0.01;

};
#endif	//	CFURNACE