#pragma once
#pragma once

//---------------------------------------------------------------------------
#ifndef CINDYFURNACE
#define CINDYFURNACE
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CGenericFurnace.h"
#include "utils.h"
//---------------------------------------------------------------------------
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <algorithm>            // std::min, std::max
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------

#define END_EFFECTOR_TOOL_LENGTH        1.0

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
    void _setIndy();
    void _setPointCloud();

public:     // member
    cVector3d renderForce, renderTorque;


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

    //! members for RealSense L515
    rs2::pointcloud _realSensePointCloudConverter;
    rs2::points _points;
    cMultiPoint* _pointCloud;
    cImagePtr _pointCloudTextureMap;
    rs2::pipeline* _realSensePipeline;

};
#endif	//	CFURNACE