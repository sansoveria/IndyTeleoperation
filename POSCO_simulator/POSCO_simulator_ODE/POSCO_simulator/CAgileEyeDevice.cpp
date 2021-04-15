//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Your name, institution, or company name.
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================


//------------------------------------------------------------------------------
#include "definitions.h"
#include "CAgileEyeDevice.h"

#include "utils.h"


#if defined(USE_AGILE_EYE) {
#ifdef WIN64
#   pragma comment( lib, "C:/TwinCAT/AdsApi/TcAdsDll/x64/lib/TcAdsDll.lib" )
#else
#   pragma comment( lib, "C:/TwinCAT/AdsApi/TcAdsDll/Lib/TcAdsDll.lib" )
#endif
#endif

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------



//==============================================================================
/*!
    Constructor of cMyCustomDevice.
*/
//==============================================================================
cAgileEyeDevice::cAgileEyeDevice(unsigned int a_deviceNumber)
{
    // the connection to your device has not yet been established.
    m_deviceReady = false;

    // haptic device model (see file "CGenericHapticDevice.h")
    m_specifications.m_model                         = C_HAPTIC_DEVICE_CUSTOM;

    // name of the device manufacturer, research lab, university.
    m_specifications.m_manufacturerName              = "MARCHLAB_POSTECH";

    // name of your device
    m_specifications.m_modelName                     = "AGILE_EYE_2D";

	// This device only rotates and does not have gripper
    m_specifications.m_maxLinearForce				= 0.0;		// [N]
	m_specifications.m_maxGripperForce				= 0.0;		// [N]
	m_specifications.m_maxLinearStiffness			= 0.0;		// [N/m]
	m_specifications.m_workspaceRadius				= 0.0;		// [m]
	m_specifications.m_maxGripperLinearStiffness	= 0.0;		// [N*m]
	m_specifications.m_gripperMaxAngleRad			= 0.0;
    m_specifications.m_maxAngularTorque				= 0.01;		// [N*m]
    m_specifications.m_maxAngularStiffness          = 1.0;		// [N*m/Rad]	
    m_specifications.m_maxLinearDamping             = 0.0;		// [N/(m/s)]
	m_specifications.m_maxGripperAngularDamping		= 0.0;		// [N*m/(Rad/s)]
    m_specifications.m_maxAngularDamping            = 0.0;		// [N*m/(Rad/s)]
    m_specifications.m_sensedPosition                = false;
    m_specifications.m_sensedRotation                = true;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = false;
    m_specifications.m_actuatedRotation              = true;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;


#if defined(USE_AGILE_EYE){
    nPort = AdsPortOpen();
    nErr = AdsGetLocalAddress(&Addr);
    if (nErr) {
        printf("[AGILE_EYE] AdsGetLocalAddress Error: %d\n", nErr);
        m_deviceAvailable = C_ERROR;
    }
    else {
        printf("[AGILE_EYE] ADS Port Opened \n");
        m_deviceAvailable = C_SUCCESS;
    }

    // TwinCAT 3 Module Port = 350
    (&Addr)->port = 350;

    // Initialize motor status
    aOutput.enableMotor1 = false;
    aOutput.enableMotor2 = false;
    aOutput.calibrationFlag = false;
    aOutput.tau1 = 0;
    aOutput.tau2 = 0;
    aOutput.tau3 = 0;

    nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000000, sizeof(ADS_OUTPUT), &aOutput);
    if (nErr != 0) {
        printf("[AGILE_EYE] Error: AdsSyncWriteReq Failed: %d\n", nErr);
    }
#else
    m_deviceAvailable = C_SUCCESS;
#endif
}


//==============================================================================
/*!
    Destructor of cMyCustomDevice.
*/
//==============================================================================
cAgileEyeDevice::~cAgileEyeDevice()
{
    // close connection to device
    if (m_deviceReady)
    {
        close();
    }
#if defined(USE_AGILE_EYE)
    nErr = AdsPortClose();
    if (nErr != 0) {
        printf("[AGILE_EYE] Error: ADS Port Close Error: %d\n", nErr);
    }
    else {
        printf("[AGILE_EYE] ADS Port Closed \n");
    }
#endif
}


//==============================================================================
/*!
    This method opens a connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAgileEyeDevice::open(){

    // check if the system is available
    if (!m_deviceAvailable) {
        printf("[AgileEye-open] Device is not available\n");
        return (C_ERROR);
    }

    // if system is already opened then return
    if (m_deviceReady) {
        printf("[AgileEye-open] Device is already opened\n");
        return (C_ERROR);
    }

  
#if defined(USE_AGILE_EYE)
    // reset ADS communication variable
    aInput.angle1 = 0;
    aInput.angle2 = 0;
    aInput.angleZero1 = 0;
    aInput.angleZero2 = 0;
    aInput.calibrationFlag = false;
    aInput.motorState1 = false;
    aInput.motorState2 = false;
    aInput.u = 0;
    aInput.v = 0;
    aInput.w = 0;

    aOutput.calibrationFlag = false;
    aOutput.enableMotor1 = false;
    aOutput.enableMotor2 = false;
    aOutput.tau1 = 0;
    aOutput.tau2 = 0;
    aOutput.tau3 = 0;

    aOutput.enableMotor1 = true;
    aOutput.enableMotor2 = true;
    printf("aInput size: %d, aOutput size: %d \n", sizeof(ADS_INPUT), sizeof(ADS_OUTPUT));
    nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000000, sizeof(ADS_OUTPUT), &aOutput);
    if (nErr != 0) {
        printf("[AGILE_EYE] Error: AdsSyncWriteReq Failed: %d\n", nErr);
    }

    m_deviceReady = false;
    for (int i = 0; i < 50; i++) {
        nErr = AdsSyncReadReq(&Addr, 0x1010010, 0x82000000, sizeof(ADS_INPUT), &aInput);
        if (nErr != 0) {
            printf("[AGILE_EYE] Error: AdsSyncReadReq Failed: %d\n", nErr);
        }

        m_deviceReady = aInput.motorState1 && aInput.motorState2;
        printf("angle1: %d, angle2: %d \n", aInput.angle1, aInput.angle2);
        if (!aInput.motorState1) printf("[AGILE_EYE] Trial %d, Failed to enable motor1\n", i);
        if (!aInput.motorState2) printf("[AGILE_EYE] Trial %d, Failed to enable motor2\n", i);
        if (m_deviceReady) break;
    }
#endif
	return m_deviceReady;
}


//==============================================================================
/*!
    This method closes the connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAgileEyeDevice::close(){
    // check if the system has been opened previously
    if (!m_deviceReady) {
        printf("[AgileEye-close] Device is not opened\n");
        return (C_ERROR);
    }

    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

#if defined(USE_AGILE_EYE)
    aOutput.tau1 = LONG(0);
    aOutput.tau2 = LONG(0);
    aOutput.tau3 = LONG(0);
    aOutput.enableMotor1 = false;
    aOutput.enableMotor2 = false;

    nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000000, sizeof(ADS_OUTPUT), &aOutput);
    if (nErr != 0) {
        printf("[AGILE_EYE] Error: AdsSyncWriteReq Failed: %d\n", nErr);
        result = C_ERROR;
    }
#endif

    // update status
    m_deviceReady = false;

    return (result);
}


//==============================================================================
/*!
    This method calibrates your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAgileEyeDevice::calibrate(bool a_forceCalibration)
{
    if (!m_deviceReady) return (C_ERROR);

    bool result = false;
#if defined(USE_AGILE_EYE)
	// disable and enable motors (reset motors)
    aOutput.enableMotor1 = false;
    aOutput.enableMotor2 = false;
    nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000000, sizeof(ADS_OUTPUT), &aOutput);
    if (nErr != 0) {
        printf("[TC:ADS] Error: AdsSyncWriteReq Failed: %d\n", nErr);
    }

    aOutput.enableMotor1 = true;
    aOutput.enableMotor2 = true;
    nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000000, sizeof(ADS_OUTPUT), &aOutput);
    if (nErr != 0) {
        printf("[AGILE_EYE] Error: AdsSyncWriteReq Failed: %d\n", nErr);
    }

    // set zero angle
    if (aOutput.calibrationFlag) aOutput.calibrationFlag = false;
    else aOutput.calibrationFlag = true;
    nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000000, sizeof(ADS_OUTPUT), &aOutput);
    if (nErr != 0) {
        printf("[AGILE_EYE] Error: AdsSyncWriteReq Failed: %d\n", nErr);
    }

    if (abs(aInput.angle1) < 50 && abs(aInput.angle2) < 50) return (C_SUCCESS);
    else return (C_ERROR);
#endif

    return result;
}


//==============================================================================
/*!
    This method returns the number of devices available from this class of device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
unsigned int cAgileEyeDevice::getNumDevices(){  
#if defined(USE_AGILE_EYE)
    unsigned int numberOfDevices = 1;  // At least set to 1 if a device is available.
#else
    unsigned int numberOfDevices = 0;
#endif

    return (numberOfDevices);
}


//==============================================================================
/*!
    This method returns the position of your device. Units are meters [m].

    \param   a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAgileEyeDevice::getPosition(cVector3d& a_position)
{
    if (!m_deviceReady) return (C_ERROR);

    bool result = C_SUCCESS;
    double x,y,z;

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***
	//////////////////// PASS FOR THIS DEVICE /////////////////////

    x = 0.0;    // x = getMyDevicePositionX()
    y = 0.0;    // y = getMyDevicePositionY()
    z = 0.0;    // z = getMyDevicePositionZ()

    // store new position values
    a_position.set(x, y, z);

    // estimate linear velocity
    estimateLinearVelocity(a_position);

    // exit
    return (result);
}

void ceiling(double x, double y, double z, cMatrix3d& res) {
    res.set(0, -z, y, z, 0, -x, -y, x, 0);
}

//==============================================================================
/*!
    This method returns the orientation frame of your device end-effector

    \param   a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAgileEyeDevice::getRotation(cMatrix3d& a_rotation){
    if (!m_deviceReady) return (C_ERROR);
       
    bool result = C_SUCCESS;

#if defined(USE_AGILE_EYE)
    nErr = AdsSyncReadReq(&Addr, 0x1010010, 0x82000000, sizeof(ADS_INPUT), &aInput);
    if (nErr != 0) {
        printf("[AGILE_EYE] Error: AdsSyncReadReq Failed: %d\n", nErr);
        return (C_ERROR);
    }

    if (!aInput.motorState1 || !aInput.motorState2) {
        printf("[AGILE_EYE] Motor state is not ready \n");
        m_deviceReady = false;

        printf("[AGILE_EYE] restore loop start\n");
        for (int i = 0; i < 30; i++) {
            nErr = AdsSyncReadReq(&Addr, 0x1010010, 0x82000000, sizeof(ADS_INPUT), &aInput);
            if (nErr != 0) {
                printf("[AGILE_EYE] Error: AdsSyncReadReq Failed: %d\n", nErr);
            }

            m_deviceReady = aInput.motorState1 && aInput.motorState2;
            if (!aInput.motorState1) printf("[AGILE_EYE] Trial %d, Failed to enable motor1\n", i);
            if (!aInput.motorState2) printf("[AGILE_EYE] Trial %d, Failed to enable motor2\n", i);
            if (m_deviceReady) break;
        }
        printf("[AGILE_EYE] restore loop end\n");

        return C_ERROR;
    }

    double axis1, axis2, axis3, angle;
    angle = sqrt(aInput.u * aInput.u + aInput.v * aInput.v + aInput.w * aInput.w)/1000.0;
    axis1 = aInput.u / 1000.0;
    axis2 = aInput.v / 1000.0;
    axis3 = aInput.w / 1000.0;
    if (angle > 0) {
        a_rotation.setAxisAngleRotationRad(cVector3d(axis1, axis2, axis3), angle);
    }
    else {
        a_rotation.set(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    }

    a_rotation = cMul(rotateX(M_PI/2.0), a_rotation);
#endif
    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns the gripper angle in radian.

    \param   a_angle  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAgileEyeDevice::getGripperAngleRad(double& a_angle)
{
    if (!m_deviceReady) return (C_ERROR);

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***
	//////////////////// PASS FOR THIS DEVICE /////////////////////


    // return gripper angle in radian
    a_angle = 0.0;  // a_angle = getGripperAngleInRadianFromMyDevice();

    // estimate gripper velocity
    estimateGripperVelocity(a_angle);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method sends a force [N] and a torque [N*m] and gripper torque [N*m] 
    to your haptic device.

    \param   a_force  Force command.
    \param   a_torque  Torque command.
    \param   a_gripperForce  Gripper force command.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAgileEyeDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                       const cVector3d& a_torque,
                                                       const double a_gripperForce)
{
    if (!m_deviceReady) return (C_ERROR);

    bool result = C_SUCCESS;

    // store new force value.
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperForce = a_gripperForce;

    // retrieve force, torque, and gripper force components in individual variables
    double fx = a_force(0);
    double fy = a_force(1);
    double fz = a_force(2);

    cVector3d torqueTransformed = cMul(rotateX(M_PI / 2.0), a_torque);
    double tx = torqueTransformed(0);
    double ty = torqueTransformed(1);
    double tz = torqueTransformed(2);

    double gf = a_gripperForce;

    // *** INSERT YOUR CODE HERE ***
	//////////////////// PASS FOR THIS DEVICE /////////////////////
    // setForceToMyDevice(fx, fy, fz);
    // setForceToGripper(fg);
#if defined(USE_AGILE_EYE)
    aOutput.tau1 = INT(tx * 1000);
    aOutput.tau2 = INT(ty * 1000);
    aOutput.tau3 = INT(tz * 1000);

    nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000000, sizeof(ADS_OUTPUT), &aOutput);
    if (nErr != 0) {
        printf("[AGILE_EYE] Error: AdsSyncWriteReq Failed: %d\n", nErr);
        return C_ERROR;
    }
#endif
    return C_SUCCESS;
}


//==============================================================================
/*!
    This method returns status of all user switches 
    [__true__ = __ON__ / __false__ = __OFF__].

    \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAgileEyeDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    if (!m_deviceReady) return (C_ERROR);

    // *** INSERT YOUR CODE HERE ***
	//////////////////// PASS FOR THIS DEVICE /////////////////////

    a_userSwitches = 0;

    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
