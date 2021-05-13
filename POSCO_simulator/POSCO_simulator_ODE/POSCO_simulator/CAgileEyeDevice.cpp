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
    // TODO: check available ports
    m_deviceAvailable = C_SUCCESS;
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
    // TODO: remove new objects if they are created.
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
    mcu_input.calibrationFlag = false;
    mcu_input.u = 0;
    mcu_input.v = 0;
    mcu_input.w = 0;
    mcu_input.message_idx = 0;

    mcu_output.calibrationFlag = false;
    mcu_output.enableMotors = false;
    mcu_output.tau1 = 0;
    mcu_output.tau2 = 0;
    mcu_output.tau3 = 0;
    mcu_output.message_idx = 0;

    mcu_output.enableMotors = true;
    printf("aInput size: %d, aOutput size: %d \n", sizeof(MCU_INPUT), sizeof(MCU_OUTPUT));

    char portname[255] = "//./COM4";
    if (!serialComm.connect(portname)){
        cout << "[AGILE_EYE-open] serial port connection faliled" << endl;     
        return (C_ERROR);
    }
    else{
        cout << "[AGILE_EYE-open] serial port connected" << endl;
    }

    mcu_output.setMessage();
    if (!serialComm.sendCommand(mcu_output.message, 26)) {
        printf("[AGILE_EYE-open] Error: sending message Failed\n");
        return (C_ERROR);
    }

    m_deviceReady = false;    
    //for (int i = 0; i < 50; i++) {
    //    if (!serialComm.receiveCommand(mcu_input.message, 30)) {
    //        printf("[AGILE_EYE-open] Error: reading message Failed\n");
    //    }
    //    mcu_input.parseMessage();
    //    if (mcu_input.message_idx == mcu_output.message_idx) {
    //        m_deviceReady = true;
    //        return (C_SUCCESS);
    //    }
    //    serialComm.purgeconnect();

    //    mcu_output.setMessage();
    //    if (!serialComm.sendCommand(mcu_output.message, 26)) {
    //        printf("[AGILE_EYE-open] Error: sending message Failed\n");
    //    }
    //}
    if (!serialComm.receiveCommand(mcu_input.message, 30)) {
        printf("[AGILE_EYE-open] Error: reading message Failed\n");
        return (C_ERROR);
    }
    mcu_input.parseMessage();
    //if (mcu_input.message_idx == mcu_output.message_idx) {
    //    m_deviceReady = true;
    //    return (C_SUCCESS);
    //}
    m_deviceReady = true;
    serialComm.purgeconnect();
#endif
	return (C_SUCCESS);
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
    printf("[AgileEye-close] Device is closed\n");
    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

#if defined(USE_AGILE_EYE)
    mcu_output.tau1 = LONG(0);
    mcu_output.tau2 = LONG(0);
    mcu_output.tau3 = LONG(0);
    mcu_output.enableMotors = false;

    mcu_output.setMessage();

    if (!serialComm.sendCommand(mcu_output.message, 26)) {
        printf("[AGILE_EYE-close] Error: sending message Failed\n");
        return (C_ERROR);
    }
    if (!serialComm.receiveCommand(mcu_input.message, 30)) {
        printf("[AGILE_EYE-close] Error: reading message Failed\n");
        return (C_ERROR);
    }
    mcu_input.parseMessage();
    serialComm.purgeconnect();

    serialComm.disconnect();
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
    mcu_output.tau1 = LONG(0);
    mcu_output.tau2 = LONG(0);
    mcu_output.tau3 = LONG(0);
    mcu_output.enableMotors = false;

    mcu_output.setMessage();
    if (!serialComm.sendCommand(mcu_output.message, 26)) {
        printf("[AGILE_EYE-calibrate] Error: sending message Failed\n");
        return (C_ERROR);
    }
    
    if (!serialComm.receiveCommand(mcu_input.message, 30)) {
        printf("[AGILE_EYE-calibrate] Error: reading message Failed\n");
        return (C_ERROR);
    }
    mcu_input.parseMessage();
    serialComm.purgeconnect();

    mcu_output.enableMotors = true;
    mcu_output.setMessage();
    if (!serialComm.sendCommand(mcu_output.message, 26)) {
        printf("[AGILE_EYE-calibrate] Error: sending message Failed\n");
        return (C_ERROR);
    }
    serialComm.purgeconnect();
    if (!serialComm.receiveCommand(mcu_input.message, 30)) {
        printf("[AGILE_EYE-calibrate] Error: reading message Failed\n");
        return (C_ERROR);
    }
    mcu_input.parseMessage();
    
    // set zero angle
    if (mcu_output.calibrationFlag) mcu_output.calibrationFlag = false;
    else mcu_output.calibrationFlag = true;
    mcu_output.setMessage();
    if (!serialComm.sendCommand(mcu_output.message, 26)) {
        printf("[AGILE_EYE-calibrate] Error: sending message Failed\n");
        return (C_ERROR);
    }
    
    if (!serialComm.receiveCommand(mcu_input.message, 30)) {
        printf("[AGILE_EYE-calibrate] Error: reading message Failed\n");
        return (C_ERROR);
    }
    serialComm.purgeconnect();
    mcu_input.parseMessage();
#endif

    return C_SUCCESS;
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

    mcu_output.setMessage();
    if (!serialComm.sendCommand(mcu_output.message, 26)) {
        printf("[AGILE_EYE-calibrate] Error: sending message Failed\n");
        return (C_ERROR);
    }
    if (!serialComm.receiveCommand(mcu_input.message, 30)) {
        printf("[AGILE_EYE-calibrate] Error: reading message Failed\n");
        return (C_ERROR);
    }
    mcu_input.parseMessage();
    serialComm.purgeconnect();

    double axis1, axis2, axis3, angle;
    angle = sqrt(mcu_input.u * mcu_input.u + mcu_input.v * mcu_input.v + mcu_input.w * mcu_input.w);
    axis1 = mcu_input.u;
    axis2 = mcu_input.v;
    axis3 = mcu_input.w;
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
    mcu_output.tau1 = tx;
    mcu_output.tau2 = ty;
    mcu_output.tau3 = tz;
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
