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
#include "system/CGlobals.h"
#include "devices/CMyCustomDevice.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/*
    INSTRUCTION TO IMPLEMENT YOUR OWN CUSTOM DEVICE:

    Please review header file CMyCustomDevice.h for some initial 
    guidelines about how to implement your own haptic device using this
    template.

    When ready, simply completed the next 11 documented steps described here
    below.
*/
////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------



//==============================================================================
/*!
    Constructor of cMyCustomDevice.
*/
//==============================================================================
cMyCustomDevice::cMyCustomDevice(unsigned int a_deviceNumber)
{
    // the connection to your device has not yet been established.
    m_deviceReady = false;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 1:

        Here you should define the specifications of your device.
        These values only need to be estimates. Since haptic devices often perform
        differently depending of their configuration withing their workspace,
        simply use average values.
    */
    ////////////////////////////////////////////////////////////////////////////

    //--------------------------------------------------------------------------
    // NAME:
    //--------------------------------------------------------------------------

    // haptic device model (see file "CGenericHapticDevice.h")
    m_specifications.m_model                         = C_HAPTIC_DEVICE_CUSTOM;

    // name of the device manufacturer, research lab, university.
    m_specifications.m_manufacturerName              = "MARCHLAB_POSTECH";

    // name of your device
    m_specifications.m_modelName                     = "AGILE_EYE_2D";


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following values must be positive or equal to zero)
    //--------------------------------------------------------------------------

	// This device only rotates and does not have gripper
    m_specifications.m_maxLinearForce				= 0.0;		// [N]
	m_specifications.m_maxGripperForce				= 0.0;		// [N]
	m_specifications.m_maxLinearStiffness			= 0.0;		// [N/m]
	m_specifications.m_workspaceRadius				= 0.0;		// [m]
	m_specifications.m_maxGripperLinearStiffness	= 0.0;		// [N*m]
	m_specifications.m_gripperMaxAngleRad			= 0.0;


    m_specifications.m_maxAngularTorque				= 0.2;		// [N*m]
    m_specifications.m_maxAngularStiffness          = 1.0;		// [N*m/Rad]
	

    ////////////////////////////////////////////////////////////////////////////
    /*
        DAMPING PROPERTIES:

        Start with small values as damping terms can be high;y sensitive to 
        the quality of your velocity signal and the spatial resolution of your
        device. Try gradually increasing the values by using example "01-devices" 
        and by enabling viscosity with key command "2".
    */
    ////////////////////////////////////////////////////////////////////////////
    
    // This device only rotates and does not have gripper
    m_specifications.m_maxLinearDamping             = 0.0;		// [N/(m/s)]
	m_specifications.m_maxGripperAngularDamping		= 0.0;		// [N*m/(Rad/s)]

    //! Maximum recommended angular damping factor Kv (if actuated torques are available)
    m_specifications.m_maxAngularDamping            = 0.0;		// [N*m/(Rad/s)]
	

    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following are of boolean type: (true or false)
    //--------------------------------------------------------------------------

    // does your device provide sensed position (x,y,z axis)?
    m_specifications.m_sensedPosition                = false;

    // does your device provide sensed rotations (i.e stylus)?
    m_specifications.m_sensedRotation                = true;

    // does your device provide a gripper which can be sensed?
    m_specifications.m_sensedGripper                 = false;

    // is you device actuated on the translation degrees of freedom?
    m_specifications.m_actuatedPosition              = false;

    // is your device actuated on the rotation degrees of freedom?
    m_specifications.m_actuatedRotation              = true;

    // is the gripper of your device actuated?
    m_specifications.m_actuatedGripper               = false;

    // can the device be used with the left hand?
    m_specifications.m_leftHand                      = true;

    // can the device be used with the right hand?
    m_specifications.m_rightHand                     = true;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 2:

        Here, you shall  implement code which tells the application if your
        device is actually connected to your computer and can be accessed.
        In practice this may be consist in checking if your I/O board
        is active or if your drivers are available.

        If your device can be accessed, set:
        m_systemAvailable = true;

        Otherwise set:
        m_systemAvailable = false;

        Your actual code may look like:

        bool result = checkIfMyDeviceIsAvailable()
        m_systemAvailable = result;

        If want to support multiple devices, using the method argument
        a_deviceNumber to know which device to setup
    */  
    ////////////////////////////////////////////////////////////////////////////
        
	nPort = AdsPortOpen();
	nErr = AdsGetLocalAddress(&Addr);
	if (nErr) {
		printf("[TC:ADS] AdsGetLocalAddress Error: %d\n", nErr);
		m_deviceAvailable = C_ERROR;
	}
	else {
		printf("[TC:ADS] ADS Port Opened \n", nErr);
		m_deviceAvailable = C_SUCCESS;
	}

	// TwinCAT 3 Module Port = 350
	(&Addr)->port = 350;
}


//==============================================================================
/*!
    Destructor of cMyCustomDevice.
*/
//==============================================================================
cMyCustomDevice::~cMyCustomDevice()
{
    // close connection to device
    if (m_deviceReady)
    {
        close();
    }
}


//==============================================================================
/*!
    This method opens a connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 3:

        Here you shall implement to open a connection to your
        device. This may include opening a connection to an interface board
        for instance or a USB port.

        If the connection succeeds, set the variable 'result' to true.
        otherwise, set the variable 'result' to false.

        Verify that your device is calibrated. If your device 
        needs calibration then call method calibrate() for wich you will 
        provide code in STEP 5 further below.
    */
    ////////////////////////////////////////////////////////////////////////////

	// TODO: Motor Enable Commands
	USHORT enableMotors = ENABLE_MOTOR;
	bool motor1Ready, motor2Ready;

	nErr = AdsSyncWriteReq(&Addr, INDEX_GROUP, MOTOR1_ENABLE, sizeof(enableMotors), &enableMotors);
	if (nErr != 0) {
		printf("[TC:ADS] MOTOR1 Enable Error: %d\n", nErr);
		motor1Ready = C_ERROR;
	}
	else {
		motor1Ready = C_SUCCESS;
	}

	nErr = AdsSyncWriteReq(&Addr, INDEX_GROUP, MOTOR2_ENABLE, sizeof(enableMotors), &enableMotors);
	if (nErr != 0) {
		printf("[TC:ADS] MOTOR2 Enable Error: %d\n", nErr);
		motor2Ready = C_ERROR;
	}
	else {
		motor2Ready = C_SUCCESS;
	}

	m_deviceReady = motor1Ready && motor2Ready;

	return m_deviceReady;
}


//==============================================================================
/*!
    This method closes the connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 4:

        Here you shall implement code that closes the connection to your
        device.

        If the operation fails, simply set the variable 'result' to C_ERROR   .
        If the connection succeeds, set the variable 'result' to C_SUCCESS.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

	nErr = AdsPortClose();
	if (nErr != 0) {
		printf("[TC:ADS] ADS Port Close Error: %d\n", nErr);
		result = C_ERROR;
	}
	else {
		result = C_SUCCESS;
	}

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
bool cMyCustomDevice::calibrate(bool a_forceCalibration)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 5:
        
        Here you shall implement code that handles a calibration procedure of the 
        device. In practice this may include initializing the registers of the
        encoder counters for instance. 

        If the device is already calibrated and  a_forceCalibration == false,
        the method may immediately return without further action.
        If a_forceCalibration == true, then the calibrartion procedure
        shall be executed even if the device has already been calibrated.
 
        If the calibration procedure succeeds, the method returns C_SUCCESS,
        otherwise return C_ERROR.
    */
    ////////////////////////////////////////////////////////////////////////////

	printf("[Calibration] Leave handle toward ground within 3 seconds");
	for (int i = 0; i < 3; i++) {
		Sleep(1000);
		printf("[Calibration] %d seconds remain \n", 3-i);
	}
	printf("[Calibration] Calibration Done \n");

	INT32 motor1_pos, motor2_pos;
	nErr = AdsSyncReadReq(&Addr, INDEX_GROUP, MOTOR1_ANGLE, sizeof(motor1_pos), &motor1_pos);
	if (nErr != 0) {
		printf("[TC:ADS] Motor1 Read Error: %d\n", nErr);		
		return (C_ERROR);
	}
	else {
		// Calibration orientation: theta1 = -M_PI/2.0, theta2 = 0.0
		m_motor1_zero_position = motor1_pos + 1024 * 5 * 4 / 4;
	}

	nErr = AdsSyncReadReq(&Addr, INDEX_GROUP, MOTOR2_ANGLE, sizeof(motor2_pos), &motor2_pos);
	if (nErr != 0) {
		printf("[TC:ADS] Motor2 Read Error: %d\n", nErr);
		return (C_ERROR);
	}
	else {
		m_motor2_zero_position = motor2_pos;
	}

    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method returns the number of devices available from this class of device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
unsigned int cMyCustomDevice::getNumDevices()
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 6:

        Here you shall implement code that returns the number of available
        haptic devices of type "cMyCustomDevice" which are currently connected
        to your computer.

        In practice you will often have either 0 or 1 device. In which case
        the code here below is already implemented for you.

        If you have support more than 1 devices connected at the same time,
        then simply modify the code accordingly so that "numberOfDevices" takes
        the correct value.
    */
    ////////////////////////////////////////////////////////////////////////////

    int numberOfDevices = 1;  // At least set to 1 if a device is available.

    return (numberOfDevices);
}


//==============================================================================
/*!
    This method returns the position of your device. Units are meters [m].

    \param   a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::getPosition(cVector3d& a_position)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 7:

        Here you shall implement code that reads the position (X,Y,Z) from
        your haptic device. Read the values from your device and modify
        the local variable (x,y,z) accordingly.
        If the operation fails return an C_ERROR, C_SUCCESS otherwise

        Note:
        For consistency, units must be in meters.
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky. 
    */
    ////////////////////////////////////////////////////////////////////////////

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


//==============================================================================
/*!
    This method returns the orientation frame of your device end-effector

    \param   a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 8:

        Here you shall implement code which reads the orientation frame from
        your haptic device. The orientation frame is expressed by a 3x3
        rotation matrix. The 1st column of this matrix corresponds to the
        x-axis, the 2nd column to the y-axis and the 3rd column to the z-axis.
        The length of each column vector should be of length 1 and vectors need
        to be orthogonal to each other.

        Note:
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        If your device has a stylus, make sure that you set the reference frame
        so that the x-axis corresponds to the axis of the stylus.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // variables that describe the rotation matrix
    cMatrix3d frame;
    frame.identity();

	INT32 motor1_cnt, motor2_cnt;
	double motor1_angle, motor2_angle;
	nErr = AdsSyncReadReq(&Addr, INDEX_GROUP, MOTOR1_ANGLE, sizeof(motor1_cnt), &motor1_cnt);
	if (nErr != 0) {
		printf("[TC:ADS] Motor1 Read Error: %d\n", nErr);
		return (C_ERROR);
	}
	else {
		motor1_angle = 2.0*M_PI / 1024.0 / 5.0 / 4.0*(motor1_cnt - m_motor1_zero_position);
	}

	nErr = AdsSyncReadReq(&Addr, INDEX_GROUP, MOTOR2_ANGLE, sizeof(motor2_cnt), &motor2_cnt);
	if (nErr != 0) {
		printf("[TC:ADS] Motor2 Read Error: %d\n", nErr);
		return (C_ERROR);
	}
	else {
		motor2_angle = 2.0*M_PI / 1024.0 / 5.0 / 4.0*(motor2_cnt - m_motor2_zero_position);
	}

    //// if the device does not provide any rotation capabilities 
    //// set the rotation matrix equal to the identity matrix.
	//double r00, r01, r02, r10, r11, r12, r20, r21, r22;

    //r00 = 1.0;  r01 = 0.0;  r02 = 0.0;
    //r10 = 0.0;  r11 = 1.0;  r12 = 0.0;
    //r20 = 0.0;  r21 = 0.0;  r22 = 1.0;

    //frame.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);

	calcForwardKinematics(motor1_angle, motor2_angle, frame);

    // store new rotation matrix
    a_rotation = frame;

    // estimate angular velocity
    estimateAngularVelocity(a_rotation);

	// calculate Jacobian
	calcJacobian(motor1_angle, motor2_angle);

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
bool cMyCustomDevice::getGripperAngleRad(double& a_angle)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 9:
        Here you may implement code which reads the position angle of your
        gripper. The result must be returned in radian.

        If the operation fails return an error code such as C_ERROR for instance.
    */
    ////////////////////////////////////////////////////////////////////////////

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
bool cMyCustomDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                       const cVector3d& a_torque,
                                                       const double a_gripperForce)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 10:
        
        Here you may implement code which sends a force (fx,fy,fz),
        torque (tx, ty, tz) and/or gripper force (gf) command to your haptic device.

        If your device does not support one of more of the force, torque and 
        gripper force capabilities, you can simply ignore them. 

        Note:
        For consistency, units must be in Newtons and Newton-meters
        If your device is placed in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        For instance: if the force = (1,0,0), the device should move towards
        the operator, if the force = (0,0,1), the device should move upwards.
        A torque (1,0,0) would rotate the handle counter clock-wise around the 
        x-axis.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // store new force value.
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperForce = a_gripperForce;

    // retrieve force, torque, and gripper force components in individual variables
    double fx = a_force(0);
    double fy = a_force(1);
    double fz = a_force(2);

    double tx = a_torque(0);
    double ty = a_torque(1);
    double tz = a_torque(2);

    double gf = a_gripperForce;

    // *** INSERT YOUR CODE HERE ***
	//////////////////// PASS FOR THIS DEVICE /////////////////////
    // setForceToMyDevice(fx, fy, fz);
    // setForceToGripper(fg);

	double motor_torque1, motor_torque2;
	motor_torque1 = m_Jacobian(0, 0)*tx + m_Jacobian(1, 0)*ty + m_Jacobian(2, 0)*tz;
	motor_torque2 = m_Jacobian(0, 0)*tx + m_Jacobian(1, 0)*ty + m_Jacobian(2, 0)*tz;

	nErr = AdsSyncWriteReq(&Addr, INDEX_GROUP, MOTOR1_TORQUE, sizeof(motor_torque1), &motor_torque1);
	if (nErr != 0) {
		printf("[TC:ADS] MOTOR1 Write Torque Error: %d\n", nErr);
		return(C_ERROR);
	}
	else {
		result = C_SUCCESS;
	}

	nErr = AdsSyncWriteReq(&Addr, INDEX_GROUP, MOTOR2_TORQUE, sizeof(motor_torque2), &motor_torque2);
	if (nErr != 0) {
		printf("[TC:ADS] MOTOR2 Write Torque Error: %d\n", nErr);
		return(C_ERROR);
	}
	else {
		result = C_SUCCESS;
	}

    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns status of all user switches 
    [__true__ = __ON__ / __false__ = __OFF__].

    \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMyCustomDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 11:

        Here you shall implement code that reads the status all user switches 
        on your device. For each user switch, set the associated bit on variable
        a_userSwitches. If your device only has one user switch, then set 
        a_userSwitches to 1, when the user switch is engaged, and 0 otherwise.
    */
    ////////////////////////////////////////////////////////////////////////////

    // *** INSERT YOUR CODE HERE ***
	//////////////////// PASS FOR THIS DEVICE /////////////////////

    a_userSwitches = 0;

    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
