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
    \version   3.2.0 $Rev: 1875 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CMyCustomDeviceH
#define CMyCustomDeviceH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
#include "C:\TwinCAT\AdsApi\TcAdsDll\Include\TcAdsDef.h"
#include "C:\TwinCAT\AdsApi\TcAdsDll\Include\TcAdsAPI.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMyCustomDevice.h

    \brief
    Implements support for custom haptic device.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cMyCustomDevice;
typedef std::shared_ptr<cMyCustomDevice> cMyCustomDevicePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cMyCustomDevice
    \ingroup    devices  

    \brief
    This class is a interface to support custom haptic devices (template).

    \details
    This class provides the basics to easily interface CHAI3D to your 
    own custom haptic device. \n\n

    Simply follow the 11 commented step in file CMyCustomDevice.cpp 
    and complete the code accordingly.
    Depending of the numbers of degrees of freedom of your device, not
    all methods need to be implemented. For instance, if your device
    does not provide any rotation degrees of freedom, simply ignore
    the getRotation() method. Default values will be returned correctly
    if these are not implemented on your device. In the case of rotations
    for instance, the identity matrix is returned.\n\n

    You may also rename this class in which case you will also want to
    customize the haptic device handler to automatically detect your device.
    Please consult method update() of the cHapticDeviceHandler class
    that is located in file CHapticDeviceHandler.cpp .
    Simply see how the haptic device handler already looks for
    device of type cMyCustomDevice.\n\n

    If you are encountering any problems with your implementation, check 
    for instance file cDeltaDevices.cpp which implement supports for the 
    Force Dimension series of haptic devices. In order to verify the implementation
    use the 01-device example to get started. Example 11-effects is a great
    demo to verify how basic haptic effects may behave with you haptic
    devices. If you do encounter vibrations or instabilities, try reducing
    the maximum stiffness and/or damping values supported by your device. 
    (see STEP-1 in file CMyCustomDevice.cpp).\n
    
    Make  sure that your device is also communicating fast enough with 
    your computer. Ideally the communication period should take less 
    than 1 millisecond in order to reach a desired update rate of at least 1000Hz.
    Problems can typically occur when using a slow serial port (RS232) for
    instance.\n
*/
//==============================================================================
class cMyCustomDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cMyCustomDevice.
    cMyCustomDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cMyCustomDevice.
    virtual ~cMyCustomDevice();

    //! Shared cMyCustomDevice allocator.
    static cMyCustomDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cMyCustomDevice>(a_deviceNumber)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method opens a connection to the haptic device.
    virtual bool open();

    //! This method closes the connection to the haptic device.
    virtual bool close();

    //! This method calibrates the haptic device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! This method returns the position of the device.
    virtual bool getPosition(cVector3d& a_position);

    //! This method returns the orientation frame of the device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! This method returns the gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitches(unsigned int& a_userSwitches); 

    //! This method sends a force [N] and a torque [N*m] and gripper force [N] to the haptic device.
    virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public: 

    //! This method returns the number of devices available from this class of device.
    static unsigned int getNumDevices();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////
    /*
        INTERNAL VARIABLES:

        If you need to declare any local variables or methods for your device,
        you may do it here below. 
    */
    ////////////////////////////////////////////////////////////////////////////

protected:
	// Variables for TwinCAT ADS communication
	long						nErr;						// Error Message
	long						nPort;						// Port number
	AmsAddr						Addr;						// Target port information(PLC1)

	INT32		m_motor1_zero_position, m_motor2_zero_position;
	cMatrix3d	m_Jacobian;
	
#define	INDEX_GROUP		0x1010010
#define	MOTOR1_ENABLE	0x81000000
#define	MOTOR1_ANGLE	0x81000000
#define	MOTOR1_TORQUE	0x81000000
#define	MOTOR2_ENABLE	0x81000000
#define	MOTOR2_ANGLE	0x81000000
#define	MOTOR2_TORQUE	0x81000000

#define	ENABLE_MOTOR	0x000F
#define	DISABLE_MOTOR	0x0006

protected:

	void calcForwardKinematics(double angle1, double angle2, cMatrix3d& rot) {
		// TODO: implement Forward Kinematics calculation
		double ca, sa, c1, s1, c2, s2;
		c1 = cos(angle1);
		s1 = sin(angle1);
		c2 = cos(angle2);
		s2 = sin(angle2);
		ca = c1 * c2 / sqrt(c1*c1*c2*c2 + s2 * s2);
		sa = -s2 / sqrt(c1*c1*c2*c2 + s2 * s2);

		double r00, r01, r02, r10, r11, r12, r20, r21, r22;

		r00 = ca*c1;		r01 = -sa*c1;		r02 = -s1;
		r10 = sa;			r11 = ca;			r12 = 0.0;
		r20 = ca * s1;		r21 = -sa * s1;		r22 = c1;

		rot.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);
	}

	void calcJacobian(double angle1, double angle2) {
		// TODO: implement Jacobian calculation
		double c1, s1, c2, s2, A;
		c1 = cos(angle1);
		s1 = sin(angle1);
		c2 = cos(angle2);
		s2 = sin(angle2);
		A = c1 * c1*c2*c2 + s2 * s2;

		double j00, j01, j02, j10, j11, j12, j20, j21, j22;

		j00 = -s2/sqrt(A);		j01 = 0.0;		j02 = 0.0;
		j10 = -c1*c2/sqrt(A);	j11 = 0.0;		j12 = 0.0;
		j20 = s1*c2*s2/A;		j21 = c1/A;		j22 = 0.0;

		m_Jacobian.set(j00, j01, j02, j10, j11, j12, j20, j21, j22);
	}
};

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
