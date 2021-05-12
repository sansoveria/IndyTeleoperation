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
#ifndef CAgileEyeDeviceDeviceH
#define CAgileEyeDeviceDeviceH
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------
#if defined(USE_AGILE_EYE)
#include "CSerialComm.h"
#endif
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
class cAgileEyeDevice;
typedef std::shared_ptr<cAgileEyeDevice> cAgileEyeDevicePtr;
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
class cAgileEyeDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cMyCustomDevice.
    cAgileEyeDevice
    (unsigned int a_deviceNumber = 0);

    //! Destructor of cMyCustomDevice.
    virtual ~cAgileEyeDevice
    ();

    //! Shared cMyCustomDevice allocator.
    static cAgileEyeDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cAgileEyeDevice>(a_deviceNumber)); }


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

    //! This method communicates with the haptic device.
    virtual bool communicate();

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
#if defined(USE_AGILE_EYE)
#pragma pack(push, 1)
    struct MCU_INPUT {
        unsigned int message_idx;
        double u;
        double v;
        double w;
        bool calibrationFlag;

        char message[30];

        void parseMessage() {
            char time_buf[6] = "";
            char u_buf[6] = "";
            char v_buf[6] = "";
            char w_buf[6] = "";
            char cal_flag_char[1] = "";
            int u_int, v_int, w_int;

            // index
            for (int i = 0; i < 6; i++) {
                time_buf[i] = message[i];
            }

            if (time_buf == "/0") time_buf[5] = 0;

            message_idx = atoi(time_buf);

            // u
            for (int i = 0; i < 6; i++) {
                u_buf[i] = message[i + 7];
            }

            if (u_buf == "/0") u_buf[5] = 0;
            u_int = atoi(u_buf) - 50000;
            u = double(u_int) / 1000.0;

            // v
            for (int i = 0; i < 6; i++) {
                v_buf[i] = message[i + 14];
            }

            if (v_buf == "/0") v_buf[5] = 0;
            v_int = atoi(v_buf) - 50000;
            v = double(v_int) / 1000.0;

            // w
            for (int i = 0; i < 6; i++) {
                w_buf[i] = message[i + 21];
            }

            if (w_buf == "/0") w_buf[5] = 0;
            w_int = atoi(w_buf) - 50000;
            w = double(w_int) / 1000.0;

            // calFlag
            cal_flag_char[0] = message[27];

            if (cal_flag_char == "/0") cal_flag_char[0] = 0;
            calibrationFlag = bool(atoi(cal_flag_char));
        }
    };

    struct MCU_OUTPUT {
        unsigned int message_idx;
        double tau1;
        double tau2;
        double tau3;
        bool enableMotors;
        bool calibrationFlag;

        char message[25];

        void setMessage() {
            message_idx++;

            int m_idx_ = int(message_idx);
            int enable_motors_ = int(enableMotors);
            int tau1_ = int(tau1 * 1000.0) + 50000;
            int tau2_ = int(tau2 * 1000.0) + 50000;
            int tau3_ = int(tau3 * 1000.0) + 50000;
            int calibration_flag_ = int(calibrationFlag);
            sprintf_s(message, "%6.0d%1.0d%5.0d%5.0d%5.0d%1.0d", m_idx_, enable_motors_, tau1_, tau2_, tau3_, calibration_flag_);
        }
    };
#pragma pack(pop)

    CSerialComm serialComm;
	
    MCU_INPUT mcu_input;
    MCU_OUTPUT mcu_output;

#endif
};

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
