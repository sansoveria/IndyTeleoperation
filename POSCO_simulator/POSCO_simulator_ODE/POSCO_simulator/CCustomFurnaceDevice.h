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
#ifndef CCustomFurnaceDeviceH
#define CCustomFurnaceDeviceH

#include "devices/CDeltaDevices.h"
#include "CAgileEyeTwinCATDevice.h"

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
    class cCustomFurnaceDevice;
    typedef std::shared_ptr<cCustomFurnaceDevice> cCustomFurnaceDevicePtr;
    //------------------------------------------------------------------------------

    //==============================================================================
    /*!
        \class      cCustomFurnaceDevice
        \ingroup    devices

        \brief
        This class is a interface to support custom haptic devices for deposited
        metal removal task.

        \details
        The custom device consists of two device: Novint Falcon and custom agile
        eye. Novint Falcon uses cDeltaDevice class (which is for the devices of
        Force Dimension). The custom agile eye device works with TwinCAT, so

    */
    //==============================================================================
    class cCustomFurnaceDevice
    {
        //--------------------------------------------------------------------------
        // CONSTRUCTOR & DESTRUCTOR:
        //--------------------------------------------------------------------------

    public:

        //! Constructor of cMyCustomDevice.
        cCustomFurnaceDevice(unsigned int a_deviceNumber = 0);

        //! Destructor of cMyCustomDevice.
        ~cCustomFurnaceDevice();

        //! Shared cMyCustomDevice allocator.
        static cCustomFurnaceDevicePtr create(unsigned int a_deviceNumber = 0) { 
            return (std::make_shared<cCustomFurnaceDevice>(a_deviceNumber)); 
        }


        //--------------------------------------------------------------------------
        // PUBLIC METHODS:
        //--------------------------------------------------------------------------

    public:
                
        bool open();
        bool close();
        bool calibrate(bool a_forceCalibration = false);
        bool getPosition(cVector3d& a_position);
        bool getRotation(cMatrix3d& a_rotation);
        bool getUserSwitches(unsigned int& a_userSwitches);                
        bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);


        //--------------------------------------------------------------------------
        // PUBLIC STATIC METHODS:
        //--------------------------------------------------------------------------

    public:

        //! This method returns the number of devices available from this class of device.
        static unsigned int getNumDevices();


        //--------------------------------------------------------------------------
        // PUBLIC MEMBER:
        //--------------------------------------------------------------------------

    public:
        cHapticDeviceInfo m_specifications;

        //--------------------------------------------------------------------------
        // PROTECTED MEMBERS:
        //--------------------------------------------------------------------------

    protected:
        cGenericHapticDevicePtr m_falcon;
        cGenericHapticDevicePtr m_agileEye;
        //cDeltaDevice* m_falcon;
        //cAgileEyeDevice* m_agileEye;
    };

//------------------------------------------------------------------------------
// namespace chai3d
//------------------------------------------------------------------------------
}

#endif  // CCustomFurnaceDeviceH
//------------------------------------------------------------------------------