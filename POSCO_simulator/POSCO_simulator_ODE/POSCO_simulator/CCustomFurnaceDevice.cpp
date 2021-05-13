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
#include "definitions.h"
#include "CCustomFurnaceDevice.h"

using namespace std;
using namespace chai3d;
//------------------------------------------------------------------------------
namespace chai3d {
    //------------------------------------------------------------------------------

    //==============================================================================
    /*!
        Constructor of cMyCustomDevice.
    */
    //==============================================================================
    cCustomFurnaceDevice::cCustomFurnaceDevice(unsigned int a_deviceNumber)
    {        
        int count;
        m_falcon = nullptr;
        count = cDeltaDevice::getNumDevices();
        if (count > 0){
            m_falcon = cDeltaDevice::create(0);
            //m_falcon = new cDeltaDevice(0);
            if (m_falcon->open()){
                printf("[CustomFurnaceDevice] Found Falcon \n");
                m_falcon->close();
            }
            else {
                printf("[CustomFurnaceDevice] Cannot open Falcon \n");
            }
        }
        else{
            printf("[CustomFurnaceDevice] Cannot find Falcon \n");
        }

        count = cAgileEyeDevice::getNumDevices();
        if (count > 0) {
            m_agileEye = cAgileEyeDevice::create(0);
            //m_agileEye = new cAgileEyeDevice(0);
            if (m_agileEye->open()) {
                printf("[CustomFurnaceDevice] Found AgileEye \n");
                m_agileEye->close();
            }
            else {
                printf("[CustomFurnaceDevice] Cannot open AgileEye \n");
            }
        }
        else {
            printf("[CustomFurnaceDevice] Cannot find AgileEye \n");
        }

        m_specifications.m_model = C_HAPTIC_DEVICE_CUSTOM;
        m_specifications.m_manufacturerName = "MARCHLAB_POSTECH";
        m_specifications.m_modelName = "FURNACE DEVICE";

        m_specifications.m_maxLinearForce = m_falcon->m_specifications.m_maxLinearForce;		                    // [N]
        m_specifications.m_maxGripperForce = m_falcon->m_specifications.m_maxGripperForce;		                    // [N]
        m_specifications.m_maxLinearStiffness = m_falcon->m_specifications.m_maxLinearStiffness;		            // [N/m]
        m_specifications.m_workspaceRadius = m_falcon->m_specifications.m_workspaceRadius;		                    // [m]
        m_specifications.m_maxGripperLinearStiffness = m_falcon->m_specifications.m_maxGripperLinearStiffness;		// [N*m]
        m_specifications.m_gripperMaxAngleRad = m_falcon->m_specifications.m_gripperMaxAngleRad;
#ifdef USE_AGILE_EYE
        m_specifications.m_maxAngularTorque = m_agileEye->m_specifications.m_maxAngularTorque;		        // [N*m]
        m_specifications.m_maxAngularStiffness = m_agileEye->m_specifications.m_maxAngularStiffness;		    // [N*m/Rad]
        m_specifications.m_sensedRotation = m_agileEye->m_specifications.m_sensedRotation;
#endif
        m_specifications.m_maxLinearDamping = m_falcon->m_specifications.m_maxLinearDamping;		                // [N/(m/s)]
        m_specifications.m_maxGripperAngularDamping = 0.0;		// [N*m/(Rad/s)]
        m_specifications.m_maxAngularDamping = m_falcon->m_specifications.m_maxAngularDamping;		// [N*m/(Rad/s)]
        m_specifications.m_sensedPosition = m_falcon->m_specifications.m_sensedPosition;
        m_specifications.m_sensedGripper = m_falcon->m_specifications.m_sensedGripper;
        m_specifications.m_actuatedPosition = m_falcon->m_specifications.m_actuatedPosition;
        m_specifications.m_actuatedRotation = m_falcon->m_specifications.m_actuatedRotation;
        m_specifications.m_actuatedGripper = m_falcon->m_specifications.m_actuatedGripper;
        m_specifications.m_leftHand = true;
        m_specifications.m_rightHand = true;
    }


    //==============================================================================
    /*!
        Destructor of cMyCustomDevice.
    */
    //==============================================================================
    cCustomFurnaceDevice::~cCustomFurnaceDevice(){
        if (close()) {
            printf("[cCustomFurnaceDevice] Device is closed successfully \n");
        }
        else {
            printf("[cCustomFurnaceDevice] Failed to close device \n");
        }
        //delete m_falcon;

        m_falcon = cGenericHapticDevicePtr();
#ifndef USE_AGILE_EYE
        m_agileEye = cGenericHapticDevicePtr();
#endif
    }


    //==============================================================================
    /*!
        This method opens a connection to your device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cCustomFurnaceDevice::open(){
#ifdef USE_AGILE_EYE
        if (m_falcon->open() && m_agileEye->open()) {
            return C_SUCCESS;
        }
        else {
            m_falcon->close();
            m_agileEye->close();
            return C_ERROR;
        }
#else
        if (m_falcon->open()) {
            return C_SUCCESS;
        }
        else {
            m_falcon->close();
            return C_ERROR;
        }
#endif
    }


    //==============================================================================
    /*!
        This method closes the connection to your device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cCustomFurnaceDevice::close(){
#ifdef USE_AGILE_EYE
        if (m_falcon->close() && m_agileEye->close()) {
            return C_SUCCESS;
        }
        else {
            return C_ERROR;
        }
#else
        if (m_falcon->close()) {
            return C_SUCCESS;
        }
        else {
            return C_ERROR;
        }
#endif
    }


    //==============================================================================
    /*!
        This method calibrates your device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cCustomFurnaceDevice::calibrate(bool a_forceCalibration){
#ifdef USE_AGILE_EYE
        if (!m_falcon->isDeviceReady() || !m_agileEye->isDeviceReady()) return (C_ERROR);
        return m_falcon->calibrate(a_forceCalibration) && m_agileEye->calibrate(a_forceCalibration);
#else
        if (!m_falcon->isDeviceReady()) return (C_ERROR);
        return m_falcon->calibrate(a_forceCalibration);
#endif
    }


    //==============================================================================
    /*!
        This method returns the number of devices available from this class of device.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    unsigned int cCustomFurnaceDevice::getNumDevices() {
        if (cDeltaDevice::getNumDevices() > 0 
#ifdef USE_AGILE_EYE
            && cAgileEyeDevice::getNumDevices() > 0
#endif
            ) {
            printf("[CustomFurnaceDevice] Found device \n");
            return 1;
        }
        else {
            printf("[CustomFurnaceDevice] Cannot find device \n");
            return 0;
        }
    }


    //==============================================================================
    /*!
        This method returns the position of your device. Units are meters [m].

        \param   a_position  Return value.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cCustomFurnaceDevice::getPosition(cVector3d& a_position){
        return m_falcon->getPosition(a_position);
    }


    //==============================================================================
    /*!
        This method returns the orientation frame of your device end-effector

        \param   a_rotation  Return value.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cCustomFurnaceDevice::getRotation(cMatrix3d& a_rotation) {
#ifdef USE_AGILE_EYE
        return m_agileEye->getRotation(a_rotation);
#else
        return true;
#endif
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
    bool cCustomFurnaceDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
        const cVector3d& a_torque,
        const double a_gripperForce){
#ifdef USE_AGILE_EYE
        return m_falcon->setForceAndTorqueAndGripperForce(a_force, a_torque, a_gripperForce)
            && m_agileEye->setForceAndTorqueAndGripperForce(a_force, a_torque, a_gripperForce);
#else
        return m_falcon->setForceAndTorqueAndGripperForce(a_force, a_torque, a_gripperForce);
#endif
    }


    //==============================================================================
    /*!
        This method returns status of all user switches
        [__true__ = __ON__ / __false__ = __OFF__].

        \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

        \return __true__ if the operation succeeds, __false__ otherwise.
    */
    //==============================================================================
    bool cCustomFurnaceDevice::getUserSwitches(unsigned int& a_userSwitches){
        return m_falcon->getUserSwitches(a_userSwitches);
    }


    //------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
