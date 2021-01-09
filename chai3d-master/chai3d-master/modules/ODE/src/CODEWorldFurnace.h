//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
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
    \author    Francois Conti
    \version   3.2.0 $Rev: 1869 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CODEWorldFurnaceH
#define CODEWorldFurnaceH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CODEWorld.h"
#include "CODEGenericBody.h"
//---------------------------------------------------------------------------
#define dSINGLE
//===========================================================================
/*!
    \file       CODEWorldFurnace.h

    \brief
    Implementation of an ODE world for furnace.
*/
//===========================================================================


//===========================================================================
/*!
    \class      cODEWorldFurnace
    \ingroup    ODE

    \brief
    This class implements an ODE dynamic world.

    \details
    cODEWorldFurnace implements a virtual world to handle ODE based objects 
    (cODEGenericBody).
*/
//===========================================================================
class cODEWorldFurnace : public cODEWorld
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cODEWorld.
    cODEWorldFurnace(chai3d::cWorld* a_parentWorld);

    //! Destructor of cODEWorld.
    virtual ~cODEWorldFurnace();


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:
	// target body to investigate collision
	dBodyID m_ode_tool;
	dGeomID toolCollisionList[5];
	int toolCollisionIdx;

    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:
	//! This method updates the simulation over a time interval.
    void initWorld(dBodyID toolID);
	void updateDynamics(double a_interval);
    void setCollisionRule(dGeomID obj1, dGeomID obj2);

    struct collisionRule {
        dGeomID obj1;
        dGeomID obj2;

        bool collided = false;

        void init(dGeomID o1, dGeomID o2) {
            obj1 = o1;
            obj2 = o2;
            collided = false;
        }

        void reset() {
            collided = false;
        }

        bool check(dGeomID o1, dGeomID o2) {
            if (((o1 == obj1) && (o2 == obj2)) || ((o2 == obj1) && (o1 == obj2))) {
                collided = true;
                return true;
            }
            else {
                return false;
            }
        }
    };

    collisionRule m_custom_rule[100];
    int m_num_rules;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //-----------------------------------------------------------------------


    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //-----------------------------------------------------------------------
    
protected:

    //! This method is an ODE callback that reports collisions.
    static void nearCallback (void *data, dGeomID o1, dGeomID o2);
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
