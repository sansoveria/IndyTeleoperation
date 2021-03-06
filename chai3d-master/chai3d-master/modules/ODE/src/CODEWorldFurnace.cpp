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
#include "CODEWorldFurnace.h"
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
//! Maximum number of contact points per body.
#define MAX_CONTACTS_PER_BODY 16
//---------------------------------------------------------------------------


//==========================================================================
/*!
    Constructor of cODEWorld.

    \param  a_parentWorld  Pointer to parent CHAI3D world.
*/
//===========================================================================
cODEWorldFurnace::cODEWorldFurnace(cWorld* a_parentWorld): cODEWorld(a_parentWorld)
{
    // init ODE
    dInitODE();

    // set parent world
    m_parentWorld = a_parentWorld;

    // reset simulation time.
    m_simulationTime = 0.0;

    // create ODE world
    m_ode_world = dWorldCreate();

    // create ODE space
    m_ode_space = dHashSpaceCreate(0);

    // create ODE contact group
    m_ode_contactgroup = dJointGroupCreate(0);

    // setup callback to handle collision
    //dSpaceCollide(m_ode_space, this, &(cODEWorld::nearCallback));

    // set some default damping parameters
    dWorldSetLinearDamping(m_ode_world, 0.00001);
    dWorldSetAngularDamping(m_ode_world, 0.0015);
    dWorldSetMaxAngularSpeed(m_ode_world, 200);
}

//------------------------------------------------------------------------------
// DLee CUSTOM:
void cODEWorldFurnace::initWorld(dBodyID toolID) {

    m_num_rules = 0;

	m_ode_tool = toolID;
	toolCollisionIdx = 0;

    // setup callback to handle collision
    dSpaceCollide(m_ode_space, this, &(cODEWorldFurnace::nearCallback));
    cout << "collision callback set" << endl;
}
//------------------------------------------------------------------------------


//===========================================================================
/*!
    Destructor of cODEWorldFurnace.
*/
//===========================================================================
cODEWorldFurnace::~cODEWorldFurnace()
{
}


//===========================================================================
/*!
	This methods updates the simulation over a time interval passed as
	argument

	\param  a_interval  Time increment.
*/
//===========================================================================
void cODEWorldFurnace::updateDynamics(double a_interval)
{
	toolCollisionIdx = 0;

	// sanity check
	if (a_interval <= 0) { return; }

	// update collision callback information
	dSpaceCollide(m_ode_space, this, &(cODEWorldFurnace::nearCallback));

	// integrate simulation during an certain interval
	dWorldStep(m_ode_world, a_interval);
	// dWorldStepFast1 (m_ode_world, a_interval, 5);
	//dWorldQuickStep(m_ode_world, a_interval);

	// cleanup contacts from previous iteration
	dJointGroupEmpty(m_ode_contactgroup);

	// add time to overall simulation
	m_simulationTime = m_simulationTime + a_interval;

	// update CHAI3D positions for of all object
	updateBodyPositions();
}

//===========================================================================
/*!
    This methods is an ODE callback for handling collision detection.

    \param  a_data     Not used here.
    \param  a_object1  Reference to ODE object 1.
    \param  a_object2  Reference to ODE object 2.
*/
//===========================================================================
void cODEWorldFurnace::nearCallback (void *a_data, dGeomID a_object1, dGeomID a_object2)
{
    // retrieve body ID for each object. This value is defined unless the object
    // is static.
    dBodyID b1 = dGeomGetBody(a_object1);
    dBodyID b2 = dGeomGetBody(a_object2);

    cODEWorldFurnace* world = (cODEWorldFurnace*)a_data;

    // exit without doing anything if the two bodies are connected by a joint
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
	if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeFixed)) return;

    dContact contact[MAX_CONTACTS_PER_BODY];
    int n = dCollide (a_object1, a_object2, MAX_CONTACTS_PER_BODY,&(contact[0].geom),sizeof(dContact));
    if (n > 0) 
    {
		bool TOOL_CONTACT = false;
		if (b1 == world->m_ode_tool) {
			world->toolCollisionList[world->toolCollisionIdx] = a_object2;
			world->toolCollisionIdx++;
			TOOL_CONTACT = true;
		}
		if (b2 == world->m_ode_tool) {
			world->toolCollisionList[world->toolCollisionIdx] = a_object1;
			world->toolCollisionIdx++;
			TOOL_CONTACT = true;
		}

        for (int i=0; i<n; i++) 
        {
            for (int j = 0; j < world->m_num_rules; j++) {
                world->m_custom_rule[i].check(contact[i].geom.g1, contact[i].geom.g2);
            }

            // define default collision properties (this section could be extended to support some ODE material class!)
            
			if (TOOL_CONTACT) {
				contact[i].surface.mode = 0;
				contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
				contact[i].surface.soft_erp = 0.90;
				contact[i].surface.soft_cfm = 0.10;
			}
			else {
				contact[i].surface.mode = 0;
				contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
				contact[i].surface.slip1 = 0.001;
				contact[i].surface.slip2 = 0.001;
				contact[i].surface.soft_erp = 0.10;
				contact[i].surface.soft_cfm = 0.50;
			}
            contact[i].surface.mu = 0.5;


            // get handles on each body
            cODEGenericBody* ode_body = NULL;
            if (b1 != NULL)
            {
                ode_body = (cODEGenericBody*)dBodyGetData (b1);
            }
            else if (b2 != NULL)
            {
                // if first object is static, use the second one. (both objects can not be static) 
                ode_body = (cODEGenericBody*)dBodyGetData (b2);
            }
                
            // create a joint following collision
            if (ode_body != NULL)
            {
                dJointID c = dJointCreateContact (ode_body->m_ODEWorld->m_ode_world,
                                                    ode_body->m_ODEWorld->m_ode_contactgroup,
                                                    &contact[i]);
                dJointAttach (c,
                            dGeomGetBody(contact[i].geom.g1),
                            dGeomGetBody(contact[i].geom.g2));
            }
        }
    }
}


//------------------------------------------------------------------------------
// DLee CUSTOM:
void cODEWorldFurnace::setCollisionRule(dGeomID obj1, dGeomID obj2) {
    m_custom_rule[m_num_rules] = collisionRule();
    m_custom_rule[m_num_rules].init(obj1, obj2);
    m_num_rules++;
}
//------------------------------------------------------------------------------
