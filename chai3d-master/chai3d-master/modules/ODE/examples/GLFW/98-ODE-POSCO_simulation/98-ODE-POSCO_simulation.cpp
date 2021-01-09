#include "98-ODE-POSCO_simulation.h"

//===========================================================================
/*
    DEMO:    98-ODE-POSCO_simulation.cpp

    This example integrates ODE and chai3D to build simulation 
	for metal deposit task in front of furnace
	*/
 //===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 04-ODE-tool" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[h] - Display help menu" << endl;
    cout << "[1] - Enable gravity" << endl;
    cout << "[2] - Disable gravity" << endl << endl;
    cout << "[3] - decrease linear haptic gain" << endl;
    cout << "[4] - increase linear haptic gain" << endl;
    cout << "[5] - decrease angular haptic gain" << endl;
    cout << "[6] - increase angular haptic gain" << endl << endl;
    cout << "[7] - decrease linear stiffness" << endl;
    cout << "[8] - increase linear stiffness" << endl;
    cout << "[9] - decrease angular stiffness" << endl;
    cout << "[0] - increase angular stiffness" << endl << endl;
#ifndef SIMULATION
    cout << "[m] - toggle teleoperation mode" << endl;
    cout << "[a] - test1" << endl;
    cout << "[b] - test2" << endl;
#else
    cout << "[m] - toggle virtical mirror" << endl;
#endif
    cout << "[f] - toggle full screen\n" << endl;
    cout << "[q] - Exit application\n" << endl;
    cout << endl << endl;


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    glfwGetWindowSize(window, &width, &height);
    glfwSetWindowPos(window, x, y);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);
    glfwSwapInterval(swapInterval);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------
    // create a new world.
    world = new cWorld();
    world->m_backgroundColor.setWhite();

    camera = new cCamera(world);
    world->addChild(camera);
    camera->setClippingPlanes(0.01, 10.0);
    camera->setStereoMode(stereoMode);
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(2.0);
    camera->setMirrorVertical(mirroredDisplay);

	light = new cSpotLight(world);
	world->addChild(light);
    light->setEnabled(true);
    light->setLocalPos(1.0, -1.0, 3.0);
    light->setDir(0, 0, -1.0);

    // set uniform concentration level of light 
    light->setSpotExponent(10.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    //light->m_shadowMap->setQualityLow();
    light->m_shadowMap->setQualityHigh();

    // set light cone half angle
    light->setCutOffAngleDeg(45);

    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

	devicePosDerivative.setTimeStep(TIMESTEP);
	devicePosDerivative.setCutOffFreq(50.0);

	toolPosDerivative.setTimeStep(TIMESTEP);
	toolPosDerivative.setCutOffFreq(50.0);
	toolRotDerivative.setTimeStep(TIMESTEP);
	toolRotDerivative.setCutOffFreq(50.0);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    panel = new cPanel();
    camera->m_frontLayer->addChild(panel);
    panel->setColor(cColorf(0.0, 0.0, 0.0));
    panel->setTransparencyLevel(0.8);
    cout << width << ", " << height << endl;

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelRates);

    labelStatus = new cLabel(font);
    labelStatus->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelStatus);

    labelSuccess = new cLabel(font);
    labelSuccess->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelSuccess);

    labelFailure = new cLabel(font);
    labelFailure->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelFailure);

    labelForce = new cLabel(font);
    labelForce->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelForce);

    labelTorque = new cLabel(font);
    labelTorque->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelTorque);

    levelForce = new cLevel();
    camera->m_frontLayer->addChild(levelForce);
    levelForce->setRange(0.0, 500.0);
    levelForce->setSingleIncrementDisplay(true);
    levelForce->rotateWidgetDeg(270);
    levelForce->m_colorActive.setBlueCornflower();
    levelForce->m_colorInactive = levelForce->m_colorActive;
    levelForce->m_colorInactive.mul(0.3f);

    levelTorque = new cLevel();
    camera->m_frontLayer->addChild(levelTorque);
    levelTorque->setRange(0.0, 500.0);
    levelTorque->setSingleIncrementDisplay(true);
    levelTorque->rotateWidgetDeg(270);
    levelTorque->m_colorActive.setRedLightCoral();
    levelTorque->m_colorInactive = levelTorque->m_colorActive;
    levelTorque->m_colorInactive.mul(0.3f);


    //////////////////////////////////////////////////////////////////////////
    // ODE WORLD
    //////////////////////////////////////////////////////////////////////////

    // stiffness properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
	cout << "maxStiffness: " << maxStiffness << endl;

    // clamp the force output gain to the max device stiffness
    linGain = cMin(linGain, maxStiffness / linStiffness);

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorldFurnace(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

#ifdef SIMULATION
    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));

    // define damping properties
    ODEWorld->setAngularDamping(0.0002);
    ODEWorld->setLinearDamping(0.0002);

    //////////////////////////////////////////////////////////////////////////
    // Define Dynamic Objects
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    // Stones
    //////////////////////////////////////////////////////////////////////////
    cMultiMesh* stone[10];    

    string file_name;
    file_name = "../../../bin/resources/models/POSCO/octagon3.stl";
    //file_name[1] = "../../../bin/resources/models/POSCO/octagon3.stl";
    //file_name[2] = "../../../bin/resources/models/POSCO/octagon3.stl";

    //stone_init_pos[0] = cVector3d(-0.9, -1.7, -0.6);
	srand(time(NULL));
	double stone_x = -0.3 - 0.4*(double)(rand() % 100) / 100.0;
	double stone_y = -0.8 - 0.9*(double)(rand() % 100) / 100.0;
	double stone_z = -0.6;
    stone_init_pos = cVector3d(stone_x, stone_y, stone_z);
	std::cout << rand() << " " << rand() << " " << rand() << " " << stone_x << ", " << stone_y << ", " << stone_z << std::endl;
    //stone_init_pos[2] = cVector3d(-0.3, -0.8, -0.6);

    stone_init_rot = cMatrix3d(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
    //stone_init_rot[1] = cMatrix3d(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
    //stone_init_rot[2] = cMatrix3d(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);

    for (int i = 0; i < NUM_STONE; i++) {
        ODEStone[i] = new cODEGenericBody(ODEWorld);
        stone[i] = new cMultiMesh();
        //fileload = stone[i]->loadFromFile(RESOURCE_PATH("../resources/models/stone/newstone/octagon2.stl"));

#if defined(_MSVC)
        bool fileload = stone[i]->loadFromFile(file_name);
#endif
        if (!fileload) {
            cout << "Cannot find " << file_name << endl;
        }

        stone[i]->setShowFrame(true);
		double stone_scale = 0.0010 + 0.0015*double(rand() % 100) / 100.0;
        stone[i]->scale(stone_scale);
       
        cMaterial mat_stone;
        mat_stone.setBlack();
        stone[i]->setMaterial(mat_stone);

        ODEStone[i]->setImageModel(stone[i]);
        ODEStone[i]->createDynamicMesh();
        ODEStone[i]->setMass(1.0);
        ODEStone[i]->setLocalPos(stone_init_pos);
        ODEStone[i]->setLocalRot(stone_init_rot);
        //dBodySetAngularDamping(ODEStone[i]->m_ode_body, 0.001);
        //dBodySetLinearDamping(ODEStone[i]->m_ode_body, 0.001);
    }

    //////////////////////////////////////////////////////////////////////////
    // Lava
    //////////////////////////////////////////////////////////////////////////
    ODELava = new cODEGenericBody(ODEWorld);
    cMultiMesh* lava = new cMultiMesh();

    bool fileload_lava;
    fileload_lava = lava->loadFromFile(RESOURCE_PATH("../resources/models/POSCO/lava.stl"));
    if (!fileload_lava)
    {
#if defined(_MSVC)
        fileload_lava = lava->loadFromFile("../../../bin/resources/models/POSCO/lava.stl");
#endif
    }
    cout << fileload_lava << endl;

    lava->scale(0.04);
    cMaterial mat_lava;
    mat_lava.setRedCrimson();
    lava->setMaterial(mat_lava);

    ODELava->setImageModel(lava);
    ODELava->createDynamicMesh(true); 
    ODELava->setLocalPos(-0.0, -1.99, -0.97);
    cMatrix3d rot_offset1(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
    cMatrix3d rot_offset2(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);

    ODELava->setLocalRot(cMul(rot_offset2, rot_offset1));

#ifdef FENCE
    //////////////////////////////////////////////////////////////////////////
    // Fence
    //////////////////////////////////////////////////////////////////////////
    ODEFence = new cODEGenericBody(ODEWorld);
    ODEFence_side1 = new cODEGenericBody(ODEWorld);
    ODEFence_side2 = new cODEGenericBody(ODEWorld);
    
    cCreateBox(fence_mesh, cube_size / 8.0, cube_size * 1.5, cube_size * 1.0);
    cCreateBox(fence_side1_mesh, cube_size / 8.0, cube_size / 8.0, cube_size * 1.0);
    cCreateBox(fence_side2_mesh, cube_size / 8.0, cube_size / 8.0, cube_size * 1.0);
    // NOTE: ODE does not support cylinder - cylinder collision
    //cCreateCylinder(fence_side1_mesh, cube_size, cube_size / 4.0, 32U, 1U, 1U, true, true);
    //cCreateCylinder(fence_side2_mesh, cube_size, cube_size / 4.0, 32U, 1U, 1U, true, true);

    cMaterial mat_Fence;

    mat_Fence.setGreen();
    fence_mesh->setMaterial(mat_Fence);
    fence_side1_mesh->setMaterial(mat_Fence);
    fence_side2_mesh->setMaterial(mat_Fence);

    ODEFence->setImageModel(fence_mesh);
    ODEFence_side1->setImageModel(fence_side1_mesh);
    ODEFence_side2->setImageModel(fence_side2_mesh);

    ODEFence->createDynamicBox(cube_size / 8.0, cube_size * 1.5, cube_size * 0.99, true);
    ODEFence_side1->createDynamicBox(cube_size / 8.0, cube_size / 8.0, cube_size*0.99, true);
    ODEFence_side2->createDynamicBox(cube_size / 8.0, cube_size / 8.0, cube_size*0.99, true);
    // NOTE: ODE does not support cylinder - cylinder collision
    //fence_side1->createDynamicCylinder(cube_size / 4.0, cube_size, true);
    //fence_side2->createDynamicCylinder(cube_size / 4.0, cube_size, true);

    ODEFence->setLocalPos(0.8, -1.0, -0.4);
    ODEFence_side1->setLocalPos(0.8, -1.275, -0.1);
    ODEFence_side2->setLocalPos(0.8, -0.725, -0.1);
#endif
    //////////////////////////////////////////////////////////////////////////
    // Lane
    //////////////////////////////////////////////////////////////////////////
    ODELane1 = new cODEGenericBody(ODEWorld);
    ODELane2 = new cODEGenericBody(ODEWorld);

    cCreateBox(lane1_mesh, 2.0, 1.5, 0.20);
    cCreateBox(lane2_mesh, 2.0, 3.5, 0.02);

    cMaterial mat_lane1, mat_lane2;
    mat_lane1.setBlueLightSteel();
    lane1_mesh->setMaterial(mat_lane1);
    mat_lane2.setRedCrimson();
    lane2_mesh->setMaterial(mat_lane2);
    
    ODELane1->setImageModel(lane1_mesh);
    ODELane2->setImageModel(lane2_mesh);

    ODELane1->createDynamicBox(1.99, 1.49, 0.19, true);
    ODELane2->createDynamicBox(1.99, 3.49, 0.01, true);
    
    ODELane1->setLocalPos(-0.5, -1.25, -0.9);
    ODELane2->setLocalPos(-0.5, 1.25, -0.999);
#endif
    //////////////////////////////////////////////////////////////////////////
    // CURSOR
    //////////////////////////////////////////////////////////////////////////
    //cCreateSphere(cursor_handle, cube_size / 4.0);
    //world->addChild(cursor_handle);

	cCreateSphere(cursor_endtip, cube_size / 4.0);
	world->addChild(cursor_endtip);

    cMaterial mat_handle;
    mat_handle.setYellowLight();
	mat_handle.setShininess(8);
	//cursor_handle->setMaterial(mat_handle);	
	cursor_endtip->setMaterial(mat_handle);
	cursor_endtip->setTransparencyLevel(0.3);	
	cursor_endtip->setShowFrame(true);

#ifdef SIMULATION
    //////////////////////////////////////////////////////////////////////////
    // STARTING POINT
    //////////////////////////////////////////////////////////////////////////
    starting_pos = cVector3d(2.0, -1.75, -0.59);

    cCreateDisk(starting_pad, cube_size, cube_size);
    world->addChild(starting_pad);

   	matBlue.setBlueMidnight();
	matBlue.setShininess(64);
	matRed.setRedFireBrick();
	matRed.setShininess(64);

    starting_pad->setMaterial(matBlue);
    starting_pad->setLocalPos(starting_pos);
	starting_pad->setTransparencyLevel(0.7);
    //////////////////////////////////////////////////////////////////////////
    // TOOL (ROD)
    //////////////////////////////////////////////////////////////////////////
    // create a virtual tool (rod)
    ODETool = new cODEGenericBody(ODEWorld);
    tool_init_pos = cVector3d(1.0, -1.75, -0.5);
    tool_init_rot = cMatrix3d(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0);

    //cCreateBox(tool_mesh, cube_size * 6.0, cube_size / 5.0, cube_size / 5.0);
    cCreateCylinder(tool_mesh, cube_size * 6.0, cube_size / 20.0, 32U, 1U, 1U, true, true, cVector3d(0.0, 0.0, cube_size * -3.0));
    //tool_mesh->setShowFrame(true);

    // define some material properties
    cMaterial matTool;
    matTool.m_ambient.set(1.0f, 1.0f, 1.0f);
    matTool.m_diffuse.set(1.0f, 1.0f, 1.0f);
    matTool.m_specular.set(1.0f, 1.0f, 1.0f);
    tool_mesh->setMaterial(matTool);

    // add mesh to ODE object
    ODETool->setImageModel(tool_mesh);
    //ODETool->createDynamicBox(cube_size * 6.0, cube_size / 5.0, cube_size / 5.0, false);
    ODETool->createDynamicCylinder(cube_size / 20.0, cube_size * 6.0, false);

    // define some mass properties
    ODETool->setMass(0.01);
    ODETool->setLocalPos(tool_init_pos);
    ODETool->setLocalRot(tool_init_rot);
    //dBodySetAngularDamping(ODETool->m_ode_body, 0.05);
    //dBodySetLinearDamping(ODETool->m_ode_body, 0.05);
	//dBodySetAngularDamping(ODETool->m_ode_body, 0.04);
	//dBodySetLinearDamping(ODETool->m_ode_body, 0.0001);

	//////////////////////////////////////////////////////////////////////////
	// FT Sensor
	//////////////////////////////////////////////////////////////////////////
	ODEToolHandle = new cODEGenericBody(ODEWorld);
	cCreateBox(tool_handle_mesh, cube_size / 8.0, cube_size / 8.0, cube_size / 8.0);
	tool_handle_mesh->setShowFrame(true);
	
	// define some material properties
	tool_handle_mesh->setMaterial(matBlue);

	// add mesh to ODE object
	ODEToolHandle->setImageModel(tool_handle_mesh);
	ODEToolHandle->createDynamicBox(cube_size / 8.0, cube_size / 8.0, cube_size / 8.0, false);

	// define some mass properties for cube
	ODEToolHandle->setMass(1.0);
	ODEToolHandle->setLocalPos(tool_init_pos + cVector3d(cube_size*3.0626, 0.0, 0.0));
	ODEToolHandle->setLocalRot(tool_init_rot);

	dBodySetAngularDamping(ODEToolHandle->m_ode_body, 0.0001);
	dBodySetLinearDamping(ODEToolHandle->m_ode_body, 0.0001);

	// define fixed joint (FT sensor)	
	sensorGroup = dJointGroupCreate(0);
	FTSensor = dJointCreateFixed(ODEWorld->m_ode_world, sensorGroup);
	dJointAttach(FTSensor, ODETool->m_ode_body, ODEToolHandle->m_ode_body);
	dJointSetFixed(FTSensor);
	dJointSetFeedback(FTSensor, &FTSensorMeasure);
	LPFSensor.setTimeStep(TIMESTEP);
	LPFSensor.setCutOffFreq(50.0);
	LPFSensor.reset();

    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////
    ODEGround = new cODEGenericBody(ODEWorld);
    ODEGround->createStaticPlane(cVector3d(0.0, 0.0, -1.0), cVector3d(0.0, 0.0, 1.0));

    //////////////////////////////////////////////////////////////////////////
    // SIDEWALKS
    //////////////////////////////////////////////////////////////////////////
    ODESidewalk1 = new cODEGenericBody(ODEWorld);
    ODESidewalk2 = new cODEGenericBody(ODEWorld);

    cCreateBox(sidewalk1_mesh, 2.0, 8.0, 0.40);
    cCreateBox(sidewalk2_mesh, 1.0, 8.0, 0.40);

    cMaterial mat_sidewalk;
    mat_sidewalk.setGrayLight();
    sidewalk1_mesh->setMaterial(mat_sidewalk);
    sidewalk2_mesh->setMaterial(mat_sidewalk);

    ODESidewalk1->setImageModel(sidewalk1_mesh);
    ODESidewalk2->setImageModel(sidewalk2_mesh);

    ODESidewalk1->createDynamicBox(1.99, 7.99, 0.39, true);
    ODESidewalk2->createDynamicBox(0.99, 7.99, 0.39, true);

    ODESidewalk1->setLocalPos(1.5, -1.0, -0.8);
    ODESidewalk2->setLocalPos(-2.0, -1.0, -0.8);

    //////////////////////////////////////////////////////////////////////////
    // Furnace wall
    //////////////////////////////////////////////////////////////////////////
    ODEFurnace = new cODEGenericBody(ODEWorld);    
    cCreateBox(furnace_mesh, 2.0, 3.0, 5.0);    

    cMaterial mat_furnace;
    mat_furnace.setGrayDark();
    furnace_mesh->setMaterial(mat_furnace);
    ODEFurnace->setImageModel(furnace_mesh);
    ODEFurnace->createDynamicBox(1.99, 2.99, 4.99, true);
    ODEFurnace->setLocalPos(-0.5, -3.5, 1.5);
    //////////////////////////////////////////////////////////////////////////
    // Initialize world and efine collision rules
    //////////////////////////////////////////////////////////////////////////
    ODEWorld->initWorld(ODETool->m_ode_body);
    MISSION_FAILED = false;
    MISSION_SUCCEEDED = false;
    MISSION_STARTED = false;

    num_stone_remain = NUM_STONE;
    num_failure = 0;
    num_success = 0;

    // rule[0] tool - lava (fail)
    // rule[1] tool - lane2 (fail)
    // rule[2] stone[0] -lane2 (success)
    // rule[3] stone[1] -lane2 (success)
    // rule[4] stone[2] -lane2 (success)
    // rule[5] stone[3] -lane2 (success)
    // rule[6] stone[4] -lane2 (success)
    // rule[7] stone[5] -lane2 (success)
    // rule[8] stone[6] -lane2 (success)
    // rule[9] stone[7] -lane2 (success)
    // rule[10] stone[8] -lane2 (success)
    // rule[11] stone[9] -lane2 (success)

    ODEWorld->setCollisionRule(ODETool->m_ode_geom, ODELava->m_ode_geom);
    ODEWorld->setCollisionRule(ODETool->m_ode_geom, ODELane2->m_ode_geom);
    for (int i = 0; i < NUM_STONE; i++) {
        //ODEWorld->setCollisionRule(ODEStone[i]->m_ode_geom, ODELava->m_ode_geom);
        ODEWorld->setCollisionRule(ODEStone[i]->m_ode_geom, ODELane2->m_ode_geom);
    }

    cout << ODEWorld->m_num_rules << endl;
#endif

#ifndef SIMULATION
    //-----------------------------------------------------------------------
    // START INDY
    //-----------------------------------------------------------------------
    indyTCP.connect();
#endif

    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	utilThread = new cThread();
	utilThread->start(updateUtils, CTHREAD_PRIORITY_GRAPHICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE CAMERA VIEW
    /////////////////////////////////////////////////////////////////////

    if (MISSION_STARTED) {
        // calculate tool endtip position
        cTransform cylinder_T_world = ODETool->getLocalTransform();
        cVector3d Eye_Point = cVector3d(0.0, 0.0, cube_size * 3.0);
        cVector3d Target_Point = cVector3d(0.0, 0.0, cube_size * -3.0);
        cVector3d Eye_Point_Global = cylinder_T_world * Eye_Point + cVector3d(2.0, 0.0, 2.0);
        cVector3d Target_Point_Global = cylinder_T_world * Target_Point;

        // position and orient the camera
        camera->set(Eye_Point_Global,    // camera position (eye)
            Target_Point_Global,    // lookat position (target)
            cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

		starting_pad->setMaterial(matRed);
    }
    else {
        // position and orient the camera
        camera->set(cVector3d(5.0, -1.0, 2.0),    // camera position (eye)
            cVector3d(0.0, -1.0, -0.5),    // lookat position (target)
            cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector
		starting_pad->setMaterial(matBlue);
    }

    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////
    // Background panel
    panel->setCornerRadius(height / 60, height / 60, height / 60, height / 60);
    panel->setSize(width * 0.9, height * 0.2);
    panel->setLocalPos(width * 0.05, height * 0.85);

    // Frequency
    labelRates->setText("Graphic freq./Haptic freq. \r\n" + cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " + cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
    labelRates->setLocalPos((int)(0.5*(width - labelRates->getWidth())), height*0.05);
    labelRates->setFontScale(1.0/500.0*height);

    // Status
    if (MISSION_STARTED) {
        labelStatus->setText("Remove black stones\r\n(remaining stones: " + cStr(num_stone_remain) + ")");
        labelStatus->setLocalPos((width - labelStatus->getWidth()) - height / 60.0 * 20.0, height*0.93);
    }
    else {
        labelStatus->setText("Move cursor to the blue circle");
        labelStatus->setLocalPos((width - labelStatus->getWidth()) - height / 60.0 * 20.0, height*0.93);
    }
    labelStatus->setFontScale(1.0 / 500.0 * height);

    // Result
    labelSuccess->setText("Success: " + cStr(num_success));
    labelSuccess->setLocalPos((width - labelSuccess->getWidth()) - height / 60.0 * 7.0, height*0.95);
    labelFailure->setText("Failure: " + cStr(num_failure));
    labelFailure->setLocalPos((width - labelFailure->getWidth()) - height / 60.0 * 7.0, height*0.90);
    labelSuccess->setFontScale(1.0 / 500.0 * height);
    labelFailure->setFontScale(1.0 / 500.0 * height);

    // Level (force, torque)
    levelForce->setLocalPos(height / 60.0 * 20.0, height*0.97);
    levelForce->setWidth(height * 0.03);
    levelTorque->setLocalPos(height / 60.0 * 20.0, height*0.92);
    levelTorque->setWidth(height * 0.03);
    labelForce->setLocalPos(height / 60.0 * 7.0, height*0.95);
    labelTorque->setLocalPos(height / 60.0 * 7.0, height*0.90);
    if (MISSION_STARTED) {
        levelForce->setValue(robot_force.length());        
        levelTorque->setValue(robot_torque.length());        
        labelForce->setText("Force (" + cStr(robot_force.length(),2) + ")");
        labelTorque->setText("Torque (" + cStr(robot_torque.length(), 2) + ")");
    }
    else {
        levelForce->setValue(0.0);
        levelTorque->setValue(0.0);
        labelForce->setText("Force");
        labelTorque->setText("Torque");
    }
    labelForce->setFontScale(1.0 / 500.0 * height);
    labelTorque->setFontScale(1.0 / 500.0 * height);



    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//---------------------------------------------------------------------------

void updateUtils(void) {
	int loopIdxSave = loopIdx;	
	int bufferIdx = 0;
	bool circular_buffer_full = false;

	SAVE_LOG = false;

	while (simulationRunning) {
		if (loopIdxSave != loopIdx && MISSION_STARTED) {
			// convert rotation matrices into axis angle vector
			cVector3d axisTool, axisDevice, axisStone, axisAngleTool, axisAngleDevice, axisAngleStone;
			double angleTool, angleDevice, angleStone;
			rotTool.toAxisAngle(axisTool, angleTool);
			rotDevice.toAxisAngle(axisDevice, angleDevice);
			rotStone.toAxisAngle(axisStone, angleStone);
			axisAngleTool = angleTool * axisTool;
			axisAngleDevice = angleDevice * axisTool;
			axisAngleStone = angleStone * axisStone;

			// log state 
			logTime[bufferIdx] = timeSystem;
			for (int i = 0; i < 3; i++) {
				logPosDevice[bufferIdx][i] = posDevice(i);
				logPosTool[bufferIdx][i] = posTool(i);
				logPosStone[bufferIdx][i] = posStone(i);
				logRotDevice[bufferIdx][i] = axisAngleDevice(i);
				logRotTool[bufferIdx][i] = axisAngleTool(i);
				logRotStone[bufferIdx][i] = axisAngleStone(i);
				logForceRobot[bufferIdx][i] = robot_force(i);
				logTorqueRobot[bufferIdx][i] = robot_torque(i);
				logForceFeedback[bufferIdx][i] = render_force(i);
			}
			logCollisionSet[bufferIdx] = collisionSet;

			bufferIdx++;
			loopIdxSave = loopIdx;
			if (bufferIdx >= LOGGING_BUFFER_SIZE) {
				bufferIdx = 0;
				circular_buffer_full = true;
			}
		}
	}

	time_t current;
	time(&current);
	struct tm *t = localtime(&current);

	char charFileName[256];
	sprintf_s(charFileName, "%ssimulation_log_%d_%d_%d_%d_%d_%d.txt", LOGGING_DIRECTORY, t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
	std::string fileName(charFileName);
	std::cout << "Start to save logged data into " << fileName << std::endl;
	FILE * logFile;
	int saveIdx = 0;
	int cnt = 0;
	if (circular_buffer_full) {
		if (bufferIdx < LOGGING_BUFFER_SIZE - 1)
			saveIdx = bufferIdx + 1;
	}

	logFile = fopen(fileName.c_str(), "a");
	while(SAVE_LOG) {
		if (saveIdx == bufferIdx) break;

		fprintf(logFile, "%f,  ", logTime[saveIdx]);
		for (unsigned int i = 0; i < 3; i++) fprintf(logFile, "%f,  ", logPosDevice[saveIdx][i]);
		for (unsigned int i = 0; i < 3; i++) fprintf(logFile, "%f,  ", logRotDevice[saveIdx][i]);			
		for (unsigned int i = 0; i < 3; i++) fprintf(logFile, "%f,  ", logPosTool[saveIdx][i]);		
		for (unsigned int i = 0; i < 3; i++) fprintf(logFile, "%f,  ", logRotTool[saveIdx][i]);			
		for (unsigned int i = 0; i < 3; i++) fprintf(logFile, "%f,  ", logPosStone[saveIdx][i]);
		for (unsigned int i = 0; i < 3; i++) fprintf(logFile, "%f,  ", logRotStone[saveIdx][i]);
		for (unsigned int i = 0; i < 3; i++) fprintf(logFile, "%f,  ", logForceRobot[saveIdx][i]);		
		for (unsigned int i = 0; i < 3; i++) fprintf(logFile, "%f,  ", logTorqueRobot[saveIdx][i]);		
		fprintf(logFile, "%d,   ", logCollisionSet[saveIdx]);
		for (unsigned int i = 0; i < 3; i++) fprintf(logFile, "%f,  ", logForceFeedback[saveIdx][i]);
		fprintf(logFile, "\n");

		cnt++;
		if (cnt % 1000 == 0) std::cout << cnt/1000 << " second data have been saved" << std::endl;
		
		saveIdx++;
		if (saveIdx >= LOGGING_BUFFER_SIZE) {
			if (circular_buffer_full) saveIdx = 0;
			else break;
		}
	}

	std::cout << "Log saving done" << std::endl;
	fclose(logFile);

	utilityThreadFinished = true;
}

//---------------------------------------------------------------------------
#ifdef SIMULATION
void updateHaptics(void)
{
	// initialize loop
	loopIdx = 0;

    // start haptic device
    hapticDevice->open();

    // simulation clock
    cPrecisionClock intervalClock;
	cPrecisionClock worldClock;
    intervalClock.start(true);
	worldClock.start(true);

    cMatrix3d prevRotTool;
    prevRotTool.identity();

    // main haptic simulation loop
    while (simulationRunning)
    {
        // update frequency counter
        freqCounterHaptics.signal(1);

        // retrieve simulation time and compute next interval
        double time = intervalClock.getCurrentTimeSeconds();
        //double nextSimInterval = cClamp(time, 0.00001, 0.0002);
        //cout << time << ", " << nextSimInterval << endl;
		timeSystem = worldClock.getCurrentTimeSeconds();

        // reset clock
        intervalClock.reset();
        intervalClock.start();

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        hapticDevice->getPosition(posDevice);
        hapticDevice->getRotation(rotDevice);

        cMatrix3d tool_offset(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0);
        //cMatrix3d tool_offset(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        //cMatrix3d tool_offset(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
        rotDevice = cMul(rotDevice, tool_offset);

        // scale position of device
        posDevice.mul(workspaceScaleFactor);
		devicePosDerivative.filter(posDevice, velDevice);

        // attach cursors (show device position)
		// Device cursor is endtip position
		cVector3d tool_endtip_desired, tool_handle_desired;
		tool_endtip_desired = rotDevice * cVector3d(0.0, 0.0, cube_size * 0.0) + posDevice;
		tool_handle_desired = rotDevice * cVector3d(0.0, 0.0, cube_size * 6.0) + posDevice;
        cursor_endtip->setLocalPos(tool_endtip_desired);
		cursor_endtip->setLocalRot(rotDevice);
		//cursor_handle->setLocalPos(tool_handle_desired);

        if (MISSION_STARTED) {
            // check success 
            MISSION_SUCCEEDED = true;
            num_stone_remain = NUM_STONE;
            for (int i = 0; i < NUM_STONE; i++) {
                MISSION_SUCCEEDED = MISSION_SUCCEEDED & ODEWorld->m_custom_rule[2 + i].collided;
                if (ODEWorld->m_custom_rule[2 + i].collided) num_stone_remain--;
            }

            // check failure
            MISSION_FAILED = false;
            MISSION_FAILED = MISSION_FAILED | ODEWorld->m_custom_rule[0].collided | ODEWorld->m_custom_rule[1].collided;

            // initialize dynamic objects and control rules
            if (MISSION_SUCCEEDED) {
                /*for (int i = 0; i < ODEWorld->m_num_rules; i++) {
                    ODEWorld->m_custom_rule[i].reset();
                }
                for (int i = 0; i < NUM_STONE; i++) {
                    ODEStone[i]->setLocalPos(stone_init_pos);
                    ODEStone[i]->setLocalRot(stone_init_rot);
                }
				ODETool->setLocalPos(tool_init_pos);
				ODETool->setLocalRot(tool_init_rot);
                ODEToolHandle->setLocalPos(tool_init_pos + cVector3d(cube_size*3.0625, 0.0, 0.0));
				ODEToolHandle->setLocalRot(tool_init_rot);

                num_success++;
                MISSION_STARTED = false;
				LPFSensor.reset();
				devicePosDerivative.reset();
				toolPosDerivative.reset();
				toolRotDerivative.reset();*/
				SAVE_LOG = true;
				glfwSetWindowShouldClose(window, GLFW_TRUE);
            }

            if (MISSION_FAILED) {
                /*for (int i = 0; i < ODEWorld->m_num_rules; i++) {
                    ODEWorld->m_custom_rule[i].reset();
                }
                for (int i = 0; i < NUM_STONE; i++) {
                    ODEStone[i]->setLocalPos(stone_init_pos);
                    ODEStone[i]->setLocalRot(stone_init_rot);
                }
				ODETool->setLocalPos(tool_init_pos);
				ODETool->setLocalRot(tool_init_rot);
				ODEToolHandle->setLocalPos(tool_init_pos + cVector3d(cube_size*3.0625, 0.0, 0.0));
				ODEToolHandle->setLocalRot(tool_init_rot);

                num_failure++;
                MISSION_STARTED = false;
				LPFSensor.reset();
				devicePosDerivative.reset();
				toolPosDerivative.reset();
				toolRotDerivative.reset();*/
				SAVE_LOG = false;
				glfwSetWindowShouldClose(window, GLFW_TRUE);
            }

			// generate virtual force to move tool
			posTool = ODEToolHandle->getLocalPos();
			rotTool = ODEToolHandle->getLocalRot();
			posStone = ODEStone[0]->getLocalPos();
			rotStone = ODEStone[0]->getLocalRot();
			cVector3d posToolDot, rotToolDot;
			cVector3d posError = (posDevice - posTool);
			cMatrix3d rotErrorMatrix = cMul(cTranspose(rotTool), rotDevice);
			double rotErrorAngle, rotAngle;
			cVector3d rotErrorAxis, rotAxis, rotError;
			rotErrorMatrix.toAxisAngle(rotErrorAxis, rotErrorAngle);
			rotTool.toAxisAngle(rotAxis, rotAngle);
			rotError = rotErrorAngle * rotErrorAxis;
			toolPosDerivative.filter(posTool, posToolDot);
			toolRotDerivative.filter(rotAngle*rotAxis, rotToolDot);

			robot_force = linStiffness*(posError - 0.1*posToolDot);
			robot_torque = angStiffness*(rotError - 0.1*rotToolDot);
			ODEToolHandle->getLocalRot().mul(robot_torque);
			
			ODEToolHandle->addExternalForce(robot_force);
			ODEToolHandle->addExternalTorque(robot_torque );
            
			/*
			// read position of tool
			//cVector3d tool_center, vec_center_handle, dir_center_handle, tool_endtip, tool_handle, tool_endtip_error, tool_handle_error;
			//tool_center = ODETool->getLocalTransform() * cVector3d(0.0, 0.0, 0.0);
			//tool_endtip = ODETool->getLocalTransform() * cVector3d(0.0, 0.0, cube_size * -3.0);
			//tool_handle = ODETool->getLocalTransform() * cVector3d(0.0, 0.0, cube_size * 3.0);
			//vec_center_handle = tool_handle - tool_center;
			//vec_center_handle.normalizer(dir_center_handle);

			//tool_endtip_error = tool_endtip_desired - tool_endtip;
			//tool_handle_error = tool_handle_desired - tool_handle;
			//cVector3d force_endtip, force_handle, torque_handle;
			//force_endtip = cMul(linStiffness, tool_endtip_error);
			//force_handle= cMul(linStiffness, tool_handle_error - dir_center_handle.dot(tool_handle_error)*dir_center_handle);
			//torque_handle = cMul(linStiffness * 1.0 / 3.0, cCross(dir_center_handle, tool_handle_error));

			//ODETool->addExternalForce(force_endtip + force_handle);
			//ODETool->addExternalTorque(torque_handle);
			*/

			// update simulation
			ODEWorld->updateDynamics(TIMESTEP);

			// check collision 
			collisionSet = 0;
			for (int i = 0; i < ODEWorld->toolCollisionIdx; i++) {
				if (ODEWorld->toolCollisionList[i] == ODEFence->m_ode_geom) collisionSet |= 1 << 0;
				if (ODEWorld->toolCollisionList[i] == ODEFence_side1->m_ode_geom) collisionSet |= 1 << 1;
				if (ODEWorld->toolCollisionList[i] == ODEFence_side2->m_ode_geom) collisionSet |= 1 << 2;
				if (ODEWorld->toolCollisionList[i] == ODELane1->m_ode_geom) collisionSet |= 1 << 3;
				if (ODEWorld->toolCollisionList[i] == ODELane2->m_ode_geom) collisionSet |= 1 << 4;
				if (ODEWorld->toolCollisionList[i] == ODESidewalk1->m_ode_geom) collisionSet |= 1 << 5;
				if (ODEWorld->toolCollisionList[i] == ODESidewalk2->m_ode_geom) collisionSet |= 1 << 6;
				if (ODEWorld->toolCollisionList[i] == ODELava->m_ode_geom) collisionSet |= 1 << 7;
				if (ODEWorld->toolCollisionList[i] == ODEFurnace->m_ode_geom) collisionSet |= 1 << 8;
				for (int j = 0; j < NUM_STONE; j++)
					if (ODEWorld->toolCollisionList[i] == ODEStone[j]->m_ode_geom) collisionSet |= 1 << (9+j);
			}
			//std::cout << collisionSet << endl;

			// read tool force
			cVector3d tool_force(FTSensorMeasure.f2[0], FTSensorMeasure.f2[1], FTSensorMeasure.f2[2]);
			cVector3d tool_force_filtered;
			LPFSensor.filter(tool_force, tool_force_filtered);
			//cout << "force: " << FTSensorMeasure.f2[0] << ", " << FTSensorMeasure.f2[1] << ", " << FTSensorMeasure.f2[2] << ", " << FTSensorMeasure.f2[3] << endl;
			//cout << velDevice << endl;
			

			render_force = 30.0*linG * tool_force_filtered;
			if (render_force.length() > 1.0) {
				render_force = render_force / render_force.length() * 1.0;
			}
			render_force += -0.03*velDevice - 0.2*linStiffness*posError / workspaceScaleFactor;
			//render_force = -0.03*velDevice - 0.2*linStiffness*posError / workspaceScaleFactor;
			//hapticDevice->setForce(render_force);
        }
        else {
            //hapticDevice->setForceAndTorqueAndGripperForce(cVector3d(0.0, 0.0, 0.0), cVector3d(0.0, 0.0, 0.0), 0.0);

            // check whether the HIP approaches the starting point
            double dist = posDevice.distance(starting_pos);
            if (dist < cube_size && posDevice.z() > -0.6) MISSION_STARTED = true;

			// update simulation
			ODEWorld->updateDynamics(TIMESTEP);

			render_force = -0.03*velDevice;
			//hapticDevice->setForce();
        }

		hapticDevice->setForce(render_force);


        if (linG < linGain)
        {
            linG = linG + 0.1 * time * linGain;
        }
        else
        {
            linG = linGain;
        }

        if (angG < angGain)
        {
            angG = angG + 0.1 * time * angGain;
        }
        else
        {
            angG = angGain;
        }

		loopIdx++;
    }

	std::cout << "Simulation ended" << endl;

    // exit haptics thread
    simulationFinished = true;
}
#else
cMatrix3d rotateY(double theta) {
    cMatrix3d offset(cos(theta / 180.0 * M_PI), 0.0, sin(theta / 180.0 * M_PI), 0.0, 1.0, 0.0, -sin(theta / 180.0 * M_PI), 0.0, cos(theta / 180.0 * M_PI));
    return offset;
}

void updateHaptics() {
    // start haptic device
    hapticDevice->open();

    // simulation clock
    cPrecisionClock intervalClock;
    intervalClock.start(true);
    
    int count = 0;
    // main haptic simulation loop
    while (simulationRunning)
    {
        // update frequency counter
        freqCounterHaptics.signal(1);

        // retrieve simulation time and compute next interval
        double time = intervalClock.getCurrentTimeSeconds();
        double nextSimInterval = cClamp(time, 0.00001, 0.0002);
        //cout << time << ", " << nextSimInterval << endl;

        // reset clock
        intervalClock.reset();
        intervalClock.start();


        // update position and orientation of tool
        cVector3d posDevice;
        cMatrix3d rotDevice;
        hapticDevice->getPosition(posDevice);
        hapticDevice->getRotation(rotDevice);

        cMatrix3d tool_offset(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0);
        //cMatrix3d tool_offset(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        //cMatrix3d tool_offset(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
        rotDevice = cMul(rotDevice, tool_offset);
        //cMatrix3d tool_offset2(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
        //rotDevice = cMul(rotDevice, cMul(tool_offset2, tool_offset));

        // scale position of device
        posDevice.mul(2.0);

        // attach cursor (show device position)
        handle->setLocalPos(posDevice);
        handle->setLocalRot(rotDevice);

        if (count % 100000 == 0 && indy_cmode == 20) {
            //cMatrix3d indy_offset1(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
            cMatrix3d indy_offset1(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0);
            cMatrix3d indy_offset2(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);
            cMatrix3d indy_offset3(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);
            rotDevice = cMul(rotateY(-90), rotDevice);
            rotDevice = cMul(rotDevice, rotateY(-90.0));
            //rotDevice = cMul(indy_offset2, rotDevice);
            rotDevice.trans();
            //rotDevice = cMul(rotDevice, indy_offset3);

            Eigen::Matrix3d rotDeviceEigen;
            rotDeviceEigen(0, 0) = rotDevice(0, 0);
            rotDeviceEigen(0, 1) = rotDevice(0, 1);
            rotDeviceEigen(0, 2) = rotDevice(0, 2);
            rotDeviceEigen(1, 0) = rotDevice(1, 0);
            rotDeviceEigen(1, 1) = rotDevice(1, 1);
            rotDeviceEigen(1, 2) = rotDevice(1, 2);
            rotDeviceEigen(2, 0) = rotDevice(2, 0);
            rotDeviceEigen(2, 1) = rotDevice(2, 1);
            rotDeviceEigen(2, 2) = rotDevice(2, 2);
            //Eigen::Matrix3d rotDeviceEigen_t = rotDeviceEigen.transpose();
            cVector3d euler = rotDeviceEigen.eulerAngles(2, 1, 0);
            //cout << "rot: " << euler << endl;

            double pos[6];
            double vel[6] = { 0.0 };
            pos[0] = - posDevice.x() + 0.80;
            pos[1] = - posDevice.y();
            pos[2] = posDevice.z() + 0.20;
            pos[3] = euler(0) * 180.0 / M_PI;
            pos[4] = euler(1) * 180.0 / M_PI;
            pos[5] = euler(2) * 180.0 / M_PI;
            indyTCP.SetRefState(pos, vel);
            if (count % 100000) {
                indy_cmode = indyTCP.GetCMode();
            }
        }

        count++;

    }
    // exit haptics thread
    simulationFinished = true;
}
#endif
//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // help menu:
    else if (a_key == GLFW_KEY_H)
    {
        cout << "Keyboard Options:" << endl << endl;
        cout << "[h] - Display help menu" << endl;
        cout << "[1] - Enable gravity" << endl;
        cout << "[2] - Disable gravity" << endl << endl;
        cout << "[3] - decrease linear haptic gain" << endl;
        cout << "[4] - increase linear haptic gain" << endl;
        cout << "[5] - decrease angular haptic gain" << endl;
        cout << "[6] - increase angular haptic gain" << endl << endl;
        cout << "[7] - decrease linear stiffness" << endl;
        cout << "[8] - increase linear stiffness" << endl;
        cout << "[9] - decrease angular stiffness" << endl;
        cout << "[0] - increase angular stiffness" << endl << endl;
#ifndef SIMULATION
        cout << "[m] - toggle teleoperation mode" << endl;
        cout << "[a] - test1" << endl;
        cout << "[b] - test2" << endl;
#else
        cout << "[m] - toggle vertical mirror" << endl;
#endif
        cout << "[f] - toggle full screen\n" << endl;
        cout << "[q] - Exit application\n" << endl;
        cout << endl << endl;
    }

    // option - enable gravity:
    else if (a_key == GLFW_KEY_1)
    {
        // enable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
        printf("gravity ON:\n");
    }

    // option - disable gravity:
    else if (a_key == GLFW_KEY_2)
    {
        // disable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
        printf("gravity OFF:\n");
    }

    // option - decrease linear haptic gain
    else if (a_key == GLFW_KEY_3)
    {
        linGain = linGain - 0.05;
        if (linGain < 0)
            linGain = 0;
        printf("linear haptic gain:  %f\n", linGain);
    }

    // option - increase linear haptic gain
    else if (a_key == GLFW_KEY_4)
    {
        linGain = linGain + 0.05;
        printf("linear haptic gain:  %f\n", linGain);
    }

    // option - decrease angular haptic gain
    else if (a_key == GLFW_KEY_5)
    {
        angGain = angGain - 0.005;
        if (angGain < 0)
            angGain = 0;
        printf("angular haptic gain:  %f\n", angGain);
    }

    // option - increase angular haptic gain
    else if (a_key == GLFW_KEY_6)
    {
        angGain = angGain + 0.005;
        printf("angular haptic gain:  %f\n", angGain);
    }

    // option - decrease linear stiffness
    else if (a_key == GLFW_KEY_7)
    {
        linStiffness = linStiffness - 50;
        if (linStiffness < 0)
            linStiffness = 0;
        printf("linear stiffness:  %f\n", linStiffness);
    }

    // option - increase linear stiffness
    else if (a_key == GLFW_KEY_8)
    {
        linStiffness = linStiffness + 50;
        printf("linear stiffness:  %f\n", linStiffness);
    }

    // option - decrease angular stiffness
    else if (a_key == GLFW_KEY_9)
    {
        angStiffness = angStiffness - 1;
        if (angStiffness < 0)
            angStiffness = 0;
        printf("angular stiffness:  %f\n", angStiffness);
    }

    // option - increase angular stiffness
    else if (a_key == GLFW_KEY_0)
    {
        angStiffness = angStiffness + 1;
        printf("angular stiffness:  %f\n", angStiffness);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle teleoperation mode
    else if (a_key == GLFW_KEY_M)
    {
#ifdef SIMULATION
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
#else
        indyTCP.ToggleTeleoperationMode();
        indy_cmode = indyTCP.GetCMode();
        cout << indy_cmode << endl;
#endif
    }

#ifndef SIMULATION
    // option - test1
    else if (a_key == GLFW_KEY_A)
    {
        indyTCP.SetRefState(pRef2, pdotRef2);
    }

    else if (a_key == GLFW_KEY_B)
    {
        indyTCP.SetRefState(pRef1, pdotRef1);
    }

    else if (a_key == GLFW_KEY_S)
    {
        float j_home[6] = { 0.0, -15.0, -90.0, 0.0, 15.0, 0.0 };
        indyTCP.MoveToJ(j_home);
        while (!indyTCP.isMoveFinished()) {
            int a = 0;
        }    
    }
#endif
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished || !utilityThreadFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

	dJointGroupDestroy(sensorGroup);

    // delete resources
    delete hapticsThread;
	delete utilThread;
    delete world;
    delete handler;
}

//---------------------------------------------------------------------------