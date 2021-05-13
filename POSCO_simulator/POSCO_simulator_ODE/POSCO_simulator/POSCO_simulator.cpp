#pragma once

#include "POSCO_simulator.h"
//#include "utils.h"

int main() 
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "furnace ODE simulation" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    if (!glfwInit()){
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

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
    if (stereoMode == C_STEREO_ACTIVE) glfwWindowHint(GLFW_STEREO, GL_TRUE);
    else glfwWindowHint(GLFW_STEREO, GL_FALSE);

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window) {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    glfwGetWindowSize(window, &width, &height);
    glfwSetWindowPos(window, x, y);

    glfwSetKeyCallback(window, keyCallback);
    glfwSetWindowSizeCallback(window, windowSizeCallback);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(swapInterval);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK) {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif

    //-----------------------------------------------------------------------
    // SIMULATION ENVIRONMENT
    //-----------------------------------------------------------------------
#if defined(USE_INDY)
    workspace = new cIndyFurnace();
#else
#if defined(USE_ODE)
	workspace = new cODEFurnace();
#elif defined(USE_BULLET)
    workspace = new cBulletFurnace();
#endif
#endif
    inputCursorPosition = cVector3d(INITIAL_CURSOR_POS_X, INITIAL_CURSOR_POS_Y, INITIAL_CURSOR_POS_Z);
    inputCursorRotation = rotateZ(M_PI);
    workspace->setEndEffectorPose(cVector3d(INITIAL_CURSOR_POS_X, INITIAL_CURSOR_POS_Y, INITIAL_CURSOR_POS_Z), inputCursorRotation);

    workspace->setEndEffectorStiffness(10.0, 3.0);
    workspace->maxForce = 50.0;
    workspace->maxTorque = 30.0;

    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    if (cCustomFurnaceDevice::getNumDevices() > 0) {
        hapticDevice = cCustomFurnaceDevice::create(0);
        if (hapticDevice->open()) {
            hapticDevice->close();
            isHapticDeviceAvailable = true;
        }
        else {
            isHapticDeviceAvailable = false;
        }
    }
    else {
        isHapticDeviceAvailable = false;
    }
    printf("[isHapticDeviceAvailable] %d\n", isHapticDeviceAvailable);
    // retrieve information about the current haptic device
    if (isHapticDeviceAvailable) cHapticDeviceInfo hapticDeviceInfo = hapticDevice->m_specifications;

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    cFontPtr font = NEW_CFONTCALIBRI20();

    panel = new cPanel();
    workspace->addVisualComponent(panel);
    panel->setColor(cColorf(0.0, 0.0, 0.0));
    panel->setTransparencyLevel(0.8);
    //cout << width << ", " << height << endl;

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setWhite();
    workspace->addVisualComponent(labelRates);

    labelStatus = new cLabel(font);
    labelStatus->m_fontColor.setWhite();
    workspace->addVisualComponent(labelStatus);

    labelSuccess = new cLabel(font);
    labelSuccess->m_fontColor.setWhite();
    workspace->addVisualComponent(labelSuccess);

    labelFailure = new cLabel(font);
    labelFailure->m_fontColor.setWhite();
    workspace->addVisualComponent(labelFailure);

    labelForce = new cLabel(font);
    labelForce->m_fontColor.setWhite();
    workspace->addVisualComponent(labelForce);

    labelTorque = new cLabel(font);
    labelTorque->m_fontColor.setWhite();
    workspace->addVisualComponent(labelTorque);

    //levelForce = new cLevel();
    //workspace->addVisualComponent(levelForce);
    //levelForce->setRange(0.0, workspace->maxForce);
    //levelForce->setSingleIncrementDisplay(true);
    //levelForce->rotateWidgetDeg(270);
    //levelForce->m_colorActive.setBlueCornflower();
    //levelForce->m_colorInactive = levelForce->m_colorActive;
    //levelForce->m_colorInactive.mul(0.3f);
    scopeForce = new cScope();
    workspace->addVisualComponent(scopeForce);
    scopeForce->setRange(0.0, workspace->maxForce*1.1);
    scopeForce->setSignalEnabled(true, true, false, false);
    scopeForce->m_colorSignal1.set(1.0, 0.0, 0.0);
    scopeTorque = new cScope();
    workspace->addVisualComponent(scopeTorque);
    scopeTorque->setRange(0.0, workspace->maxTorque*1.1);
    scopeTorque->setSignalEnabled(true, true, false, false);
    scopeTorque->m_colorSignal1.set(1.0, 0.0, 0.0);

    //levelTorque = new cLevel();
    //workspace->addVisualComponent(levelTorque);
    //levelTorque->setRange(0.0, workspace->maxTorque);
    //levelTorque->setSingleIncrementDisplay(true);
    //levelTorque->rotateWidgetDeg(270);
    //levelTorque->m_colorActive.setBlueCornflower();
    //levelTorque->m_colorInactive = levelTorque->m_colorActive;
    //levelTorque->m_colorInactive.mul(0.3f);

    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    userInterfaceThread = new cThread();
    userInterfaceThread->start(updateUserInterface, CTHREAD_PRIORITY_HAPTICS);

    worldThread = new cThread();
    worldThread->start(updateWorld, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window)) {
        glfwGetWindowSize(window, &width, &height);
        updateGraphics();
        glfwSwapBuffers(window);
        glfwPollEvents();
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);
    glfwTerminate();

	return 0;
}
//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height){
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
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT)) return;

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q)){
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F) {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen) {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - help menu
    else if (a_key == GLFW_KEY_H) {
        cout << "Keyboard Options:" << endl << endl;
        cout << "[h] - Display help menu" << endl;
        cout << "[q] - Exit application\n" << endl;
        cout << endl << endl;
    }

    else if (a_key == GLFW_KEY_M) {
        teleoperationMode = !teleoperationMode;
    }

    else if (a_key == GLFW_KEY_C) {
        if (isHapticDeviceAvailable) {
            hapticDevice->calibrate();
        }
    }

#ifdef USE_INDY
    else if (a_key == GLFW_KEY_O) {
        workspace->switchCameraMode();
    }

    else if (a_key == GLFW_KEY_R) {
        workspace->moveRobotHome();
    }

    else if (a_key == GLFW_KEY_T) {
        workspace->toggleDirectTeachingMode();
    }

    else if (a_key == GLFW_KEY_Z) {
        workspace->moveRobotZero();
    }

    else if (a_key == GLFW_KEY_P) {
        workspace->test();
    }
#endif
    else if (a_key == GLFW_KEY_W) {
        if (!teleoperationMode) {
            workspace->moveCamera(cameraMovementStepSize * cVector3d(1.0, 0.0, 0.0));
        }
        else {
            inputCursorPosition += cMul(inputCursorRotation, cameraMovementStepSize * cVector3d(1.0, 0.0, 0.0));
        }
    }

    else if (a_key == GLFW_KEY_S) {
        if (!teleoperationMode) {
            workspace->moveCamera(cameraMovementStepSize * cVector3d(-1.0, 0.0, 0.0));
        }
        else {
            inputCursorPosition += cMul(inputCursorRotation, cameraMovementStepSize * cVector3d(-1.0, 0.0, 0.0));
        }
    }

    else if (a_key == GLFW_KEY_A) {
        if (!teleoperationMode) {
            workspace->moveCamera(cameraMovementStepSize * cVector3d(0.0, 1.0, 0.0));
        }
        else {
            inputCursorPosition += cMul(inputCursorRotation, cameraMovementStepSize * cVector3d(0.0, 1.0, 0.0));
        }
    }

    else if (a_key == GLFW_KEY_D) {
        if (!teleoperationMode) {
            workspace->moveCamera(cameraMovementStepSize * cVector3d(0.0, -1.0, 0.0));
        }
        else {
            inputCursorPosition += cMul(inputCursorRotation, cameraMovementStepSize * cVector3d(0.0, -1.0, 0.0));
        }
    }

    else if (a_key == GLFW_KEY_PAGE_UP) {
        if (!teleoperationMode) {
            workspace->moveCamera(cameraMovementStepSize * cVector3d(0.0, 0.0, 1.0));
        }
        else {
            inputCursorPosition += cameraMovementStepSize * cVector3d(0.0, 0.0, 1.0);
        }
    }

    else if (a_key == GLFW_KEY_PAGE_DOWN) {
        if (!teleoperationMode) {
            workspace->moveCamera(cameraMovementStepSize * cVector3d(0.0, 0.0, -1.0));
        }
        else {
            inputCursorPosition += cameraMovementStepSize * cVector3d(0.0, 0.0, -1.0);
        }
    }

    else if (a_key == GLFW_KEY_UP) {
        if (!teleoperationMode) {
            workspace->rotateCamera(0.0, cameraRotationStepSize);
        }
        else {
            inputCursorRotation *= cTranspose(rotateY(motionRotationStepSize));
        }
    }

    else if (a_key == GLFW_KEY_DOWN) {
        if (!teleoperationMode) {
            workspace->rotateCamera(0.0, -cameraRotationStepSize);
        }
        else {
            inputCursorRotation *= cTranspose(rotateY(-motionRotationStepSize));
        }
    }

    else if (a_key == GLFW_KEY_LEFT) {
        if (!teleoperationMode) {
            workspace->rotateCamera(-cameraRotationStepSize, 0.0);
        }
        else {
            inputCursorRotation *= cTranspose(rotateZ(motionRotationStepSize));
        }
    }

    else if (a_key == GLFW_KEY_RIGHT) {
        if (!teleoperationMode) {
            workspace->rotateCamera(cameraRotationStepSize, 0.0);
        }
        else {
            inputCursorRotation *= cTranspose(rotateZ(-motionRotationStepSize));
        }
    }
}

//---------------------------------------------------------------------------

void close(void){
    // stop the simulation
    userInterfaceCommunicating = false;
    worldRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!userInterfaceThreadFinished) { cSleepMs(100); }
    while (!worldThreadFinished) { cSleepMs(100); }

    // close haptic device
    if (isHapticDeviceAvailable) hapticDevice->close();

    // delete resources
    delete userInterfaceThread;
    delete worldThread;
}

//---------------------------------------------------------------------------

void updateGraphics(void){
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

     // Background panel
    //panel->setCornerRadius(height / 60, height / 60, height / 60, height / 60);
    //panel->setSize(width * 0.9, height * 0.10);
    //panel->setLocalPos(width * 0.05, height * 0.90);
    panel->setCornerRadius(height / 100, height / 100, height / 100, height / 100);
    panel->setSize(width * 0.2, height * 0.5);
    panel->setLocalPos(width * 0.8, height * 0.5);

    // Frequency
    labelRates->setText("Graphic freq./UI freq./World freq \r\n" + cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " + cStr(freqCounterUserInterface.getFrequency(), 0) + " Hz / " + cStr(freqCounterWorld.getFrequency(), 0) + " Hz");
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getTextWidth())), height * 0.05);
    labelRates->setFontScale(1.0 / 500.0 * height);

    //// Level (force, torque)
    //levelForce->setLocalPos(width * 4.0 / 16.0, height * 0.97);
    //levelForce->setWidth(height * 0.03);
    //levelTorque->setLocalPos(width * 11.0 / 16.0, height * 0.97);
    //levelTorque->setWidth(height * 0.03);
    //levelForce->setNumIncrements(height / 540 * 100);
    //levelTorque->setNumIncrements(height / 540 * 100);
    //levelForce->setValue(sensorForce.length() * 10.0);
    //levelTorque->setValue(sensorTorque.length() * 10.0);

    scopeForce->setLocalPos(width * 0.81, height * 0.8);
    scopeForce->setSize(width * 0.18, height*0.15);
    scopeTorque->setLocalPos(width * 0.81, height * 0.6);
    scopeTorque->setSize(width * 0.18, height * 0.15);
    if (worldRunning) {
        scopeForce->setSignalValues(sensorForce.length(), workspace->maxForce);
        scopeTorque->setSignalValues(sensorTorque.length(), workspace->maxTorque);
    }

    //labelForce->setLocalPos(width / 8.0, height * 0.94);
    //labelTorque->setLocalPos(width * 9.0 / 16.0, height * 0.94);
    labelForce->setLocalPos(width * 0.81, height * 0.95);
    labelTorque->setLocalPos(width * 0.81, height * 0.75);
    if (worldRunning) {
        labelForce->setText("Force (" + cStr(sensorForce.length(), 2) + ")");
        labelTorque->setText("Torque (" + cStr(sensorTorque.length(), 2) + ")");
        //labelForce->setText("Force");
        //labelTorque->setText("Torque");
    }
    labelForce->setFontScale(1.0 / 500.0 * height);
    labelTorque->setFontScale(1.0 / 500.0 * height);

#ifndef USE_INDY
    /////////////////////////////////////////////////////////////////////
    // UPDATE CAMERA
    /////////////////////////////////////////////////////////////////////
    // calculate tool endtip position
    cVector3d endEffectorPosition;
    cMatrix3d endEffectorRotation;
    workspace->getEndEffectorPose(endEffectorPosition, endEffectorRotation);
    cVector3d Eye_Point = cVector3d(-0.3, 0.0, 0.0);
    cVector3d Target_Point = cVector3d(END_EFFECTOR_TOOL_LENGTH, 0.0, 0.0);
    cVector3d Eye_Point_Global = endEffectorRotation * Eye_Point + endEffectorPosition + cVector3d(0.0, 0.0, 0.1);
    cVector3d Target_Point_Global = endEffectorRotation * Target_Point + endEffectorPosition;

    // position and orient the camera
    workspace->updateCameraPose(Eye_Point_Global,      // camera position (eye)
        Target_Point_Global);                        // lookat position (target)
#endif

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////
    workspace->updateShadowMaps(false, mirroredDisplay);
    workspace->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateUserInterface(void){
    // simulation in now running
    userInterfaceCommunicating = true;
    userInterfaceThreadFinished = false;

    // start haptic device
    if (isHapticDeviceAvailable) hapticDevice->open();
    else cout << "[updateUserInterface] Haptic device is not available!" << endl;

    // simulation clock
    cPrecisionClock userInterfaceClock;
    userInterfaceClock.start(true);

    // main haptic simulation loop
    while (userInterfaceCommunicating){

        // retrieve simulation time and compute next interval
        double time = userInterfaceClock.getCurrentTimeSeconds();
        
        if (time < 0.001) {
            continue;
        }

        // reset clock
        userInterfaceClock.reset();
        userInterfaceClock.start();

        // signal frequency counter
        freqCounterUserInterface.signal(1);

        if (isHapticDeviceAvailable) {     
            cVector3d devicePosition;
            cMatrix3d deviceRotation;
            if (hapticDevice->getPosition(devicePosition) && hapticDevice->getRotation(deviceRotation)) {
                devicePosition.mul(workspaceScaleFactor);
            };
            unsigned int deviceSwitch;
            hapticDevice->getUserSwitches(deviceSwitch);

            if (deviceSwitch == 2) {
                if (!isButtonPressed) {
                    // set device zero pose
                    cVector3d endEffectorPosition;
                    cMatrix3d endEffectorRotation;
                    workspace->getEndEffectorPose(endEffectorPosition, endEffectorRotation);
                    deviceZeroPosition = endEffectorPosition - devicePosition;
                    deviceZeroRotation = cMul(cTranspose(deviceRotation), endEffectorRotation);

                    isButtonPressed = true;
                    printf("button pressed\n");
                }
                else {
                    commandActivate = true;
                }
            }
            else {
                isButtonPressed = false;
                commandActivate = false;
            }

            inputCursorPosition = deviceZeroPosition + devicePosition;
            inputCursorRotation = cMul(deviceRotation, deviceZeroRotation);

#ifdef USE_INDY
            double maxRenderForce = 5.0;
            double maxRenderTorque = 0.05;
            renderForce = sensorForce * 3.0;
            renderTorque = sensorTorque * 0.1;

            if (renderForce.length() > maxRenderForce) {
                renderForce = renderForce / renderForce.length() * maxRenderForce;
            }
            if (renderTorque.length() > maxRenderTorque) {
                renderTorque = renderTorque / renderTorque.length() * maxRenderTorque;
            }

            if (commandActivate) {
                hapticDevice->setForceAndTorqueAndGripperForce(renderForce, renderTorque, 0.0);
            }
            else {
                //printf("%.3f, %.3f, %.3f\n", renderTorque(0), renderTorque(1), renderTorque(2));
                hapticDevice->setForceAndTorqueAndGripperForce(cVector3d(0.0, 0.0, 0.0), cVector3d(0.0, 0.0, 0.0), 0.0);
            }
            //hapticDevice->setForceAndTorqueAndGripperForce(renderForce, renderTorque, 0.0);
#else
            double maxRenderForce = 4.0;
            double maxRenderTorque = 0.05;
            renderForce = sensorForce * 1.0;
            renderTorque = sensorTorque * 0.04;

            if (renderForce.length() > maxRenderForce) {
                renderForce = renderForce / renderForce.length() * maxRenderForce;
            }
            if (renderTorque.length() > maxRenderTorque) {
                renderTorque = renderTorque / renderTorque.length() * maxRenderTorque;
            }

            if (commandActivate) {
                //printf("%.3f, %.3f, %.3f\n", renderTorque(0), renderTorque(1), renderTorque(2));
                hapticDevice->setForceAndTorqueAndGripperForce(renderForce, renderTorque, 0.0);
            }
            else {
                //printf("%.3f, %.3f, %.3f\n", renderTorque(0), renderTorque(1), renderTorque(2));
                hapticDevice->setForceAndTorqueAndGripperForce(cVector3d(0.0, 0.0, 0.0), cVector3d(0.0, 0.0, 0.0), 0.0);
            }
#endif
        } 
        else {
            commandActivate = true;
        }
    }

    // exit haptics thread
    userInterfaceThreadFinished = true;
}


void updateWorld(void){
    // simulation in now running
    worldRunning = true;
    worldThreadFinished = false;

    // simulation clock
    cPrecisionClock worldClock;
    worldClock.start(true);

    // main haptic simulation loop
    while (worldRunning) {

        // retrieve simulation time and compute next interval
        double time = worldClock.getCurrentTimeSeconds();

#if defined(USE_INDY)
        while (time < 0.00199) {
            time = worldClock.getCurrentTimeSeconds();
        }
#else
        while (time < 0.00199) {
            time = worldClock.getCurrentTimeSeconds();
        }
#endif

        //double nextSimInterval = 0.001;//
        double nextSimInterval = time;

        // reset clock
        worldClock.reset();
        worldClock.start();

        // signal frequency counter
        freqCounterWorld.signal(1);

        // compute global reference frames for each object
        workspace->computeGlobalPositions(true);

#ifdef USE_INDY
        if (commandActivate) {
            if (workspace->getRobotControlMode() != 20) {
                workspace->startTeleoperationMode();
            }
        }
        else {
            if (workspace->getRobotControlMode() == 20) {
                workspace->quitTeleoperationMode();
            }
        }
#endif

        // update position and orientation of tool
        //printf("%.3f, %.3f, %.3f\n", inputCursorPosition(0), inputCursorPosition(1), inputCursorPosition(2));

        workspace->updateUserCommand(inputCursorPosition, inputCursorRotation, commandActivate);
        workspace->getForceTorqueSensorValue(sensorForce, sensorTorque);
        workspace->updateDynamics(nextSimInterval);


#ifdef USE_INDY
        //// check simulation rule
        //int res = workspace->checkSimulationRule();
        //if (res > 0) {
        //    printf("Demonstration succeeded (%d)\n", res);

        //    panelMessage = new cPanel();
        //    panelMessage->setColor(cColorf(0.0, 0.0, 0.0));
        //    panelMessage->setTransparencyLevel(0.8);
        //    panelMessage->setCornerRadius(height / 100, height / 100, height / 100, height / 100);
        //    panelMessage->setSize(width, height * 0.2);
        //    panelMessage->setLocalPos(0, height * 0.4);
        //    workspace->addVisualComponent(panelMessage);

        //    cFontPtr font = NEW_CFONTCALIBRI20();
        //    labelMessage = new cLabel(font);
        //    labelMessage->m_fontColor.setWhite();

        //    labelMessage->setText("Demonstration succeeded.");
        //    labelMessage->setFontScale(1.5 / 500.0 * height);
        //    labelMessage->setLocalPos((int)(0.5 * (width - labelMessage->getTextWidth())), (int)(0.5 * (height - labelMessage->getTextHeight())));

        //    workspace->addVisualComponent(labelMessage);
        //    worldRunning = false;
        //}
#else
        // check simulation rule
        int res = workspace->checkSimulationRule();
        if (res > 0) {
            printf("Demonstration succeeded (%d)\n", res);

            panelMessage = new cPanel();
            panelMessage->setColor(cColorf(0.0, 0.0, 0.0));
            panelMessage->setTransparencyLevel(0.8);
            panelMessage->setCornerRadius(height / 100, height / 100, height / 100, height / 100);
            panelMessage->setSize(width, height * 0.2);
            panelMessage->setLocalPos(0, height * 0.4);
            workspace->addVisualComponent(panelMessage);

            cFontPtr font = NEW_CFONTCALIBRI20();
            labelMessage = new cLabel(font);
            labelMessage->m_fontColor.setWhite();

            labelMessage->setText("Demonstration succeeded.");
            labelMessage->setFontScale(1.5 / 500.0 * height);
            labelMessage->setLocalPos((int)(0.5 * (width - labelMessage->getTextWidth())), (int)(0.5 * (height - labelMessage->getTextHeight())));

            workspace->addVisualComponent(labelMessage);
            sensorForce = cVector3d(0.0, 0.0, 0.0);
            sensorTorque = cVector3d(0.0, 0.0, 0.0);
            worldRunning = false;
        }
        if (res < 0) {
            printf("Demonstration failed (%d)\n", res);

            panelMessage = new cPanel();
            panelMessage->setColor(cColorf(0.0, 0.0, 0.0));
            panelMessage->setTransparencyLevel(0.8);
            panelMessage->setCornerRadius(height / 100, height / 100, height / 100, height / 100);
            panelMessage->setSize(width, height * 0.2);
            panelMessage->setLocalPos(0, height * 0.4);
            workspace->addVisualComponent(panelMessage);

            cFontPtr font = NEW_CFONTCALIBRI20();
            labelMessage = new cLabel(font);
            labelMessage->m_fontColor.setRed();
            if (res == -1) labelMessage->setText("Tool touched flowing iron, demonstration failed.");
            if (res == -2) labelMessage->setText("Tool touched pouring iron, demonstration failed.");
            if (res == -3) labelMessage->setText("Excessive force occured, demonstration failed.");
            if (res == -4) labelMessage->setText("Excessive torque occured, demonstration failed.");

            labelMessage->setFontScale(1.5 / 500.0 * height);
            labelMessage->setLocalPos((int)(0.5 * (width - labelMessage->getTextWidth())), (int)(0.5 * (height - labelMessage->getTextHeight())));

            workspace->addVisualComponent(labelMessage);
            sensorForce = cVector3d(0.0, 0.0, 0.0);
            sensorTorque= cVector3d(0.0, 0.0, 0.0);
            worldRunning = false;
        }
#endif

    }

    // exit haptics thread
    worldThreadFinished = true;
}


