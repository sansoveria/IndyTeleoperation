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
    cout << "Demo: 05-bullet-dental" << endl;
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
	simEnvironment = new cFurnace();
    simEnvironment->setEndEffectorPose(cVector3d(INITIAL_CURSOR_POS_X, INITIAL_CURSOR_POS_Y, INITIAL_CURSOR_POS_Z), identityMatrix());
    keyInputCursorPosition = cVector3d(INITIAL_CURSOR_POS_X, INITIAL_CURSOR_POS_Y, INITIAL_CURSOR_POS_Z);
    keyInputCursorRotation = rotateY(-M_PI/2.0);
    simEnvironment->setEndEffectorStiffness(10.0, 3.0);

    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    cFontPtr font = NEW_CFONTCALIBRI20();

    labelRates = new cLabel(font);
    labelRates->m_fontColor.setWhite();
    simEnvironment->addVisualComponent(labelRates);

    //panelRates = new cPanel();
    //panelRates->setSize(300, 200);
    //panelRates->setCornerRadius(10, 10, 10, 10);
    //panelRates->setLocalPos(40, 60);
    //panelRates->setColor(cColorf(1.0, 1.0, 1.0));
    //panelRates->setTransparencyLevel(0.5);
    //simEnvironment->addVisualComponent(panelRates);

    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

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

    else if (a_key == GLFW_KEY_W) {
        if (!teleoperationMode) {
            simEnvironment->moveCamera(cameraMovementStepSize * cVector3d(1.0, 0.0, 0.0));
        }
        else {
            keyInputCursorPosition += cameraMovementStepSize * cVector3d(1.0, 0.0, 0.0);
        }
    }

    else if (a_key == GLFW_KEY_S) {
        if (!teleoperationMode) {
            simEnvironment->moveCamera(cameraMovementStepSize * cVector3d(-1.0, 0.0, 0.0));
        }
        else {
            keyInputCursorPosition += cameraMovementStepSize * cVector3d(-1.0, 0.0, 0.0);
        }
    }

    else if (a_key == GLFW_KEY_A) {
        if (!teleoperationMode) {
            simEnvironment->moveCamera(cameraMovementStepSize * cVector3d(0.0, 1.0, 0.0));
        }
        else {
            keyInputCursorPosition += cameraMovementStepSize * cVector3d(0.0, 1.0, 0.0);
        }
    }

    else if (a_key == GLFW_KEY_D) {
        if (!teleoperationMode) {
            simEnvironment->moveCamera(cameraMovementStepSize * cVector3d(0.0, -1.0, 0.0));
        }
        else {
            keyInputCursorPosition += cameraMovementStepSize * cVector3d(0.0, -1.0, 0.0);
        }
    }

    else if (a_key == GLFW_KEY_PAGE_UP) {
        if (!teleoperationMode) {
            simEnvironment->moveCamera(cameraMovementStepSize * cVector3d(0.0, 0.0, 1.0));
        }
        else {
            keyInputCursorPosition += cameraMovementStepSize * cVector3d(0.0, 0.0, 1.0);
        }
    }

    else if (a_key == GLFW_KEY_PAGE_DOWN) {
        if (!teleoperationMode) {
            simEnvironment->moveCamera(cameraMovementStepSize * cVector3d(0.0, 0.0, -1.0));
        }
        else {
            keyInputCursorPosition += cameraMovementStepSize * cVector3d(0.0, 0.0, -1.0);
        }
    }

    else if (a_key == GLFW_KEY_UP) {
        if (!teleoperationMode) {
            simEnvironment->rotateCamera(0.0, cameraRotationStepSize);
        }
    }

    else if (a_key == GLFW_KEY_DOWN) {
        if (!teleoperationMode) {
            simEnvironment->rotateCamera(0.0, -cameraRotationStepSize);
        }
    }

    else if (a_key == GLFW_KEY_LEFT) {
        if (!teleoperationMode) {
            simEnvironment->rotateCamera(-cameraRotationStepSize, 0.0);
        }
    }

    else if (a_key == GLFW_KEY_RIGHT) {
        if (!teleoperationMode) {
            simEnvironment->rotateCamera(cameraRotationStepSize, 0.0);
        }
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete simEnvironment;
    delete handler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz / " + cStr(teleoperationMode));

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////
    simEnvironment->updateShadowMaps(false, mirroredDisplay);
    simEnvironment->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // start haptic device
    hapticDevice->open();

    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);

    cMatrix3d prevRotTool;
    prevRotTool.identity();

    // main haptic simulation loop
    while (simulationRunning)
    {

        // retrieve simulation time and compute next interval
        double time = simClock.getCurrentTimeSeconds();
        //double nextSimInterval = 0.001;//
        double nextSimInterval = time;
        if (time < 0.01) {
            continue;
        }

        // reset clock
        simClock.reset();
        simClock.start();

        // signal frequency counter
        freqCounterHaptics.signal(1);

        // compute global reference frames for each object
        simEnvironment->computeGlobalPositions(true);

        // update position and orientation of tool
        cVector3d posDevice;
        cMatrix3d rotDevice;
        if (hapticDevice->getPosition(posDevice) && hapticDevice->getRotation(rotDevice)) {
            posDevice.mul(workspaceScaleFactor);           
            simEnvironment->updateUserCommand(posDevice, rotDevice);

            // send forces to device
            //hapticDevice->setForceAndTorqueAndGripperForce(force, torque, 0.0);
        }
        else {
            //printf("test\n");
            cVector3d sensorForce, sensorTorque;
            simEnvironment->getForceTorqueSensorValue(sensorForce, sensorTorque);
            simEnvironment->updateUserCommand(keyInputCursorPosition, keyInputCursorRotation);
            //cout << sensorForce(0) << "\t" << sensorForce(1) << "\t" << sensorForce(2) << endl;
            //cout << sensorTorque(0) << "\t" << sensorTorque(1) << "\t" << sensorTorque(2) << endl;
        }

        // update simulation
        simEnvironment->updateDynamics(nextSimInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}