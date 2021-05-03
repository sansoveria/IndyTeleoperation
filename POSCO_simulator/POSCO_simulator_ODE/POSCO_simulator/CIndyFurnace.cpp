#pragma once
#ifdef USE_INDY
#include "CIndyFurnace.h"
#include "example.hpp"          // Include short list of convenience functions for rendering

#ifdef WIN64
#   pragma comment( lib, "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64/realsense2.lib" )
#else
#   pragma comment( lib, "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x86/realsense2.lib" )
#endif


using namespace std;

cIndyFurnace::cIndyFurnace() {
    // set world and camera
    _setWorld();

    // setup devices
    _setIndy();
    _setRealsense();
    _createCursor();
}

cIndyFurnace::~cIndyFurnace() {
    _indyTCP.disconnect();
    _customTCP.disconnect();

    delete _realSensePipeline;
    delete _pointCloud;
    delete _renderingWorld;       // deconstructor of cODEWorld deletes every child objects
}

void cIndyFurnace::_setWorld() {
    _renderingWorld = new cWorld();
    _renderingWorld->setShowFrame(true);
    // set the background color of the environment
    _renderingWorld->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    _camera = new cCamera(_renderingWorld);
    _renderingWorld->addChild(_camera);

    // position and oriente the camera
    _cameraEye = cVector3d(1.55, 1.4, 1.0);
    _cameraTarget = cVector3d(-1.1, 0.54, 0.0);
    _camera->set(_cameraEye, _cameraTarget, cVector3d(0.0, 0.0, 1.0));

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    _camera->setClippingPlanes(0.01, 100.0);
    _camera->setUseMultipassTransparency(true);
    _camera->setStereoEyeSeparation(0.005);
    _camera->setStereoFocalLength(0.7);

    // create a directional light
    cDirectionalLight* _light = new cDirectionalLight(_renderingWorld);
    _renderingWorld->addChild(_light);                        // attach light to camera
    _light->setEnabled(true);                               // enable light source
    _light->setDir(0.0, 0.0, -1.0);                         // define the direction of the light beam
    _light->m_ambient.set(1.0, 1.0, 1.0);
    _light->m_diffuse.set(1.0, 1.0, 1.0);
    _light->m_specular.set(0.1, 0.1, 0.1);
}

void cIndyFurnace::_setIndy() {
    _indyTCP.connect();
    _customTCP.connect();

    _indyTCP.ResetFTBias();

    _indyCommand.setParam(0.002, 5, rotateZ(M_PI), cMatrix3d(0, 0, 1, 0, 1, 0, -1, 0, 0));

    _indy = new cMesh();
    _renderingWorld->addChild(_indy);
    cCreateSphere(_indy, 0.01);

    _indy->m_material->setRedCrimson();
    _indy->setShowFrame(true);
    //_indy->setFrameSize(0.05, true);
}

void cIndyFurnace::_setRealsense() {
    _realSensePipeline = new rs2::pipeline();
    _realSensePipeline->start();

    _pointCloud = new cMultiPoint();
    //_pointCloud->m_texture = cTexture2d::create();
    //_pointCloudTextureMap = cImage::create();
    //_renderingWorld->addChild(_pointCloud);
    _indy->addChild(_pointCloud);

    //_pointCloud->setShowFrame(true);
    _pointCloud->rotateAboutGlobalAxisDeg(0, 1, 0, -90);
    _pointCloud->rotateAboutGlobalAxisDeg(1, 0, 0, -90);
    _pointCloud->rotateAboutGlobalAxisDeg(0, 0, 1, 180);
    _pointCloud->setLocalPos(0.0, 0.0, 0.1);

    _monitor2D = new cBitmap();
    _camera->m_frontLayer->addChild(_monitor2D);
}

void cIndyFurnace::_createCursor() {
    _cursor = new cMesh();
    _renderingWorld->addChild(_cursor);

    cCreateSphere(_cursor, 0.01);
  
    _cursor->m_material->setRedCrimson();
    _cursor->setShowFrame(true);
    //_cursor->setFrameSize(0.05, true);
}

void cIndyFurnace::setEndEffectorPose(cVector3d position, cMatrix3d rotation) {
    // TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
    // move Indy to given pose (TaskMoveTo)
    // TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO

    _cursor->setLocalPos(position);
    _cursor->setLocalRot(rotation);
}

void cIndyFurnace::getEndEffectorPose(cVector3d& position, cMatrix3d& rotation) {
    _indy->getLocalPos().copyto(position);
    _indy->getLocalRot().copyto(rotation);
}

void cIndyFurnace::updateUserCommand(cVector3d posCommand, cMatrix3d rotCommand, bool activateCommand) {
    double commandPose[6], commandVel[6], indyPose[6], indyForceTorque[6];
    double passivityPort = 0.0;
    int cmode;
    double damping;
    
    _indyCommand.updateCommand(posCommand, rotCommand, 0.002, false);

    _indyCommand.getPoseCommand(commandPose);
    _indyCommand.getVelocityCommand(commandVel);
    //for (int i = 0; i < 6; i++) {
    //    commandVel[i] = 0.0;
    //}

    _customTCP.SendIndyCommandAndReadState(commandPose, commandVel, passivityPort, indyPose, indyForceTorque, cmode, damping);

    double axis1, axis2, axis3, angle;
    cMatrix3d rotIndy;
    angle = sqrt(indyPose[3] * indyPose[3] + indyPose[4] * indyPose[4] + indyPose[5] * indyPose[5]);
    axis1 = indyPose[3];
    axis2 = indyPose[4];
    axis3 = indyPose[5];
    if (angle > 0) {
        rotIndy.setAxisAngleRotationRad(cVector3d(axis1, axis2, axis3), angle);
    }
    else {
        rotIndy.set(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    }

    _sensorForce = cMul(cTranspose(_indyCommand.getGlobalTransformation()), cVector3d(indyForceTorque[0], indyForceTorque[1], indyForceTorque[2]));
    _sensorTorque = cMul(rotateZ(-M_PI / 2.0), cMul(rotateX(M_PI / 2.0), cVector3d(indyForceTorque[3], indyForceTorque[4], indyForceTorque[5])));

    _indyCmode = cmode;

    _indy->setLocalPos(cMul(cTranspose(_indyCommand.getGlobalTransformation()), cVector3d(indyPose[0], indyPose[1], indyPose[2])));
    _indy->setLocalRot(cMul(cTranspose(_indyCommand.getGlobalTransformation()), rotIndy * cTranspose(_indyCommand.getLocalTransformation())));
    _cursor->setLocalPos(posCommand);
    _cursor->setLocalRot(rotCommand);
}

bool cIndyFurnace::updateShadowMaps(const bool mirror_x, const bool mirror_y) {
    return _renderingWorld->updateShadowMaps(mirror_x, mirror_y);
}

void cIndyFurnace::renderView(const int a_windowWidth, const int a_windowHeight, const cEyeMode a_eyeMode, const bool a_defaultBuffer) {
    auto frames = _realSensePipeline->wait_for_frames();

    auto color = frames.get_color_frame();
    if (!color) color = frames.get_infrared_frame();

    if (_renderPointCloud) {
        _realSensePointCloudConverter.map_to(color);

        auto depth = frames.get_depth_frame();
        _points = _realSensePointCloudConverter.calculate(depth);
        const rs2::vertex* vertex = _points.get_vertices();
        const rs2::texture_coordinate* texCoord = _points.get_texture_coordinates();
        const unsigned char* colorStream = static_cast<const unsigned char*>(color.get_data());
        int width = color.get_width();
        int height = color.get_height();
        int widthDepth = depth.get_width();
        int heightDepth = depth.get_height();

        //_pointCloudTextureMap->setSize(width, height);
        //void* textureImageData = malloc(color.get_data_size());
        //memcpy(textureImageData, color.get_data(), color.get_data_size());
        //_pointCloudTextureMap->setData(static_cast<unsigned char*>(textureImageData), color.get_data_size());
        //cout << color.get_data_size() << endl;

        _pointCloud->clear();
        int pointSampling = 3;
        _pointCloud->setUseVertexColors(true, true);
        _pointCloud->setPointSize(pointSampling);

        //_pointCloud->m_texture->setImage(_pointCloudTextureMap);
        //_pointCloud->setUseTexture(true, true);
        //_pointCloud->setUseMaterial(true, true);
        //_pointCloud->m_material->setWhite();
        for (int xIdx = 0; xIdx < widthDepth / pointSampling; xIdx++) {
            for (int yIdx = 0; yIdx < heightDepth / pointSampling; yIdx++) {
                int pointIdx = yIdx * pointSampling * widthDepth + xIdx * pointSampling;
                int xColorMap = texCoord[pointIdx].u * width;
                int yColorMap = texCoord[pointIdx].v * height;

                if (yColorMap <= 0 || yColorMap >= height) continue;
                if (xColorMap <= 0 || xColorMap >= width) continue;

                int colorLocation = 3 * (yColorMap * width + xColorMap);
                unsigned char pointColor[3] = { colorStream[colorLocation], colorStream[colorLocation + 1], colorStream[colorLocation + 2] };
                unsigned int vertexIdx = _pointCloud->newPoint(cVector3d(vertex[pointIdx].x, vertex[pointIdx].y, vertex[pointIdx].z), cColorf(pointColor[0] / 255.0, pointColor[1] / 255.0, pointColor[2] / 255.0, 1));
                //cColorb tmp;
                //_pointCloudTextureMap->getPixelColor(xColorMap, yColorMap, tmp);
                //unsigned int vertexIdx = _pointCloud->newPoint(cVector3d(vertex[pointIdx].x, vertex[pointIdx].y, vertex[pointIdx].z));
                //_pointCloud->m_vertices->setTexCoord(vertexIdx, cVector3d(texCoord[pointIdx].u, texCoord[pointIdx].v, 0.0));
            }
        }
        _pointCloud->m_points->m_vertices = _pointCloud->m_vertices;
        //free(textureImageData);
    }
    else {
        cImagePtr image2D = cImage::create();
        int width = color.get_width();
        int height = color.get_height();
        image2D->setSize(width, height);
        memcpy(image2D->getData(), color.get_data(), color.get_data_size());
        image2D->flipHorizontal();
        _monitor2D->loadFromImage(image2D);
        _monitor2D->setSize(a_windowWidth, a_windowHeight);
        _monitor2D->setZoom(float(a_windowWidth) / float(width), float(a_windowHeight) / float(height));
    }

    _camera->renderView(a_windowWidth, a_windowHeight, a_eyeMode, a_defaultBuffer);
}


void cIndyFurnace::computeGlobalPositions(const bool frameOnly, const cVector3d& globalPos, const cMatrix3d& globalRot) {
    _renderingWorld->computeGlobalPositions(frameOnly, globalPos, globalRot);
}

void cIndyFurnace::updateDynamics(double interval) {
    // Do nothing in this class.
    // If feedback force is calculated, update this function.
}

void cIndyFurnace::addVisualComponent(cGenericObject* object) {
    _camera->m_frontLayer->addChild(object);
}

void cIndyFurnace::getCameraPose(cVector3d& eye, cVector3d& target) {
    eye.copyfrom(_cameraEye);
    target.copyfrom(_cameraTarget);
}

void cIndyFurnace::updateCameraPose(cVector3d eye, cVector3d target) {
    _cameraEye.copyfrom(eye);
    _cameraTarget.copyfrom(target);
    _camera->set(eye, target, cVector3d(0.0, 0.0, 1.0));
}

void cIndyFurnace::moveCamera(cVector3d step) {
    cMatrix3d viewMatrix = rotationMatrixFromVectors(_cameraTarget - _cameraEye, cVector3d(0.0, 0.0, 1.0));
    _cameraEye += cMul(viewMatrix, step);
    _cameraTarget += cMul(viewMatrix, step);
    _camera->set(_cameraEye, _cameraTarget, cVector3d(0.0, 0.0, 1.0));
    cout << _cameraEye << ", " << _cameraTarget << endl;
}

void cIndyFurnace::rotateCamera(double yawStepInDeg, double pitchStepInDeg) {
    cMatrix3d viewMatrix = rotationMatrixFromVectors(_cameraTarget - _cameraEye, cVector3d(0.0, 0.0, 1.0));
    cMatrix3d yawMatrix, pitchMatrix;
    yawMatrix.setAxisAngleRotationDeg(cMul(viewMatrix, cVector3d(0.0, 0.0, 1.0)), yawStepInDeg);
    pitchMatrix.setAxisAngleRotationDeg(cMul(viewMatrix, cVector3d(0.0, 1.0, 0.0)), pitchStepInDeg);

    cVector3d dirVector = _cameraEye - _cameraTarget;
    cVector3d resVector = cMul(pitchMatrix, cMul(yawMatrix, dirVector));
    cVector3d resVectorNormalized;
    resVector.normalizer(resVectorNormalized);
    if (resVectorNormalized.dot(cVector3d(0, 0, 1)) < 0.99) {
        _cameraEye = resVector + _cameraTarget;
        _camera->set(_cameraEye, _cameraTarget, cVector3d(0.0, 0.0, 1.0));
    }

    cout << _cameraEye << ", " << _cameraTarget << endl;
}

void cIndyFurnace::switchCameraMode() {
    if (_renderPointCloud) {
        _renderPointCloud = false;
        _monitor2D->setShowPanel(true);
    }
    else {
        _renderPointCloud = true;
        _monitor2D->setShowPanel(false);
    }
}

void cIndyFurnace::setEndEffectorStiffness(double posStiffness_, double rotStiffness_) {
    _linStiffness = posStiffness_;
    _angStiffness = rotStiffness_;
}

void cIndyFurnace::setEndEffectorDamping(double posDamping_, double rotDamping_) {
    _linDamping = posDamping_;
    _angDamping = rotDamping_;
}


void cIndyFurnace::getForceTorqueSensorValue(cVector3d& sensorForce, cVector3d& sensorTorque) {
    _sensorForce.copyto(sensorForce);
    _sensorTorque.copyto(sensorTorque);
}

int cIndyFurnace::getRobotControlMode() {
    return _indyCmode;
}

void cIndyFurnace::moveRobotHome() {
    //_indyTCP.GoHome();
    _indyTCP.ResetRobot();
    while (!_indyTCP.isReady()) {
        Sleep(1000);
    }
    _indyTCP.ResetFTBias();
    _indyTCP.MoveToJ(_defaultPose);
}

void cIndyFurnace::moveRobotZero() {
    _indyTCP.GoZero();
}

void cIndyFurnace::toggleDirectTeachingMode() {
    int cmode = _indyTCP.GetCMode();
    if (cmode == 0) {
        _indyTCP.StartDirectTeaching();
    }
    if (cmode == 3) {
        _indyTCP.StopDirectTeaching();
    }
}

void cIndyFurnace::startTeleoperationMode() {
    printf("start teleoperation mode\n");
    int cmode = _indyTCP.GetCMode();

    if (cmode == 0) {
        _indyCommand.initFilters();
        _indyTCP.ToggleTeleoperationMode();
    }
}

void cIndyFurnace::quitTeleoperationMode() {
    printf("quit teleoperation mode\n");
    int cmode = _indyTCP.GetCMode();

    if (cmode == 20) {
        _indyTCP.ToggleTeleoperationMode();
    }
}

void cIndyFurnace::test() {
    double dist[6] = { 0.2,0,0,0,0,0 };
    _indyTCP.MoveByT(dist);
}

#endif