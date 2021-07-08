#pragma once
#include "CIndyFurnace.h"

using namespace std;

cIndyFurnace::cIndyFurnace() {
    // set world and camera
    _setWorld();

    // setup devices
    _setIndy();
    _createCursor();
}

cIndyFurnace::~cIndyFurnace() {
    _customTCP.disconnect();
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
    _customTCP.connect();

    _customTCP.reset_ft_bias();
    _indyCommand.setParam(0.002, 5, rotateZ(M_PI), cMatrix3d(0, 0, 1, 0, 1, 0, -1, 0, 0));

    _indy = new cMesh();
    _renderingWorld->addChild(_indy);
    cCreateSphere(_indy, 0.01);

    _indy->m_material->setRedCrimson();
    _indy->setShowFrame(true);
    //_indy->setFrameSize(0.05, true);
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
    double commandPose[6], commandVel[6], indyPose[6], indyVelocity[6], indyForceTorque[6];
    double passivityPort = 0.0;
    int cmode;
    double curr_time_constant;
    
    _indyCommand.updateCommand(posCommand, rotCommand, 0.002, false);
    _indyCommand.getPoseCommand(commandPose);

    _customTCP.update_teleoperation_command(commandPose, cmode, curr_time_constant, indyPose, indyVelocity, indyForceTorque);
    if (cmode == -1) {
        // quit process
    }

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
    _customTCP.reset_robot();
    Sleep(3000);
    
    _customTCP.reset_ft_bias();
    _customTCP.joint_move_to(_defaultPose, true);
}

void cIndyFurnace::moveRobotZero() {
    _customTCP.go_zero();
}

void cIndyFurnace::startDirectTeachingMode() {
    _customTCP.start_direct_teaching();
}

void cIndyFurnace::stopDirectTeachingMode() {
    _customTCP.stop_direct_teaching();
}

void cIndyFurnace::startTeleoperationMode() {
    printf("start teleoperation mode\n");
    _customTCP.start_teleoperation();
}

void cIndyFurnace::quitTeleoperationMode() {
    _customTCP.stop_teleoperation();
}

void cIndyFurnace::test() {

}

