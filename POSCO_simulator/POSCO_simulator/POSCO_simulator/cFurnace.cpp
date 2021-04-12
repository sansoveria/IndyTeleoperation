#pragma once

#include "cFurnace.h"

using namespace std;

cFurnace::cFurnace() {
    // set world and camera
    _setWorld();    

    // setup static objects
    _createStaticObjects();

    // setup static objects
    _createEndEffector();
}

cFurnace::~cFurnace() {
    delete _endEffectorForceTorqueSensor;
    _furnaceWorld->m_bulletWorld->removeConstraint(_endEffectorJoint);
    delete _endEffectorJoint;
    delete _furnaceWorld;       // deconstructor of cBulletWorld deletes every child objects
}

void cFurnace::_setWorld() {
    // set chai3D world
    _furnaceWorld = new cCustomBulletWorld();
    _furnaceWorld->setTimeStep(0.001);
    _furnaceWorld->setMaxIteration(100);

    // set the background color of the environment
    _furnaceWorld->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    _camera = new cCamera(_furnaceWorld);
    _furnaceWorld->addChild(_camera);

    // position and oriente the camera
    _cameraEye = cVector3d(6.0, 0.0, 4.7);
    _cameraTarget = cVector3d(1.5, -0.8, 1.6);
    _camera->set(_cameraEye, _cameraTarget, cVector3d(0.0, 0.0, 1.0));

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    _camera->setClippingPlanes(0.01, 100.0);
    _camera->setUseMultipassTransparency(true);
    _camera->setStereoEyeSeparation(0.005);
    _camera->setStereoFocalLength(0.7);

    // create a directional light
    cDirectionalLight* _light = new cDirectionalLight(_furnaceWorld);
    _furnaceWorld->addChild(_light);                        // attach light to camera
    _light->setEnabled(true);                               // enable light source
    _light->setDir(0.0, 0.0, -1.0);                         // define the direction of the light beam
    _light->m_ambient.set(0.15, 0.15, 0.15);
    _light->m_diffuse.set(0.4, 0.4, 0.4);
    _light->m_specular.set(0.1, 0.1, 0.1);

    // set world gravity
    _furnaceWorld->setGravity(cVector3d(0.0, 0.0, -9.8));
}


void cFurnace::_createStaticObjects() {
    // define objects
    _wall = new cBulletMultiMesh(_furnaceWorld);
    _splashCover = new cBulletMultiMesh(_furnaceWorld);
    _outlet = new cBulletMultiMesh(_furnaceWorld);
    _fence = new cBulletMultiMesh(_furnaceWorld);
    _pouringIron = new cBulletMultiMesh(_furnaceWorld);
    //_origin = new cBulletSphere(_furnaceWorld, 0.05);
    _cursor = new cBulletSphere(_furnaceWorld, 0.01);

    _furnaceWorld->addChild(_wall);
    _furnaceWorld->addChild(_splashCover);
    _furnaceWorld->addChild(_outlet);
    _furnaceWorld->addChild(_fence);
    _furnaceWorld->addChild(_pouringIron);
    //_furnaceWorld->addChild(_origin);
    _furnaceWorld->addChild(_cursor);

    // load meshes
    if (!_wall->loadFromFile("../Resources/POSCO/outlet.STL"))
        printf("[Load wall] 3D Model is failed to load correctly.\n");

    if (!_splashCover->loadFromFile("../Resources/POSCO/cover_cutting_v2.STL")) 
        printf("[Load splash cover] 3D Model is failed to load correctly.\n");

    if (!_outlet->loadFromFile("../Resources/POSCO/outlet_part.STL"))
        printf("[Load outlet] 3D Model is failed to load correctly.\n");
    
    if (!_fence->loadFromFile("../Resources/POSCO/support.STL")) 
        printf("[Load fence] 3D Model is failed to load correctly.\n");
    
    if (!_pouringIron->loadFromFile("../Resources/POSCO/fluid.STL")) 
        printf("[Load splash cover] 3D Model is failed to load correctly.\n");


    // set textures 
    vector<cMesh*>::iterator it;
    for (it = _wall->m_meshes->begin(); it < _wall->m_meshes->end(); it++) {
        setTextureMapFromFile(*it, "../Resources/POSCO/sand4.jpg", 10.0);
    }

    for (it = _splashCover->m_meshes->begin(); it < _splashCover->m_meshes->end(); it++) {
        setTextureMapFromFile(*it, "../Resources/POSCO/steel5.jpg", 10.0);
    }

    for (it = _outlet->m_meshes->begin(); it < _outlet->m_meshes->end(); it++) {
        (*it)->m_material->setBrownBurlyWood();
    }

    for (it = _fence->m_meshes->begin(); it < _fence->m_meshes->end(); it++) {
        setTextureMapFromFile(*it, "../Resources/POSCO/steel5.jpg", 10.0);
        //(*it)->m_material->m_diffuse.setb(0x19, 0x19, 0x70);
        //(*it)->m_material->m_ambient.setb(0x19, 0x19, 0x70);
        //(*it)->m_material->m_emission.setb(0x19/6, 0x19/6, 0x70/6);
    }

    for (it = _pouringIron->m_meshes->begin(); it < _pouringIron->m_meshes->end(); it++) {
        (*it)->m_material->m_ambient = cColorf(1.0, 0.2, 0.0, 1.0);
        (*it)->m_material->m_diffuse = cColorf(1.0, 0.6, 0.0, 1.0);
        (*it)->m_material->m_emission = cColorf(1.0, 0.6, 0.0, 1.0);
        (*it)->m_material->m_specular = cColorf(1.0, 0.6, 0.0, 1.0);
        (*it)->m_material->setShininess(128);
    }

    _lightIron1 = new cSpotLight(_furnaceWorld);
    _furnaceWorld->addChild(_lightIron1);                       // attach light to camera
    _lightIron1->setEnabled(true);                              // enable light source
    _lightIron1->setLocalPos(0.0, -2.0, 1.8);                   // position the light source
    _lightIron1->m_ambient.set(0.0, 0.0, 0.0);
    _lightIron1->m_diffuse.set(1.0, 0.25, 0.0);
    _lightIron1->m_specular.set(0.8, 0.2, 0.0);

    _lightIron1->setDir(0.0, 0.0, 1.0);
    _lightIron1->setShadowMapEnabled(true);
    _lightIron1->setCutOffAngleDeg(45);
    //_lightIron1->setDisplaySettings();
    _lightIron1->setAttLinear(1.0);
    _lightIron1->setAttQuadratic(1.0);
    cout << _lightIron1->getAttConstant() << ", " << _lightIron1->getAttLinear() << ", " << _lightIron1->getAttQuadratic() << endl;
        
    _lightIron2 = new cSpotLight(_furnaceWorld);
    _furnaceWorld->addChild(_lightIron2);                       // attach light to camera
    _lightIron2->setEnabled(true);                              // enable light source
    _lightIron2->setLocalPos(0.0, -2.0, 1.5);                   // position the light source
    _lightIron2->m_ambient.set(0.0, 0.00, 0.0);
    _lightIron2->m_diffuse.set(1.0, 0.25, 0.0);
    _lightIron2->m_specular.set(0.8, 0.2, 0.0);

    _lightIron2->setDir(-1.0, 0.0, -1.0);
    _lightIron2->setShadowMapEnabled(true);
    _lightIron2->setCutOffAngleDeg(45);
    //_lightIron2->setDisplaySettings();
    _lightIron2->setAttLinear(1.0);
    _lightIron2->setAttQuadratic(1.0);
    cout << _lightIron2->getAttConstant() << ", " << _lightIron2->getAttLinear() << ", " << _lightIron2->getAttQuadratic() << endl;

    _lightIron3 = new cSpotLight(_furnaceWorld);
    _furnaceWorld->addChild(_lightIron3);                       // attach light to camera
    _lightIron3->setEnabled(true);                              // enable light source
    _lightIron3->setLocalPos(0.0, -1.6, 1.8);                   // position the light source
    _lightIron3->m_ambient.set(0.0, 0.00, 0.0);
    _lightIron3->m_diffuse.set(1.0, 0.25, 0.0);
    _lightIron3->m_specular.set(0.8, 0.2, 0.0);

    _lightIron3->setDir(1.0, 0.0, 0.0);
    _lightIron3->setShadowMapEnabled(true);
    _lightIron3->setCutOffAngleDeg(45);
    //_lightIron3->setDisplaySettings();
    _lightIron3->setAttLinear(1.0);
    _lightIron3->setAttQuadratic(1.0);
    cout << _lightIron3->getAttConstant() << ", " << _lightIron3->getAttLinear() << ", " << _lightIron3->getAttQuadratic() << endl;

    _lightIron4 = new cSpotLight(_furnaceWorld);
    _furnaceWorld->addChild(_lightIron4);                       // attach light to camera
    _lightIron4->setEnabled(true);                              // enable light source
    _lightIron4->setLocalPos(0.0, -1.6, 1.5);                   // position the light source
    _lightIron4->m_ambient.set(0.0, 0.0, 0.0);
    _lightIron4->m_diffuse.set(1.0, 0.25, 0.0);
    _lightIron4->m_specular.set(0.8, 0.2, 0.0);

    _lightIron4->setDir(1.0, 0.0, 0.0);
    _lightIron4->setShadowMapEnabled(true);
    _lightIron4->setCutOffAngleDeg(45);
    //_lightIron4->setDisplaySettings();
    _lightIron4->setAttLinear(1.0);
    _lightIron4->setAttQuadratic(1.0);
    cout << _lightIron4->getAttConstant() << ", " << _lightIron4->getAttLinear() << ", " << _lightIron4->getAttQuadratic() << endl;

    //_origin->m_material->setShininess(128);
    //_origin->m_material->m_ambient.set(1.0, 1.0, 1.0);
    //_origin->m_material->m_diffuse.set(1.0, 1.0, 1.0);
    //_origin->m_material->m_specular.set(1.0, 1.0, 1.0);
    //_origin->m_material->m_emission.set(1.0, 1.0, 1.0);

    _cursor->m_material->setShininess(128);
    _cursor->m_material->m_ambient.set(1.0, 1.0, 1.0);
    _cursor->m_material->m_diffuse.set(1.0, 1.0, 1.0);
    _cursor->m_material->m_specular.set(1.0, 1.0, 1.0);
    _cursor->m_material->m_emission.set(1.0, 1.0, 1.0);
    _cursor->setShowFrame(true);
    

    // set scale
    _wall->scale(0.001);
    _splashCover->scale(0.001);
    _outlet->scale(0.001);
    _fence->scale(0.001);
    _pouringIron->scale(0.001);
    

    // set collision detection margin
    _wall->buildContactTriangles(_contactTriangleSize);
    _splashCover->buildContactTriangles(_contactTriangleSize);
    _outlet->buildContactTriangles(_contactTriangleSize);
    _fence->buildContactTriangles(_contactTriangleSize);
    _pouringIron->buildContactTriangles(_contactTriangleSize);


    // build dynamic model
    _wall->buildDynamicModel();
    _splashCover->buildDynamicModel();
    _outlet->buildDynamicModel();
    _fence->buildDynamicModel();
    _pouringIron->buildDynamicModel();

    _wall->setStatic(true);
    _splashCover->setStatic(true);
    _outlet->setStatic(true);
    _fence->setStatic(true);
    _pouringIron->setStatic(true);


    // build dynamic model
    _wall->rotateAboutGlobalAxisDeg(1, 0, 0, 90);
    _wall->rotateAboutGlobalAxisDeg(0, 0, 1, 180);
    _wall->setLocalPos(4.0, -3.0, 0.0);

    _splashCover->rotateAboutGlobalAxisDeg(1, 0, 0, 90);
    _splashCover->setLocalPos(-1.8, 4.5, 1.25);

    _outlet->rotateAboutGlobalAxisDeg(1, 0, 0, -90);
    _outlet->setLocalPos(-0.6, -2.7, 2.05);

    _fence->rotateAboutGlobalAxisDeg(1, 0, 0, 90);
    _fence->rotateAboutGlobalAxisDeg(0, 0, 1, -90);
    _fence->setLocalPos(1.2, -1.6, 1.3);

    _pouringIron->rotateAboutGlobalAxisDeg(1, 0, 0, 90);
    _pouringIron->setLocalPos(-0.165, 0.73, 0.845);

    //_origin->setLocalPos(0.0, 0.0, 3.0);
    //_origin->setLocalPos(0.0, 0.0, 3.0);

}

void cFurnace::_createEndEffector() {
    _endEffectorCursor = new cBulletCylinder(_furnaceWorld, 0.1, 0.04);
    _endEffectorTool = new cBulletCylinder(_furnaceWorld, 2.0, 0.01);

    _furnaceWorld->addChild(_endEffectorCursor);
    _furnaceWorld->addChild(_endEffectorTool);

    _endEffectorTool->buildContactTriangles(_contactTriangleSize);

    _endEffectorTool->setMass(0.5);
    _endEffectorTool->estimateInertia();

    _endEffectorCursor->buildDynamicModel();
    _endEffectorCursor->setStatic(true);
    _endEffectorTool->buildDynamicModel();

    // add constraint
    btTransform frameInCursor(btMatrix3x3::getIdentity(),
        btVector3(0.0, 0.0, _endEffectorCursor->getHeight() / 2.0));
    btTransform frameInTool(btMatrix3x3::getIdentity(),
        btVector3(0.0, 0.0, -_endEffectorTool->getHeight() / 2.0));
    _endEffectorJoint = new btFixedConstraint(
        *_endEffectorCursor->m_bulletRigidBody,
        *_endEffectorTool->m_bulletRigidBody,
        frameInCursor,
        frameInTool
    );

    _furnaceWorld->m_bulletWorld->addConstraint(_endEffectorJoint, true);
    for (int i = 0; i < 3; i++) {
        _endEffectorJoint->enableSpring(i, true);
        _endEffectorJoint->setStiffness(i, 10, true);
        _endEffectorJoint->setDamping(i, 1, true);
        _endEffectorJoint->enableSpring(i+3, true);
        _endEffectorJoint->setStiffness(i+3, 10, true);
        _endEffectorJoint->setDamping(i+3, 10, true);
    }
    _endEffectorForceTorqueSensor = new btJointFeedback();
    _endEffectorJoint->setJointFeedback(_endEffectorForceTorqueSensor);

    _endEffectorCursor->setLocalPos(1.5, 0.2, 2.5);
    _endEffectorCursor->rotateAboutGlobalAxisDeg(0, 0, 1, 90);
    _endEffectorTool->setLocalPos(1.5, 0.2, 2.5 + (_endEffectorCursor->getHeight()+ _endEffectorTool->getHeight())/2.0);
    _endEffectorTool->rotateAboutGlobalAxisDeg(0, 0, 1, 90);

    _endEffectorCursor->setDamping(0.8, 0.8);
    _endEffectorTool->setDamping(0.8, 0.8);
    _linStiffness = 100.0;
    _angStiffness = 3.0;
    _linDamping = 0.0;
    _angDamping = 0.0;
}

void cFurnace::setEndEffectorPose(cVector3d position, cMatrix3d rotation) {
    _endEffectorCursor->setLocalPos(position);
    _endEffectorCursor->setLocalRot(rotation);
    cVector3d toolOffset = cMul(rotation, cVector3d(0, 0, (_endEffectorCursor->getHeight() + _endEffectorTool->getHeight()) / 2.0));
    _endEffectorTool->setLocalPos(position + toolOffset);
    _endEffectorTool->setLocalRot(rotation);
}


bool cFurnace::updateShadowMaps(const bool mirror_x, const bool mirror_y) {
    return _furnaceWorld->updateShadowMaps(mirror_x, mirror_y);
}

void cFurnace::renderView(const int a_windowWidth, const int a_windowHeight, const cEyeMode a_eyeMode, const bool a_defaultBuffer) {
    _camera->renderView(a_windowWidth, a_windowHeight, a_eyeMode, a_defaultBuffer);
}

void cFurnace::computeGlobalPositions(const bool frameOnly, const cVector3d& globalPos, const cMatrix3d& globalRot) {
    _furnaceWorld->computeGlobalPositions(frameOnly, globalPos, globalRot);
}

void cFurnace::updateDynamics(double interval) {
    _furnaceWorld->updateDynamics(interval);
}

void cFurnace::addVisualComponent(cGenericObject* object) {
    _camera->m_frontLayer->addChild(object);
}

void cFurnace::updateUserCommand(cVector3d posCommand, cMatrix3d rotCommand) {
    //cVector3d posTool = _endEffectorCursor->getLocalPos();
    //cMatrix3d rotTool = _endEffectorCursor->getLocalRot();

    //// compute position and angular error between tool and haptic device
    //cVector3d deltaPos = (posCommand - posTool);
    //cMatrix3d deltaRot = cMul(cTranspose(rotTool), rotCommand);
    //btVector3 linVelocity = _endEffectorCursor->m_bulletRigidBody->getLinearVelocity();
    //btVector3 angVelocity = _endEffectorCursor->m_bulletRigidBody->getAngularVelocity();
    //double angle;
    //cVector3d axis;
    //deltaRot.toAxisAngle(axis, angle);

    //// compute force and torque to apply to tool
    //cVector3d force, torque;
    //force = _linStiffness * deltaPos - _linDamping * cVector3d(linVelocity[0], linVelocity[1], linVelocity[2]);
    //_endEffectorCursor->addExternalForce(force);

    //torque = cMul((_angStiffness * angle), axis) - _angDamping * cVector3d(angVelocity[0], angVelocity[1], angVelocity[2]);
    //rotTool.mul(torque);
    //_endEffectorCursor->addExternalTorque(torque);

    _endEffectorCursor->setLocalPos(posCommand);
    _endEffectorCursor->setLocalRot(rotCommand);

    _cursor->setLocalPos(posCommand);
    _cursor->setLocalRot(rotCommand);

    // gravity compensation
    btVector3 gravity;
    gravity = _furnaceWorld->m_bulletWorld->getGravity();
    _endEffectorCursor->addExternalForce(_endEffectorCursor->getMass() * cVector3d(-gravity[0], -gravity[1], -gravity[2]));
    _endEffectorTool->addExternalForce(_endEffectorTool->getMass() * cVector3d(-gravity[0], -gravity[1], -gravity[2]));

    //printf("%f \n", _endEffectorCursor->m_bulletRigidBody->getLinearVelocity()[0]);
}

void cFurnace::getCameraPose(cVector3d& eye, cVector3d& target) {
    eye.copyfrom(_cameraEye);
    target.copyfrom(_cameraTarget);
}

void cFurnace::updateCameraPose(cVector3d eye, cVector3d target) {
    _cameraEye.copyfrom(eye);
    _cameraTarget.copyfrom(target);
    _camera->set(eye, target, cVector3d(0.0, 0.0, 1.0));     
}

void cFurnace::moveCamera(cVector3d step) {
    cMatrix3d viewMatrix = rotationMatrixFromVectors(_cameraTarget-_cameraEye, cVector3d(0.0, 0.0, 1.0));
    _cameraEye += cMul(viewMatrix, step);
    _cameraTarget += cMul(viewMatrix, step);
    _camera->set(_cameraEye, _cameraTarget, cVector3d(0.0, 0.0, 1.0));
    cout << _cameraEye << ", " << _cameraTarget << endl;
}

void cFurnace::rotateCamera(double yawStepInDeg, double pitchStepInDeg) {
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

void cFurnace::setEndEffectorStiffness(double posStiffness_, double rotStiffness_){
    _linStiffness = posStiffness_;
    _angStiffness = rotStiffness_;
}

void cFurnace::setEndEffectorDamping(double posDamping_, double rotDamping_) {
    _linDamping = posDamping_;
    _angDamping = rotDamping_;
}


void cFurnace::getForceTorqueSensorValue(cVector3d& sensorForce, cVector3d& sensorTorque) {
    for (int i = 0; i < 3; i++) {
        sensorForce(i) = _endEffectorForceTorqueSensor->m_appliedForceBodyA[i];
        sensorTorque(i) = _endEffectorForceTorqueSensor->m_appliedTorqueBodyA[i];
    }
}

