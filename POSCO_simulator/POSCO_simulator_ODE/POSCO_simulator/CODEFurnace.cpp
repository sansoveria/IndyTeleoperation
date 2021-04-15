#pragma once

#include "CODEFurnace.h"

using namespace std;

cODEFurnace::cODEFurnace() {
    // set world and camera
    _setWorld();    

    // setup objects
    _createStaticObjects();
    _createDynamicObjects();

    // setup static objects
    _createEndEffector();

    // setup rule
    _createSimulationRule();

    //printf("%p \n", _endEffectorTool->m_ode_body);
    //printf("%p \n", _depositedIron->m_ode_body);
    //printf("%p \n", _wall->m_ode_body);
    //printf("%p \n", _splashCover->m_ode_body);
    //printf("%p \n", _outlet->m_ode_body);
    //printf("%p \n", _fence->m_ode_body);
    //printf("%p \n", _pouringIron->m_ode_body);
}

cODEFurnace::~cODEFurnace() {
    //delete _endEffectorForceTorqueSensor;
    //_furnaceWorld->m_ODEWorld->removeConstraint(_endEffectorJoint);
    //delete _endEffectorJoint;
    delete _renderingWorld;       // deconstructor of cODEWorld deletes every child objects
}

void cODEFurnace::_setWorld() {
    _renderingWorld = new cWorld();

    // set chai3D world
    _furnaceWorld = new cCustomODEWorld(_renderingWorld);
    _renderingWorld->addChild(_furnaceWorld);

    // set the background color of the environment
    _renderingWorld->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    _camera = new cCamera(_renderingWorld);
    _renderingWorld->addChild(_camera);

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
    cDirectionalLight* _light = new cDirectionalLight(_renderingWorld);
    _renderingWorld->addChild(_light);                        // attach light to camera
    _light->setEnabled(true);                               // enable light source
    _light->setDir(0.0, 0.0, -1.0);                         // define the direction of the light beam
    _light->m_ambient.set(0.15, 0.15, 0.15);
    _light->m_diffuse.set(0.4, 0.4, 0.4);
    //_light->m_ambient.set(1.0, 1.0, 1.0);
    //_light->m_diffuse.set(1.0, 1.0, 1.0);
    _light->m_specular.set(0.1, 0.1, 0.1);

    // set world gravity
    _furnaceWorld->setGravity(cVector3d(0.0, 0.0, -9.8));
}


void cODEFurnace::_createStaticObjects() {
    // define objects
    _fence = new cODEGenericBody(_furnaceWorld);
    _wall = new cODEGenericBody(_furnaceWorld);
    _splashCover = new cODEGenericBody(_furnaceWorld);
    _outlet = new cODEGenericBody(_furnaceWorld);
    _pouringIron = new cODEGenericBody(_furnaceWorld);
    _flowingIron = new cODEGenericBody(_furnaceWorld);

    _fenceMesh = new cMultiMesh();
    _wallMesh = new cMultiMesh();
    _splashCoverMesh = new cMultiMesh();
    _outletMesh = new cMultiMesh();
    _pouringIronMesh = new cMultiMesh();
    _flowingIronMesh = new cMesh();


    // load meshes
    if (!_fenceMesh->loadFromFile("../Resources/POSCO/support.STL"))
        printf("[Load fence] 3D Model is failed to load correctly.\n");

    if (!_wallMesh->loadFromFile("../Resources/POSCO/outlet_v2.STL"))
        printf("[Load wall] 3D Model is failed to load correctly.\n");

    if (!_splashCoverMesh->loadFromFile("../Resources/POSCO/cover_cutting_v2.STL"))
        printf("[Load splash cover] 3D Model is failed to load correctly.\n");

    if (!_outletMesh->loadFromFile("../Resources/POSCO/outlet_part.STL"))
        printf("[Load outlet] 3D Model is failed to load correctly.\n");

    if (!_pouringIronMesh->loadFromFile("../Resources/POSCO/fluid.STL"))
        printf("[Load splash cover] 3D Model is failed to load correctly.\n");

    cCreateBox(_flowingIronMesh, 1.8, 8.0, 0.3);


    // set textures 
    vector<cMesh*>::iterator it;
    for (it = _fenceMesh->m_meshes->begin(); it < _fenceMesh->m_meshes->end(); it++) {
        setTextureMapFromFile(*it, "../Resources/POSCO/steel5.jpg", 0.001, 10.0);
        //(*it)->m_material->m_diffuse.setb(0x19, 0x19, 0x70);
        //(*it)->m_material->m_ambient.setb(0x19, 0x19, 0x70);
        //(*it)->m_material->m_emission.setb(0x19/6, 0x19/6, 0x70/6);
    }

    for (it = _wallMesh->m_meshes->begin(); it < _wallMesh->m_meshes->end(); it++) {
        setTextureMapFromFile(*it, "../Resources/POSCO/sand4.jpg", 0.001, 10.0);
    }

    for (it = _splashCoverMesh->m_meshes->begin(); it < _splashCoverMesh->m_meshes->end(); it++) {
        setTextureMapFromFile(*it, "../Resources/POSCO/steel5.jpg", 0.001, 10.0);
    }

    for (it = _outletMesh->m_meshes->begin(); it < _outletMesh->m_meshes->end(); it++) {
        (*it)->m_material->setBrownBurlyWood();
    }

    for (it = _pouringIronMesh->m_meshes->begin(); it < _pouringIronMesh->m_meshes->end(); it++) {
        (*it)->m_material->m_ambient = cColorf(1.0, 0.2, 0.0, 1.0);
        (*it)->m_material->m_diffuse = cColorf(1.0, 0.6, 0.0, 1.0);
        (*it)->m_material->m_emission = cColorf(1.0, 0.6, 0.0, 1.0);
        (*it)->m_material->m_specular = cColorf(1.0, 0.6, 0.0, 1.0);
        (*it)->m_material->setShininess(128);
    }

    _flowingIronMesh->m_material->m_ambient = cColorf(1.0, 0.2, 0.0, 1.0);
    _flowingIronMesh->m_material->m_diffuse = cColorf(1.0, 0.6, 0.0, 1.0);
    _flowingIronMesh->m_material->m_emission = cColorf(1.0, 0.6, 0.0, 1.0);
    _flowingIronMesh->m_material->m_specular = cColorf(1.0, 0.6, 0.0, 1.0);
    _flowingIronMesh->m_material->setShininess(128);

    _lightIron1 = new cSpotLight(_renderingWorld);
    _renderingWorld->addChild(_lightIron1);                       // attach light to camera
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
        
    _lightIron2 = new cSpotLight(_renderingWorld);
    _renderingWorld->addChild(_lightIron2);                       // attach light to camera
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

    _lightIron3 = new cSpotLight(_renderingWorld);
    _renderingWorld->addChild(_lightIron3);                       // attach light to camera
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

    _lightIron4 = new cSpotLight(_renderingWorld);
    _renderingWorld->addChild(_lightIron4);                       // attach light to camera
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


    // set scale
    _fenceMesh->scale(0.001);
    _wallMesh->scale(0.001);
    _splashCoverMesh->scale(0.001);
    _outletMesh->scale(0.001);
    _pouringIronMesh->scale(0.001);


    // build dynamic model
    //_fence->setImageModel(_fenceMesh);
    //_fence->createDynamicMesh(true);
    _wall->setImageModel(_wallMesh);
    _wall->createDynamicMesh(true);
    _splashCover->setImageModel(_splashCoverMesh);
    _splashCover->createDynamicMesh(true);
    _outlet->setImageModel(_outletMesh);
    _outlet->createDynamicMesh(true);
    _pouringIron->setImageModel(_pouringIronMesh);
    _pouringIron->createDynamicMesh(true);
    _flowingIron->setImageModel(_flowingIronMesh);
    _flowingIron->createDynamicBox(1.8, 8.0, 0.3, true);


    _fence->rotateAboutGlobalAxisDeg(1, 0, 0, 90);
    _fence->rotateAboutGlobalAxisDeg(0, 0, 1, -90);
    _fence->setLocalPos(1.2, -1.6, 1.3);

    _wall->rotateAboutGlobalAxisDeg(1, 0, 0, 90);
    _wall->rotateAboutGlobalAxisDeg(0, 0, 1, 180);
    _wall->setLocalPos(4.0, -3.0, 0.0);

    _splashCover->rotateAboutGlobalAxisDeg(1, 0, 0, 90);
    _splashCover->setLocalPos(-1.8, 4.5, 1.25);

    _outlet->rotateAboutGlobalAxisDeg(1, 0, 0, -90);
    _outlet->setLocalPos(-0.6, -2.7, 2.35);

    _pouringIron->rotateAboutGlobalAxisDeg(1, 0, 0, 90);
    _pouringIron->setLocalPos(-0.165, 0.73, 1.145);

    _flowingIron->setLocalPos(0.0, 1.25, 0.7);
    //_origin->setLocalPos(0.0, 0.0, 3.0);
    //_origin->setLocalPos(0.0, 0.0, 3.0);

}

void cODEFurnace::_createDynamicObjects() {
    _depositedIron = new cODEGenericBody(_furnaceWorld);
    _depositedIronMesh = new cMultiMesh();

    if (!_depositedIronMesh->loadFromFile("../Resources/POSCO/octagon3.STL"))
    //if (!_depositedIronMesh->loadFromFile("../Resources/POSCO/dummy.STL"))
        printf("[Load deposited iron] 3D Model is failed to load correctly.\n");

    // set textures 
    vector<cMesh*>::iterator it;
    for (it = _depositedIronMesh->m_meshes->begin(); it < _depositedIronMesh->m_meshes->end(); it++) {
        setTextureMapFromFile(*it, "../Resources/POSCO/steel3.jpg", 0.002, 10.0);
    }

    _depositedIronMesh->scale(0.002);
    _depositedIronMesh->setShowFrame(false);
    _depositedIron->setImageModel(_depositedIronMesh);
    _depositedIron->createDynamicMesh(false);
    //_depositedIron->setShowFrame(true);
    _depositedIron->setMass(0.1);
    //dMass depositedIronMass;
    //dMassSetTrimesh(&depositedIronMass, 1.0, _depositedIron->m_ode_geom);
    //cout << depositedIronMass.c[0] << "," << depositedIronMass.c[1] << "," << depositedIronMass.c[2] << endl;
    //dBodySetPosition(_depositedIron->m_ode_body, depositedIronMass.c[0], depositedIronMass.c[1], depositedIronMass.c[2]);
    //dGeomSetOffsetPosition(_depositedIron->m_ode_geom, -depositedIronMass.c[0], -depositedIronMass.c[1], -depositedIronMass.c[2]);
    ////_depositedIronMesh->setLocalPos(depositedIronMass.c[0], depositedIronMass.c[1], depositedIronMass.c[2]);
    //dMassTranslate(&depositedIronMass, -depositedIronMass.c[0], -depositedIronMass.c[1], -depositedIronMass.c[2]);
    //dBodySetMass(_depositedIron->m_ode_body, &depositedIronMass);

    //dMassSetTrimesh(&depositedIronMass, 1.0, _depositedIron->m_ode_geom);
    //cout << depositedIronMass.c[0] << "," << depositedIronMass.c[1] << "," << depositedIronMass.c[2] << endl;


    _depositedIron->rotateAboutGlobalAxisDeg(1, 0, 0, 90);
    //_depositedIron->rotateAboutGlobalAxisDeg(0, 0, 1, 180);
    _depositedIron->setLocalPos(0.0, -2.0, 1.3);
}

void cODEFurnace::_createEndEffector() {
    _endEffectorTool = new cODEGenericBody(_furnaceWorld);
    _endEffectorToolMesh = new cMesh();
    _endEffectorCursorMesh = new cMesh();
    _renderingWorld->addChild(_endEffectorCursorMesh);


    cCreateSphere(_endEffectorCursorMesh, 0.01);
    cCreateCylinder(_endEffectorToolMesh, END_EFFECTOR_TOOL_LENGTH, 0.01, 
        32U, 1U, 1U, true, true, cVector3d(0.0, 0.0, -END_EFFECTOR_TOOL_LENGTH/2.0));

    _endEffectorTool->setImageModel(_endEffectorToolMesh);
    _endEffectorTool->createDynamicCylinder(0.01, END_EFFECTOR_TOOL_LENGTH, false);
    _endEffectorTool->setMass(0.1);
    localToolCoordRotation = rotateY(M_PI / 2.0);

    dBodySetAngularDamping(_endEffectorTool->m_ode_body, 0.02);
    dBodySetLinearDamping(_endEffectorTool->m_ode_body, 0.1);

    _endEffectorToolMesh->m_material->setWhite();
    _endEffectorToolMesh->setShowFrame(false);
    _endEffectorCursorMesh->m_material->setRedCrimson();
    _endEffectorCursorMesh->setShowFrame(true);
    _endEffectorCursorMesh->setFrameSize(0.05, true);

    _linStiffness = 1.0;
    _angStiffness = 0.01;
    _linDamping = 0.0;
    _angDamping = 0.0;

    _furnaceWorld->setTool(_endEffectorTool->m_ode_body);
}

void cODEFurnace::setEndEffectorPose(cVector3d position, cMatrix3d rotation) {
    cMatrix3d toolRotation = rotation * localToolCoordRotation;
    _endEffectorCursorMesh->setLocalPos(position);
    _endEffectorCursorMesh->setLocalRot(rotation);
    
    cVector3d toolOffset = cMul(toolRotation, cVector3d(0, 0, +END_EFFECTOR_TOOL_LENGTH / 2.0));
    _endEffectorTool->setLocalPos(position + toolOffset);
    //_endEffectorTool->setLocalPos(position);
    _endEffectorTool->setLocalRot(toolRotation);
}

void cODEFurnace::updateUserCommand(cVector3d posCommand, cMatrix3d rotCommand, bool activateCommand) {
    cVector3d posTool = _endEffectorTool->getLocalPos();
    cMatrix3d rotTool = _endEffectorTool->getLocalRot() * cTranspose(localToolCoordRotation);
    cVector3d posControlPoint = posTool + cMul(_endEffectorTool->getLocalRot(), cVector3d(0, 0, -END_EFFECTOR_TOOL_LENGTH / 2.0));

    // compute position and angular error between tool and haptic device
    cVector3d deltaPos = (posCommand - posControlPoint);
    cMatrix3d deltaRot = cMul(cTranspose(rotTool), rotCommand);
    double angle;
    cVector3d axis, axisAngle;
    deltaRot.toAxisAngle(axis, angle);
    axisAngle = axis * angle;
    //axisAngle.normalize();

    // compute force and torque to apply to tool
    cVector3d force, torque, forceAux;
    
    force = _linStiffness * deltaPos;
    if (activateCommand) {
        if (_toolContactOccur > 0){
            _endEffectorTool->addExternalForceAtPoint(force, posControlPoint);
        }
        else {
            _endEffectorTool->addExternalForce(force);
        }
    }

    torque = _angStiffness * axisAngle;
    forceAux = cCross(
        cMul(localToolCoordRotation, cVector3d(0, 0, -END_EFFECTOR_TOOL_LENGTH / 2.0)), torque);
    rotTool.mul(forceAux);
    rotTool.mul(torque);

    if (activateCommand) {
        _endEffectorTool->addExternalTorque(torque);
        _endEffectorTool->addExternalForce(forceAux);
    }

    if (activateCommand) {
        _endEffectorCursorMesh->setLocalPos(posControlPoint + force * 0.01);
        _endEffectorCursorMesh->setLocalRot(rotCommand);
    }
    else{
        _endEffectorCursorMesh->setLocalPos(posCommand);
        _endEffectorCursorMesh->setLocalRot(rotCommand);
    }
    // gravity compensation
    cVector3d gravity = _furnaceWorld->getGravity();
    _endEffectorTool->addExternalForce(_endEffectorTool->getMass() * -gravity);
}

bool cODEFurnace::updateShadowMaps(const bool mirror_x, const bool mirror_y) {
    return _renderingWorld->updateShadowMaps(mirror_x, mirror_y);
}

void cODEFurnace::renderView(const int a_windowWidth, const int a_windowHeight, const cEyeMode a_eyeMode, const bool a_defaultBuffer) {
    _camera->renderView(a_windowWidth, a_windowHeight, a_eyeMode, a_defaultBuffer);
}

void cODEFurnace::computeGlobalPositions(const bool frameOnly, const cVector3d& globalPos, const cMatrix3d& globalRot) {
    _furnaceWorld->computeGlobalPositions(frameOnly, globalPos, globalRot);
}

void cODEFurnace::updateDynamics(double interval) {
    _furnaceWorld->updateDynamics(interval);
    _furnaceWorld->calculateToolContactForceAndTorque();
    if (_furnaceWorld->m_toolContactForce.length() > 0) {
        _toolContactOccur = 10;
    }
    else {
        if (_toolContactOccur > 0) {
            _toolContactOccur--;
        }
    }
}

void cODEFurnace::addVisualComponent(cGenericObject* object) {
    _camera->m_frontLayer->addChild(object);
}

void cODEFurnace::getCameraPose(cVector3d& eye, cVector3d& target) {
    eye.copyfrom(_cameraEye);
    target.copyfrom(_cameraTarget);
}

void cODEFurnace::getEndEffectorPose(cVector3d& pos, cMatrix3d& rot) {
    (_endEffectorTool->getLocalPos() + cMul(_endEffectorTool->getLocalRot(), cVector3d(0, 0, -END_EFFECTOR_TOOL_LENGTH / 2.0))).copyto(pos);
    (_endEffectorTool->getLocalRot() * cTranspose(localToolCoordRotation)).copyto(rot);
}

void cODEFurnace::updateCameraPose(cVector3d eye, cVector3d target) {
    _cameraEye.copyfrom(eye);
    _cameraTarget.copyfrom(target);
    _camera->set(eye, target, cVector3d(0.0, 0.0, 1.0));     
}

void cODEFurnace::moveCamera(cVector3d step) {
    cMatrix3d viewMatrix = rotationMatrixFromVectors(_cameraTarget-_cameraEye, cVector3d(0.0, 0.0, 1.0));
    _cameraEye += cMul(viewMatrix, step);
    _cameraTarget += cMul(viewMatrix, step);
    _camera->set(_cameraEye, _cameraTarget, cVector3d(0.0, 0.0, 1.0));
    cout << _cameraEye << ", " << _cameraTarget << endl;
}

void cODEFurnace::rotateCamera(double yawStepInDeg, double pitchStepInDeg) {
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

void cODEFurnace::setEndEffectorStiffness(double posStiffness_, double rotStiffness_){
    _linStiffness = posStiffness_;
    _angStiffness = rotStiffness_;
}

void cODEFurnace::setEndEffectorDamping(double posDamping_, double rotDamping_) {
    _linDamping = posDamping_;
    _angDamping = rotDamping_;
}


void cODEFurnace::getForceTorqueSensorValue(cVector3d& sensorForce, cVector3d& sensorTorque) {
    sensorForce = _furnaceWorld->m_toolContactForce;
    sensorTorque = _furnaceWorld->m_toolContactTorque;
    
    //for (int i = 0; i < 3; i++) {
    //    sensorForce(i) = _endEffectorForceTorqueSensor->m_appliedForceBodyA[i];
    //    sensorTorque(i) = _endEffectorForceTorqueSensor->m_appliedTorqueBodyA[i];
    //}
}


void cODEFurnace::_createSimulationRule() {
    // failure
    _furnaceWorld->setCollisionRule(_endEffectorTool->m_ode_geom, _flowingIron->m_ode_geom);
    _furnaceWorld->setCollisionRule(_endEffectorTool->m_ode_geom, _pouringIron->m_ode_geom);
    
    // success
    _furnaceWorld->setCollisionRule(_depositedIron->m_ode_geom, _flowingIron->m_ode_geom);
}

int cODEFurnace::checkSimulationRule() {
    int res = 0;                // nothing

    // tool touched flowing iron
    if (_furnaceWorld->m_custom_rule[0].collided) {
        printf("[Demonstration failed] tool touched flowing iron\n");
        return -1;
    }

    // tool touched pouring iron
    if (_furnaceWorld->m_custom_rule[1].collided) {
        printf("[Demonstration failed] tool touched pouring iron\n");
        return -2;
    }

    // maximum force exceeded
    if (_furnaceWorld->m_toolContactForce.length() > maxForce) {
        printf("[Demonstration failed] tool force exceeded maximum value\n");
        return -3;
    }

    // maximum torque exceeded
    if (_furnaceWorld->m_toolContactTorque.length() > maxTorque) {
        printf("[Demonstration failed] tool torque exceeded maximum value\n");
        return -4;
    }

    // deposited iron removed
    if (_furnaceWorld->m_custom_rule[2].collided) {
        return 1;
    }

    return res;
}