#pragma once

#include "CIndyFurnace.h"
#include "example.hpp"          // Include short list of convenience functions for rendering


using namespace std;

cIndyFurnace::cIndyFurnace() {
    // set world and camera
    _setWorld();

    // setup devices
    _setIndy();
    _setPointCloud();

}

cIndyFurnace::~cIndyFurnace() {
    delete _realSensePipeline;
    delete _pointCloud;
    delete _renderingWorld;       // deconstructor of cODEWorld deletes every child objects
}

void cIndyFurnace::_setWorld() {
    _renderingWorld = new cWorld();

    // set the background color of the environment
    _renderingWorld->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    _camera = new cCamera(_renderingWorld);
    _renderingWorld->addChild(_camera);

    // position and oriente the camera
    _cameraEye = cVector3d(3.0, 0.0, 0.0);
    _cameraTarget = cVector3d(0.0, 0.0, 0.0);
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

}


void cIndyFurnace::_setPointCloud() {
    _realSensePipeline = new rs2::pipeline();
    _realSensePipeline->start();

    _pointCloud = new cMultiPoint();
    //_pointCloud->m_texture = cTexture2d::create();
    //_pointCloudTextureMap = cImage::create();
    _renderingWorld->addChild(_pointCloud);

    _pointCloud->setShowFrame(true);
    _pointCloud->rotateAboutGlobalAxisDeg(0, 1, 0, -90);
    _pointCloud->rotateAboutGlobalAxisDeg(1, 0, 0, -90);
}


void cIndyFurnace::setEndEffectorPose(cVector3d position, cMatrix3d rotation) {
    // TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
    // move Indy to given pose (TaskMoveTo)
    // TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
}

void cIndyFurnace::updateUserCommand(cVector3d posCommand, cMatrix3d rotCommand) {
    // TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
    // code for indy communication (send pose and receive sensor values)
    // TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
}

bool cIndyFurnace::updateShadowMaps(const bool mirror_x, const bool mirror_y) {
    return _renderingWorld->updateShadowMaps(mirror_x, mirror_y);
}

void cIndyFurnace::renderView(const int a_windowWidth, const int a_windowHeight, const cEyeMode a_eyeMode, const bool a_defaultBuffer) {
    auto frames = _realSensePipeline->wait_for_frames();
    
    auto color = frames.get_color_frame();
    if (!color) color = frames.get_infrared_frame();

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

    //_pointCloud->m_texture->setImage(_pointCloudTextureMap);
    //_pointCloud->setUseTexture(true, true);
    _pointCloud->setUseVertexColors(true, true);
    //_pointCloud->setUseMaterial(true, true);
    //_pointCloud->m_material->setWhite();
    _pointCloud->setPointSize(4.0);
    int pointSampling = 4;
    for (int xIdx = 0; xIdx < widthDepth / pointSampling; xIdx++){
        for (int yIdx = 0; yIdx < heightDepth / pointSampling; yIdx++) {
            int pointIdx = yIdx * pointSampling * widthDepth + xIdx * pointSampling;
            //cout << pointIdx << ", " << _points.size() << ", " << widthDepth << ", " << heightDepth << endl;
            int xColorMap = texCoord[pointIdx].u * width;
            int yColorMap = texCoord[pointIdx].v * height;

            if (yColorMap <= 0 || yColorMap >= height) continue;
            if (xColorMap <= 0 || xColorMap >= width) continue;

            int colorLocation = 3 * (yColorMap * width + xColorMap);
            //cout << pointIdx << ", " << colorLocation << ", " << texCoord[pointIdx].u << ", " << texCoord[pointIdx].v << ", " << width << ", " << height << endl;
            unsigned char pointColor[3] = { colorStream[colorLocation], colorStream[colorLocation + 1], colorStream[colorLocation + 2] };
            //cout << colorLocation << ", " << int(pointColor[0]) << ", " << int(pointColor[1]) << ", " << int(pointColor[2]) << endl;
            unsigned int vertexIdx = _pointCloud->newPoint(cVector3d(vertex[pointIdx].x, vertex[pointIdx].y, vertex[pointIdx].z), cColorf(pointColor[0]/255.0, pointColor[1]/255.0, pointColor[2]/255.0, 1));
            //cout << texCoord[pointIdx].u << ", " << texCoord[pointIdx].v << endl;
            //cColorb tmp;
            //_pointCloudTextureMap->getPixelColor(xColorMap, yColorMap, tmp);
            //cout << tmp[0] << ", " << tmp[1] << ", " << tmp[2] << endl;
            //unsigned int vertexIdx = _pointCloud->newPoint(cVector3d(vertex[pointIdx].x, vertex[pointIdx].y, vertex[pointIdx].z));
            //_pointCloud->m_vertices->setTexCoord(vertexIdx, cVector3d(texCoord[pointIdx].u, texCoord[pointIdx].v, 0.0));
        }
    }
    _pointCloud->m_points->m_vertices = _pointCloud->m_vertices;

    _camera->renderView(a_windowWidth, a_windowHeight, a_eyeMode, a_defaultBuffer);
    //free(textureImageData);
}

void cIndyFurnace::computeGlobalPositions(const bool frameOnly, const cVector3d& globalPos, const cMatrix3d& globalRot) {
    // TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
    // update local pos, local rot of _indy object
    // TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO

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

