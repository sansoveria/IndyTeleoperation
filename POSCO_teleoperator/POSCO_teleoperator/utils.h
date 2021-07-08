#pragma once
//---------------------------------------------------------------------------
#ifndef UTILS_H
#define UTILS_H
//---------------------------------------------------------------------------
#include "chai3d.h"
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------

cMatrix3d identityMatrix();
cMatrix3d rotateX(double thetaInRad);
cMatrix3d rotateY(double thetaInRad);
cMatrix3d rotateZ(double thetaInRad);
cMatrix3d rotationMatrixFromVectors(cVector3d viewDirection, cVector3d upVector);
void setTextureMapFromFile(cMesh* mesh, string fileName, double scale, double maxSize);
#endif	//	UTILS