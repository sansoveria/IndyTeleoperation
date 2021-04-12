#include "utils.h"

cMatrix3d identityMatrix() {
	cMatrix3d res;
	res.identity();
	return res;
}

cMatrix3d rotateX(double thetaInRad) {
	cMatrix3d res;
	double s = sin(thetaInRad);
	double c = cos(thetaInRad);
	res(0, 0) = 1.0;
	res(0, 1) = 0.0;
	res(0, 2) = 0.0;
	res(1, 0) = 0.0;
	res(1, 1) = c;
	res(1, 2) = -s;
	res(2, 0) = 0.0;
	res(2, 1) = s;
	res(2, 2) = c;

	return res;
}

cMatrix3d rotateY(double thetaInRad) {
	cMatrix3d res;
	double s = sin(thetaInRad);
	double c = cos(thetaInRad);
	res(0, 0) = c;
	res(0, 1) = 0.0;
	res(0, 2) = s;
	res(1, 0) = 0.0;
	res(1, 1) = 1.0;
	res(1, 2) = 0.0;
	res(2, 0) = -s;
	res(2, 1) = 0.0;
	res(2, 2) = c;

	return res;
}

cMatrix3d rotateZ(double thetaInRad) {
	cMatrix3d res;
	double s = sin(thetaInRad);
	double c = cos(thetaInRad);
	res(0, 0) = c;
	res(0, 1) = -s;
	res(0, 2) = 0.0;
	res(1, 0) = s;
	res(1, 1) = c;
	res(1, 2) = 0.0;
	res(2, 0) = 0.0;
	res(2, 1) = 0.0;
	res(2, 2) = 1.0;

	return res;
}

cMatrix3d rotationMatrixFromVectors(cVector3d viewDirection, cVector3d upVector) {
	// Z axis - upVector
	// X axis - perpendicular to upVector and aligned to viewDirection

	cVector3d xAxis, yAxis, zAxis;
	upVector.normalizer(zAxis);
	viewDirection.normalizer(xAxis);
	zAxis.crossr(xAxis, yAxis);
	yAxis.normalize();
	yAxis.crossr(zAxis, xAxis);
	xAxis.normalize();

	cMatrix3d res(xAxis, yAxis, zAxis);
	return res;
}

void setTextureMapFromFile(cMesh* mesh, string fileName, double maxSize = 20.0) {
	mesh->m_texture = cTexture2d::create();
	if (!mesh->m_texture->loadFromFile(fileName))
		printf("[Load wall] texture file is failed to load correctly.\n");
	double textureMapHeight = mesh->m_texture->m_image->getHeight();
	double textureMapWidth = mesh->m_texture->m_image->getWidth();
	if (textureMapHeight > textureMapWidth)
		printf("[Load wall] texture image should be wide image (width > height).\n");
	mesh->setUseTexture(true);
	mesh->m_material->setWhite();
	unsigned int numTriangles = mesh->getNumTriangles();
	for (unsigned int i = 0; i < numTriangles; i++) {
		if (mesh->m_triangles->m_allocated[i]) {
			unsigned int index0 = mesh->m_triangles->getVertexIndex0(i);
			unsigned int index1 = mesh->m_triangles->getVertexIndex1(i);
			unsigned int index2 = mesh->m_triangles->getVertexIndex2(i);

			cVector3d pos0 = mesh->m_vertices->getLocalPos(index0) / 1000.0;
			cVector3d pos1 = mesh->m_vertices->getLocalPos(index1) / 1000.0;
			cVector3d pos2 = mesh->m_vertices->getLocalPos(index2) / 1000.0;

			cVector3d dir0 = pos1 - pos0;
			cVector3d dir1 = pos2 - pos0;
			dir0.normalize();
			dir1.normalize();

			cVector3d projCoord0((pos1 - pos0).length() / maxSize * textureMapHeight / textureMapWidth, 0.0, 0.0);
			double projCoord1x = abs(dir0.dot(pos2 - pos0));
			double projCoord1y = ((pos2 - pos0) - dir0.dot(pos2 - pos0) * dir0).length();
			cVector3d projCoord1(projCoord1x / maxSize * textureMapHeight / textureMapWidth, projCoord1y / maxSize, 0.0);
			cVector3d origin = pos0 / maxSize;
			if (projCoord0(0) > 1.0 || projCoord0(1) > 1.0 || projCoord1(0) > 1.0 || projCoord1(1) > 1.0)
				cout << projCoord0 << ", " << projCoord1 << endl;

			mesh->m_vertices->setTexCoord(index0, origin(0), origin(1), 0.0);
			mesh->m_vertices->setTexCoord(index1, origin(0) + projCoord0(0), origin(1) + projCoord0(1));
			mesh->m_vertices->setTexCoord(index2, origin(0) + projCoord1(0), origin(1) + projCoord1(1));
		}
	}
}