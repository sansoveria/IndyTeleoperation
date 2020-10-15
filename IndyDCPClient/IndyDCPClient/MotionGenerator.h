#pragma once

#include <fstream>
#include <sstream>
#include <vector>

#define MAX_POINTS 100
#define DEBUG_PRINT

struct Point6D {
	double x, y, z, u, v, w;
	double operator()(int i, bool ROT_FIRST = true) {
		if (ROT_FIRST) {
			switch (i) {
			case 0:
				return u;
			case 1:
				return v;
			case 2:
				return w;
			case 3:
				return x;
			case 4:
				return y;
			case 5:
				return z;
			default:
				printf("wrong point index \r\n");
			}
		}
		else
		{
			switch (i) {
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;
			case 3:
				return u;
			case 4:
				return v;
			case 5:
				return w;
			default:
				printf("wrong point index \r\n");
			}
		}
	}

	void operator()(double* point, bool ROT_FIRST = true) {
		if (ROT_FIRST) {
			u = point[0];
			v = point[1];
			w = point[2];

			x = point[3];
			y = point[4];
			z = point[5];
		}
		else {
			x = point[0];
			y = point[1];
			z = point[2];

			u = point[3];
			v = point[4];
			w = point[5];
		}		
	}

	Point6D operator*(double arg) {
		Point6D tmp;
		tmp.x = arg * x;
		tmp.y = arg * y;
		tmp.z = arg * z;

		tmp.u = arg * u;
		tmp.v = arg * v;
		tmp.w = arg * w;
			   
		return tmp;
	}

	Point6D operator+(double arg) {
		Point6D tmp;
		tmp.x = arg + x;
		tmp.y = arg + y;
		tmp.z = arg + z;

		tmp.u = arg + u;
		tmp.v = arg + v;
		tmp.w = arg + w;

		return tmp;
	}

	Point6D operator+(Point6D arg) {
		Point6D tmp;
		tmp.x = arg.x + x;
		tmp.y = arg.y + y;
		tmp.z = arg.z + z;

		tmp.u = arg.u + u;
		tmp.v = arg.v + v;
		tmp.w = arg.w + w;

		return tmp;
	}

	double dispNorm() {
		return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	}

	double distance(Point6D point) {
		// calculate only displacement
		double res = sqrt(pow(point.x - x, 2) + pow(point.y - y, 2) + pow(point.z - z, 2));
		return res;
	}
};

template <int FREQ>
class cubicBasisSpline {
private:
	Point6D wayPoint[MAX_POINTS];
	double knot[MAX_POINTS + 6];
	double totalArcLength;
	int totalTrajLength;
	double totalTrajTime;
	double refLength[50000];
	double refVelocity[50000];
	int numRefPoints;
	bool DYNANMICS_SET;

	int pointIdx;
	const double delT = 1.0/(double)FREQ;

	double basisRecursive(int p, int i, double x) {
		double basis;
		if (p == 0) {
			if ((x == 0) && (knot[i] <= x) && (x <= knot[i + 1])) {
				basis = 1.0;
				return basis;
			}
			if ((knot[i] < x) && (x <= knot[i + 1])) {
				basis = 1.0;
				return basis;
			}
			else {
				basis = 0.0;
				return basis;
			}
		}

		double coeff1, coeff2;
		if (knot[i + p] - knot[i] == 0) {
			coeff1 = 0.0;
		}
		else {
			coeff1 = (x - knot[i]) / (knot[i + p] - knot[i]);
		}
		if (knot[i + p + 1] - knot[i + 1] == 0) {
			coeff2 = 0.0;
		}
		else {
			coeff2 = (knot[i + p + 1] - x) / (knot[i + p + 1] - knot[i + 1]);
		}
		basis = coeff1 * basisRecursive(p - 1, i, x) + coeff2 * basisRecursive(p - 1, i + 1, x);
		return basis;
	}

	double basisDerivativeRecursive(int p, int i, double x) {
		double basis;
		if (p == 0) {
			return 0.0;
		}

		double coeff1, coeff2, coeffDerivative1, coeffDerivative2;
		
		if (knot[i + p] - knot[i] == 0) {
			coeff1 = 0.0;
			coeffDerivative1 = 0.0;
		}
		else {
			coeff1 = (x - knot[i]) / (knot[i + p] - knot[i]);
			coeffDerivative1 = 1.0 / (knot[i + p] - knot[i]);
		}

		if (knot[i + p + 1] - knot[i + 1] == 0) {
			coeff2 = 0.0;
			coeffDerivative2 = 0.0;
		}
		else {
			coeff2 = (knot[i + p + 1] - x) / (knot[i + p + 1] - knot[i + 1]);
			coeffDerivative2 = -1.0 / (knot[i + p + 1] - knot[i + 1]);
		}

		basis = coeff1 * basisDerivativeRecursive(p - 1, i, x) + coeffDerivative1 * basisRecursive(p - 1, i, x)
			+ coeff2 * basisDerivativeRecursive(p - 1, i + 1, x) + coeffDerivative2 * basisRecursive(p - 1, i + 1, x);
		return basis;
	}

	double basisDDerivativeRecursive(int p, int i, double x) {
		double basis;
		if (p < 2) {
			return 0.0;
		}

		double coeff1, coeff2, coeffDerivative1, coeffDerivative2;

		if (knot[i + p] - knot[i] == 0) {
			coeff1 = 0.0;
			coeffDerivative1 = 0.0;
		}
		else {
			coeff1 = (x - knot[i]) / (knot[i + p] - knot[i]);
			coeffDerivative1 = 1.0 / (knot[i + p] - knot[i]);
		}

		if (knot[i + p + 1] - knot[i + 1] == 0) {
			coeff2 = 0.0;
			coeffDerivative2 = 0.0;
		}
		else {
			coeff2 = (knot[i + p + 1] - x) / (knot[i + p + 1] - knot[i + 1]);
			coeffDerivative2 = -1.0 / (knot[i + p + 1] - knot[i + 1]);
		}

		basis = coeff1 * basisDerivativeRecursive(p - 1, i, x) + coeffDerivative1 * basisRecursive(p - 1, i, x)
			+ coeff2 * basisDerivativeRecursive(p - 1, i + 1, x) + coeffDerivative2 * basisRecursive(p - 1, i + 1, x);

		basis = coeffDerivative1 * basisDerivativeRecursive(p - 1, i, x) + coeff1 * basisDDerivativeRecursive(p - 1, i, x)
			+ coeffDerivative1 * basisDerivativeRecursive(p - 1, i, x)
			+ coeffDerivative2 * basisDerivativeRecursive(p - 1, i + 1, x) + coeff2 * basisDDerivativeRecursive(p - 1, i + 1, x)
			+ coeffDerivative2 * basisDerivativeRecursive(p - 1, i + 1, x);
		return basis;
	}

	void calcTotalArcLength() {
		Point6D ptCur, ptPrev;
		double length = 0.0;

		ptPrev = wayPoint[0];
		for (int i = 1; i < (pointIdx - 3) * 1000 + 1; i++) {
			calcPosition(i / 1000.0, ptCur);
			length += ptCur.distance(ptPrev);
			ptPrev = ptCur;
		}

		totalArcLength = length;
		printf("[calcTotalArcLength] total length: approx. %.5f\r\n", length);
	}

	double calcArcLength(double xPrev, double x, double prevLength) {
		Point6D ptCur, ptPrev;
		double length = prevLength;

		int prevIdx = floor(xPrev * 10000);
		int curIdx = floor(x * 10000);
		calcPosition(prevIdx / 10000.0, ptPrev);

		for (int i = prevIdx; i < curIdx + 1; i++) {
			calcPosition(i / 10000.0, ptCur);
			length += ptCur.distance(ptPrev);	
			ptPrev = ptCur;
		}
		return length;
	}

	bool getRefVelocity(double x, int& idx, double & pos, double & vel) {
		for (int i = idx; i < numRefPoints; i++) {
			if (x <= refLength[i]) {
#ifdef DEBUG_PRINT
				printf("[getRefVelocity] refLen: %.5f refIdx: %d refVel: %.5f\r\n", x, i, refVelocity[i]);
#endif	
				pos = refLength[i];
				vel = refVelocity[i];
				idx = i;
				return true;
			}
		}

		printf("[getRefVelocity] Cannot find reference point\r\n");
		return false;
	}

public:
	cubicBasisSpline() {
		resetSpline();

#ifdef DEBUG_PRINT
		printf("[cubicBasisSpline] Trajectory timestep: %.5f\r\n", delT);
#endif
	}

	void resetSpline() {
		for (int i = 0; i < MAX_POINTS + 6; i++) {
			knot[i] = 0.0;
		}
		DYNANMICS_SET = false;
		pointIdx = 0;
		numRefPoints = 0;
		totalTrajLength = 0;
		totalTrajTime = 0.0;
	}

	int numPoints() {
		return pointIdx;
	}

	int trajLen() {
		return 0;
	}
	
	// function to read the reference trajectory (txt)
	bool setDynamics(char* filename) {
		if (pointIdx > 4) {
			calcTotalArcLength();

			// set trajectory dynamics
			// in txt file, distance - velocity mapping is assigned
			// ex) dist1, vel1
			//     dist2, vel2
			//     dist3, vel3
			//     ...

			int idx = 0;
			//double dummy;
			std::ifstream cFile(filename);
			if (cFile.is_open())
			{
				while (!cFile.eof()) {
					cFile >> refLength[idx];
					cFile >> refVelocity[idx];
					//cFile >> dummy;				// we do not need acceleration

					idx++;

					if (idx > 50000) {
						printf("[SetDynamics] Too many reference points\r\n");
						break;
					}
				}
				numRefPoints = idx - 1;			// throw away the last line for safe parsing
			}
			else {
				std::cerr << "[SetDynamics] Couldn't open config file for reading.\n";
				return false;
			}

			if (numRefPoints > 0) {
				printf("[SetDynamics] %d points are read. Reference trajectory length: %.5f\r\n", numRefPoints, refLength[numRefPoints - 1]);
#ifdef DEBUG_PRINT
				//for (int i = 0; i < numRefPoints - 1; i++) {
				//	printf("%d ref point: %.5f %.5f\r\n", i, refLength[i], refVelocity[i]);
				//}
#endif
			}
			else {
				printf("[SetDynamics] No point exists\r\n");
				return false;
			}



			DYNANMICS_SET = true;
			return true;
		}
		else {
			printf("[SetDynamics] Too few waypoints. Cannot build spline. \r\n");
			return false;
		}
	}

	// build full trajectory 
	bool writeTrajectory(char * filename) {
		// calculate p, pdot, pddot for every timestep
		if (DYNANMICS_SET) {
			double x = 0.0;
			double xPrev = 0.0;
			double xDot, xDotPrev, xDDot;
			double aLen = 0.0;
			double aLenPrev = 0.0;
			int refIdx = 0;
			int idx, arcIdx;

			double refPos, refVel;
			Point6D curPos, curVel, curAcc;
			std::vector<Point6D> outputPos, outputVel, outputAcc;

#ifdef DEBUG_PRINT
			printf("[writeTrajectory] Traj length: %.5f, reference traj length: %.5f\r\n", totalArcLength, refLength[numRefPoints - 1]);
#endif

			//idx = -1.0;
			idx = 0;
			xDot = 1.0;
			xDotPrev = 0.0;
			xDDot = 0.0;
			while (x < pointIdx - 3) {
				if (xDot < 0.00001) {
					printf("[writeTrajectory] Dynamics is too slow \r\n");
					//return false;
					break;
				}

				aLen = calcArcLength(xPrev, x, aLenPrev);
							
				if (!(getRefVelocity(aLen / totalArcLength * refLength[numRefPoints - 1], refIdx, refPos, refVel)
					&& calcPosition(x, curPos)
					&& calcVelocity(x, curVel)
					&& calcAcceleration(x, curAcc)))
				{
					return false;
				}				

				xDot = sqrt(totalArcLength/ refLength[numRefPoints - 1]) *refVel / curVel.dispNorm();
				xDDot = (xDot - xDotPrev) / delT;

				//save trajectory
				outputPos.push_back(curPos);
				Point6D tmpVel = curVel * xDot;
				outputVel.push_back(tmpVel);
				Point6D tmpAcc = curAcc * pow(xDot, 2) + curVel * xDDot;
				outputAcc.push_back(tmpAcc);

#ifdef DEBUG_PRINT
				if (idx % 100 == 0) {
					printf("[writeTrajectory] time: %.5f x: %.5f arcLength: %.5f xDot: %.5f xDDot: %.5f Pref: %.5f Vref: %.5f \r\n", idx*delT, x, aLen, xDot, xDDot, refPos, refVel);
				}
#endif

				//update
				xPrev = x;
				aLenPrev = aLen;
				x += xDot * delT;
				xDotPrev = xDot;
			
				idx++;
			}
			totalTrajLength = idx;
			totalTrajTime = idx * delT;

			printf("[writeTrajectory] %d points are generated. Total running time is %.5f \r\n", totalTrajLength, totalTrajTime);

			//write file
			ofstream output(filename);

			if (output.is_open()) {
				output << 2 << " " << FREQ << " " << 3 << " " << 6 << " " << totalTrajLength << std::endl;

				for (int i = 0; i < outputPos.size(); i++) {
					for (int j = 0; j < 6; j++) {
						output << outputPos[i](j, false) << " ";
					}
					for (int j = 0; j < 6; j++) {
						output << outputVel[i](j, false) << " ";
					}
					for (int j = 0; j < 6; j++) {
						output << outputAcc[i](j, false) << " ";
					}
					output << std::endl;
				}
			}

			output.close();
			printf("[writeTrajectory] File is written\r\n");
		}
		else {
			printf("[writeTrajectory] Trajectory Dynamics is not set yet. \r\n");
			return false;
		}
	}

	// a test function to logging all the trajectory variables for debugging
	int test(char * filename) {
		// calculate p, pdot, pddot for every timestep
		if (DYNANMICS_SET) {
			double x = 0.0;
			double xPrev = 0.0;
			double xDot, xDotPrev, xDDot;
			double aLen = 0.0;
			double aLenPrev = 0.0;
			int refIdx = 0;
			//double totalLen = arcLength[(pointIdx - 3) * 1000];
			int idx, arcIdx;

			double refPos, refVel;
			Point6D curPos, curVel, curAcc;
			std::vector<Point6D> outputPos, outputVel, outputAcc;
			std::vector<double> outputX, outputXdot, outputXddot;

#ifdef DEBUG_PRINT
			printf("[test] Traj length: %.5f, reference traj length: %.5f\r\n", totalArcLength, refLength[numRefPoints - 1]);
#endif

			//idx = -1.0;
			idx = 0;
			xDot = 1.0;
			xDotPrev = 0.0;
			xDDot = 0.0;
			while (x < pointIdx - 3) {
				if (xDot < 0.00001) {
					printf("[test] Dynamics is too slow \r\n");
					//return false;
					break;
				}

				//if (xDot < 0.001) {
				//	aLen = calcArcLength(x);
				//}
				//else {
				//	arcIdx = floor(x * 1000);
				//	aLen = arcLength[arcIdx];
				//}
				aLen = calcArcLength(xPrev, x, aLenPrev);

				if (!(getRefVelocity(aLen / totalArcLength * refLength[numRefPoints - 1], refIdx, refPos, refVel)
					&& calcPosition(x, curPos)
					&& calcVelocity(x, curVel)
					&& calcAcceleration(x, curAcc)))
				{
					return false;
				}

				xDot = sqrt(totalArcLength / refLength[numRefPoints - 1]) * refVel / curVel.dispNorm();
				xDDot = (xDot - xDotPrev) / delT;

				//save trajectory
				outputPos.push_back(curPos);
				Point6D tmpVel = curVel * xDot;
				outputVel.push_back(tmpVel);
				Point6D tmpAcc = curAcc * pow(xDot, 2) + curVel * xDDot;
				outputAcc.push_back(tmpAcc);
				outputX.push_back(x);
				outputXdot.push_back(xDot);
				outputXddot.push_back(xDDot);

#ifdef DEBUG_PRINT
				if (idx % 100 == 0) {
					printf("[test] time: %.5f x: %.5f arcLength: %.5f xDot: %.5f xDDot: %.5f Pref: %.5f Vref: %.5f \r\n", idx*delT, x, aLen, xDot, xDDot, refPos, refVel);
				}
#endif

				//update
				xPrev = x;
				aLenPrev = aLen;
				x += xDot * delT;
				xDotPrev = xDot;

				idx++;
			}
			totalTrajLength = idx;
			totalTrajTime = idx * delT;

			printf("[test] %d points are generated. Total running time is %.5f \r\n", totalTrajLength, totalTrajTime);

			//write file
			ofstream output(filename);

			if (output.is_open()) {
				output << 2 << " " << FREQ << " " << 3 << " " << 6 << " " << totalTrajLength << std::endl;

				for (int i = 0; i < outputPos.size(); i++) {
					for (int j = 0; j < 6; j++) {
						output << outputPos[i](j, false) << ", ";
					}
					for (int j = 0; j < 6; j++) {
						output << outputVel[i](j, false) << ", ";
					}
					for (int j = 0; j < 6; j++) {
						output << outputAcc[i](j, false) << ", ";
					}
					output << outputX[i] << ", " << outputXdot[i] << "," << outputXddot[i];
					output << std::endl;
				}
			}

			output.close();
			printf("[test] File is written\r\n");
		}
		else {
			printf("[test] Trajectory Dynamics is not set yet. \r\n");
			return false;
		}
	}

	bool calcPosition(double param, Point6D& res) {
		// 3rd order b-Spline
		int order = 3;
		double tmp[6] = { 0.0 };		
		if (pointIdx > 4) {
			for (int i = 0; i < pointIdx; i++) {
				for (int j = 0; j < 6; j++) {
					tmp[j] += wayPoint[i](j)*basisRecursive(order, i, param);
				}
			}

			//for (int i = 0; i < 6; i++) {
			//	res[i] = tmp[i];
			//}
			res(tmp);
			return true;
		}
		else {
			printf("[calcPosition] Too few waypoints. Cannot build spline. \r\n");
			return false;
		}
	}

	bool calcVelocity(double param, Point6D& res) {
		// 3rd order b-Spline
		int order = 3;
		double tmpDot[6] = { 0.0 };
		if (pointIdx > 4) {
			for (int i = 0; i < pointIdx; i++) {
				for (int j = 0; j < 6; j++) {
					tmpDot[j] += wayPoint[i](j)*basisDerivativeRecursive(order, i, param);
				}
			}

			//for (int i = 0; i < 6; i++) {
			//	res[i] = tmpDot[i];
			//}

#ifdef DEBUG_PRINT
			//printf("[calcVelocity] vel: %.5f %.5f %.5f %.5f %.5f %.5f \r\n", tmpDot[0], tmpDot[1], tmpDot[2], tmpDot[3], tmpDot[4], tmpDot[5]);
#endif

			res(tmpDot);
			return true;
		}
		else {
			printf("[calcVelocity] Too few waypoints. Cannot build spline. \r\n");
			return false;
		}
	}

	bool calcAcceleration(double param, Point6D& res) {
		// 3rd order b-Spline
		int order = 3;
		double tmpDDot[6] = { 0.0 };
		if (pointIdx > 4) {
			for (int i = 0; i < pointIdx; i++) {
				for (int j = 0; j < 6; j++) {
					tmpDDot[j] += wayPoint[i](j)*basisDDerivativeRecursive(order, i, param);
				}
			}

			//for (int i = 0; i < 6; i++) {
			//	res[i] = tmpDot[i];
			//}
			res(tmpDDot);
			return true;
		}
		else {
			printf("[calcAcceleration] Too few waypoints. Cannot build spline. \r\n");
			return false;
		}
	}


	void interpolate(double param, double* res, double* resDot, double* resDDot) {
		// 3rd order b-Spline
		int order = 3;
		double tmp[6] = { 0.0 };
		double tmpDot[6] = { 0.0 };
		double tmpDDot[6] = { 0.0 };
		if (pointIdx > 4) {

			for (int i = 0; i < pointIdx; i++) {
				for (int j = 0; j < 6; j++) {
					tmp[j] += wayPoint[i](j)*basisRecursive(order, i, param);
					tmpDot[j] += wayPoint[i](j)*basisDerivativeRecursive(order, i, param);
					tmpDDot[j] += wayPoint[i](j)*basisDDerivativeRecursive(order, i, param);
				}
#ifdef DEBUG_PRINT
				printf("[interpolate] basis f(%d, %.3f): %.3f\r\n", i, param, basisRecursive(order, i, param));
#endif
			}

//			for (int i = -2; i < pointIdx-2; i++) {
//				for (int j = 0; j < 6; j++) {
//					if (i < 0) {
//						tmp[j] += wayPoint[0](j)*basis_recursive(order, i, param);
//					}
//					else if(i >= pointIdx-2) {
//						tmp[j] += wayPoint[pointIdx-1](j)*basis_recursive(order, i, param);
//					}
//					else {
//						tmp[j] += wayPoint[i+2](j)*basis_recursive(order, i, param);
//					}
//				}
//#ifdef DEBUG_PRINT
//				printf("basis f(%d, %.3f): %.3f\r\n", i, param, basis_recursive(order, i, param));
//#endif
//			}

			for (int i = 0; i < 6; i++) {
				res[i] = tmp[i];
				resDot[i] = tmpDot[i];
				resDDot[i] = tmpDDot[i];
			}
		}
		else {
			printf("[interpolate] Too few waypoints. Cannot build spline. \r\n");			
		}
	}

	void addWayPoint(double * point, bool ROT_FIRST = true) {
		wayPoint[pointIdx](point, ROT_FIRST);
		//if (ROT_FIRST) {
		//	wayPoint[pointIdx].u = point[0];
		//	wayPoint[pointIdx].v = point[1];
		//	wayPoint[pointIdx].w = point[2];

		//	wayPoint[pointIdx].x = point[3];
		//	wayPoint[pointIdx].y = point[4];
		//	wayPoint[pointIdx].z = point[5];
		//}
		//else {
		//	wayPoint[pointIdx].x = point[0];
		//	wayPoint[pointIdx].y = point[1];
		//	wayPoint[pointIdx].z = point[2];

		//	wayPoint[pointIdx].u = point[3];
		//	wayPoint[pointIdx].v = point[4];
		//	wayPoint[pointIdx].w = point[5];
		//}

		if (pointIdx > 2) {
			knot[pointIdx + 1] = (double)pointIdx-2;
			knot[pointIdx + 2] = (double)pointIdx-2;
			knot[pointIdx + 3] = (double)pointIdx-2;
			knot[pointIdx + 4] = (double)pointIdx-2;
		}
#ifdef DEBUG_PRINT
		printf("[addWayPoint] knot: ");
		for (int i = 0; i < pointIdx + 5; i++) {
			printf("%f ", knot[i]);
		}
		printf("\r\n");
		printf("[addWayPoint] new waypoint: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n",
			wayPoint[pointIdx].x, wayPoint[pointIdx].y, wayPoint[pointIdx].z,
			wayPoint[pointIdx].u, wayPoint[pointIdx].v, wayPoint[pointIdx].w);
#endif
		pointIdx++;
	}
};