// Demo_execution.cpp : Defines the entry point for the console application.
//
#pragma once

#include "pch.h"

#include "conio.h"
#include "iostream"
#include <vector>
#include <fstream>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>

// custom
#include "NRMK_IndyDCPClient.h"
#include "CustomIndyDCPClent.h"
#include "Demo_setup.h"
#include "Timer.hpp"

// task
#include "MotionGenerator.h"

#if defined(TASK_SPACE_CONTROL_OFFLINE)
double *pos;

// for test
	//double point1[6] = { 0.455, -0.187, 0.416, 0.0, 180.000, 0.000, };
	//double point2[6] = { 0.605, -0.268, 0.416, 0.0, 180.000, 0.000,};
	//double point3[6] = { 0.605, 0.076, 0.416, 0.0, 180.000, 0.000, };
	//double point4[6] = { 0.605, 0.076, 0.221, 0.0, 180.000, 0.000, };
	//double point5[6] = { 0.455, -0.186, 0.416, 0.0, 180.000, 0.000, };

	//double point1[6] = { 0.455, -0.186, 0.416, 180.0, 0.000, 180.000, };
	//double point2[6] = { 0.505, -0.186, 0.356, 180.0, 0.000, 180.000, };
	//double point3[6] = { 0.555, -0.186, 0.296, 180.0, 0.000, 180.000, };
	//double point4[6] = { 0.605, -0.186, 0.236, 180.0, 0.000, 180.000, };
	//double point5[6] = { 0.655, -0.186, 0.176, 180.0, 0.000, 180.000, };
	//double point6[6] = { 0.705, -0.186, 0.116, 180.0, 0.000, 180.000, };

	// reference2
	//double point1[6] = { 0.45480, -0.18614, 0.41584, 0.0, M_PI, 0.000, };
	//double point2[6] = { 0.505, -0.186, 0.356, 0.0, M_PI, 0.000, };
	//double point3[6] = { 0.555, -0.186, 0.296, 0.0, M_PI, 0.000, };
	//double point4[6] = { 0.605, -0.186, 0.236, 0.0, M_PI, 0.000, };
	//double point5[6] = { 0.655, -0.186, 0.176, 0.0, M_PI, 0.000, };
	//double point6[6] = { 0.705, -0.186, 0.116, 0.0, M_PI, 0.000, };

	//// reference3/4
	//double point1[6] = { 0.45480, -0.18614, 0.41584, 0.0, M_PI, 0.000, };
	//double point2[6] = { 0.45480, -0.186, 0.356, 0.0, M_PI, 0.000, };
	//double point3[6] = { 0.45480, -0.186, 0.296, 0.0, M_PI, 0.000, };
	//double point4[6] = { 0.45480, -0.186, 0.236, 0.0, M_PI, 0.000, };
	//double point5[6] = { 0.45480, -0.186, 0.176, 0.0, M_PI, 0.000, };
	//double point6[6] = { 0.45480, -0.186, 0.116, 0.0, M_PI, 0.000, };

	// reference5 / 6
double point1[6] = { 0.45480, -0.18614, 0.41584, 0.0, M_PI, 0.000, };
double point2[6] = { 0.45480, -0.18614, 0.39584, 0.0, M_PI, 0.000, };
double point3[6] = { 0.45480, -0.18614, 0.37584, 0.0, M_PI, 0.000, };
double point4[6] = { 0.45480, -0.18614, 0.35584, 0.0, M_PI, 0.000, };
double point5[6] = { 0.45480, -0.18614, 0.33584, 0.0, M_PI, 0.000, };
double point6[6] = { 0.45480, -0.18614, 0.31584, 0.0, M_PI, 0.000, };

//// reference7
//double point1[6] = { 0.45480, -0.18614, 0.41584, 0.0, M_PI, 0.000, };
//double point2[6] = { 0.46080, -0.18614, 0.40784, 0.0, M_PI, 0.000, };
//double point3[6] = { 0.46680, -0.18614, 0.39984, 0.0, M_PI, 0.000, };
//double point4[6] = { 0.47280, -0.18614, 0.39184, 0.0, M_PI, 0.000, };
//double point5[6] = { 0.47880, -0.18614, 0.38384, 0.0, M_PI, 0.000, };
//double point6[6] = { 0.48480, -0.18614, 0.37584, 0.0, M_PI, 0.000, };

double res[6] = { 0.0 };
double resDot[6] = { 0.0 };
double resDDot[6] = { 0.0 };


ofstream trajectory;
//ofstream testOut("test.csv");
//ofstream testDerivativeOut("testDerivative.csv");
//ofstream testDDerivativeOut("testDDerivative.csv");

#endif

#ifdef TELEOPERATION
double pRef1[6] = { 0.605, 0.076, 0.221, 0.0, 90.000, 0.000, };
double pdotRef1[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double pRef2[6] = { 0.455, 0.076, 0.221, 0.0, 180.000, 0.000, };
double pdotRef2[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
#endif

int main(int argc, char* argv[])
{
	IndyDedicatedTCPTestClient indyTCP;
	indyTCP.connect();

	cubicBasisSpline<4000> traj;
	char refFileName[] = "reference.txt";
	char outputFileName[] = "traj.txt";

	

	char key = 'x';
	printf("insert input\r\n");
	while (key != 'q') {
		key = _getch();
		printf("input: %c\r\n", key);
		switch (key) {
		case 'h':
			// go home
			indyTCP.GoHome();
			break;

		case 'z':
			// go zero
			indyTCP.GoZero();
			break;

		case 'q':
			// quit
			printf("Quit IndyDCP Client\r\n");
			break;

		case 'a':
#ifdef TASK_SPACE_CONTROL_OFFLINE
			// add waypoint
			pos = indyTCP.GetTaskPos();
			traj.addWayPoint(pos);
			printf("current position %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
			printf("currently %d points are added\r\n", traj.numPoints());
#endif
#ifdef TELEOPERATION
			indyTCP.SetRefState(pRef2, pdotRef2);
#endif 
			break;

		case 'm':
#ifdef TELEOPERATION
			indyTCP.ToggleTeleoperationMode();
#endif 
			break;

		case 'b':
#ifdef TELEOPERATION
			indyTCP.SetRefState(pRef1, pdotRef1);
#endif
			break;

		case 'e':
#ifdef TASK_SPACE_CONTROL_OFFLINE
			//execute motion
			indyTCP.TrackTrajectory("traj.txt");
#endif
			break;

		case 'r':
#ifdef TASK_SPACE_CONTROL_OFFLINE
			// reset trajectory
			traj.resetSpline();
#endif
			break;

		case 'w':
#ifdef TASK_SPACE_CONTROL_OFFLINE
			// write trajectory
			trajectory.open("traj.txt");
#endif

			break;

		case 't':
			// test
#ifdef TASK_SPACE_CONTROL_OFFLINE
			// traj.test();
			traj.addWayPoint(point1, false);
			traj.addWayPoint(point2, false);
			traj.addWayPoint(point3, false);
			traj.addWayPoint(point4, false);
			traj.addWayPoint(point5, false);
			traj.addWayPoint(point6, false);

			traj.setDynamics(refFileName);
			traj.writeTrajectory(outputFileName);
#endif
			break;

		default:
			printf("Wrong command\r\n");
		}
	}
}

