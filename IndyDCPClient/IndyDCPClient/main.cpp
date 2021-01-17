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
#include "CustomIndyDCPClient.h"
#include "Demo_setup.h"
#include "Timer.hpp"


#ifdef TELEOPERATION
double pRef1[6] = { 0.605, 0.076, 0.221, 0.0, 90.000, 0.000, };
double pdotRef1[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double pRef2[6] = { 0.455, 0.076, 0.221, 0.0, 180.000, 0.000, };
double pdotRef2[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

double* fext;

double masterPos[6], indyPos[6];
double passivityPort;
int cmode;
#endif

LARGE_INTEGER clockStart, clockEnd, clockFreq;
double clockTime_ms;

int main(int argc, char* argv[])
{
	QueryPerformanceFrequency(&clockFreq);

	IndyDedicatedTCPTestClient indyTCP;
	indyTCP.connect();
	
	CustomIndyDedicatedTCPTestClient teleoperationComm;
	teleoperationComm.connect();

	char key = 'x';
	printf("insert input\r\n");
	while (key != 'q') {
		key = _getch();
		printf("input: %c\r\n", key);
		switch (key) {
		case 'h':
			// go home
			QueryPerformanceCounter(&clockStart);
			indyTCP.GoHome();
			QueryPerformanceCounter(&clockEnd);
			clockTime_ms = ((double)clockEnd.QuadPart - (double)clockStart.QuadPart) / ((double)clockFreq.QuadPart) * 1000;
			printf("clock1: %f\n", clockTime_ms);
			break;

		case 'z':
			// go zero
			indyTCP.GoZero();
			break;

		case 'q':
			// quit

			printf("Quit IndyDCP Client\r\n");
			break;

		case 'j':
			for (int i = 0; i < 6; i++) {
				masterPos[i] = 0.0;
				indyPos[i] = 0.0;
			}
			passivityPort = 0.0;

			QueryPerformanceCounter(&clockStart);
			//for (int i = 0; i < 1000; i++) {
				teleoperationComm.SendIndyCommandAndReadState(&masterPos[0], passivityPort, &indyPos[0], cmode);
			//}
			
			QueryPerformanceCounter(&clockEnd);
			clockTime_ms = ((double)clockEnd.QuadPart - (double)clockStart.QuadPart) / ((double)clockFreq.QuadPart) * 1000;
			printf("clock1: %f\n", clockTime_ms);
			printf("indy pose: %.5f\t %.5f\t %.5f\t %.5f\t %.5f\t %.5f\t \n", indyPos[0], indyPos[1], indyPos[2], indyPos[3], indyPos[4], indyPos[5]);
			printf("cmode: %d \n", cmode);

			break;

		case 'm':
#ifdef TELEOPERATION
			indyTCP.ToggleTeleoperationMode();
#endif 
			break;

		default:
			printf("Wrong command\r\n");
		}
	}
}

