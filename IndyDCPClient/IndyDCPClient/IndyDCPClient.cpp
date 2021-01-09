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
#endif

int main(int argc, char* argv[])
{
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
#ifdef TELEOPERATION
			fext = teleoperationComm.SetRefState(pRef2, pdotRef2);
			for (int i = 0; i < 6; i++) {
				printf("%f ", fext[i]);
			}
			printf("\n");
#endif
			break;

		case 'm':
#ifdef TELEOPERATION
			indyTCP.ToggleTeleoperationMode();
#endif 
			break;

		case 'b':
#ifdef TELEOPERATION
			fext = teleoperationComm.SetRefState(pRef1, pdotRef1);
			for (int i = 0; i < 6; i++) {
				printf("%f ", fext[i]);
			}
			printf("\n");
#endif
			break;

		case 't':
#ifdef TELEOPERATION
			teleoperationComm.RaiseExtWrench();
#endif
			break;

		case 'g':
#ifdef TELEOPERATION
			teleoperationComm.ReduceExtWrench();
#endif
			break;
		default:
			printf("Wrong command\r\n");
		}
	}
}

