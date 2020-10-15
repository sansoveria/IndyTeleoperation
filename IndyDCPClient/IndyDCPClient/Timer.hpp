#pragma once

#include <Windows.h>
#include <vector>
#include <string>
class timer
{
private:
	LARGE_INTEGER start;
	std::vector<LARGE_INTEGER> starts;

	LARGE_INTEGER end;
	LARGE_INTEGER freq;

public:
	timer(){
		QueryPerformanceFrequency(&freq);
		QueryPerformanceCounter(&start);
	}

	void tick(){
		QueryPerformanceCounter(&start);
	}

	//void tic(int number){
	//	LARGE_INTEGER tmp;
	//	QueryPerformanceCounter(&tmp);
	//	starts.push_back(tmp);
	//	indice.push_back(number);
	//}

	double tock(){
		QueryPerformanceCounter(&end);
		double time = (end.QuadPart - start.QuadPart) / (double)freq.QuadPart;
		return time;
	}


};