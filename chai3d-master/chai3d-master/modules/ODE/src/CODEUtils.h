//===========================================================================
/*
Collection of miscellaneous functions for simulation

\author		<sansoveria@postech.ac.kr>
\author		Donghyeon Lee
\version	2020.12.12
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CODEUtilsH
#define CODEUtilsH
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
using namespace chai3d;
//---------------------------------------------------------------------------

//===========================================================================
/*!
	\class      cLowPassFilter
	\ingroup    ODE

	\brief
	This class implements an low pass filter for signal smoothing.

	\details
	cLowPassFilter is a discrete and first order low pass filter.
*/
//===========================================================================
class cLowPassFilter {
public:		// constructor and destructor
	cLowPassFilter() {	
		_delT = 0.001;			// default 1000 Hz
		_cutOffFreq = 50;		// default 50 Hz
		_initial_step = true;
	}

	~cLowPassFilter() {}

public:		// public members
public:		// public methods
	void setCutOffFreq(double freq);
	void setTimeStep(double dT);
	void reset();
	void filter(cVector3d input, cVector3d& output);

private:	// private members
	double _cutOffFreq;
	double _delT;

	bool _initial_step;

	cVector3d _input_prev;
	cVector3d _output_prev;

private:	// private methods
	
// TODO: requires variable timestep...?
// TODO: higher order LPF...?
};


//===========================================================================
/*!
	\class      cFilteredDerivative
	\ingroup    ODE

	\brief
	This class implements an filtered derivative for signal smoothing.

	\details
	cFilteredDerivative is a discrete derivative with first order low pass filter.
*/
//===========================================================================
class cFilteredDerivative {
public:		// constructor and destructor
	cFilteredDerivative() {
		_delT = 0.001;			// default 1000 Hz
		_cutOffFreq = 50;		// default 50 Hz
		_initial_step = true;
	}

	~cFilteredDerivative() {}

public:		// public members
public:		// public methods
	void setCutOffFreq(double freq);
	void setTimeStep(double dT);
	void reset();
	void filter(cVector3d input, cVector3d& output);

private:	// private members
	double _cutOffFreq;
	double _delT;

	bool _initial_step;

	cVector3d _input_prev;
	cVector3d _output_prev;

private:	// private methods

// TODO: requires variable timestep...?
// TODO: higher order LPF...?
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------