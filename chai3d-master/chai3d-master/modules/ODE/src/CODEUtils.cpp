//===========================================================================
/*
Collection of miscellaneous functions for simulation

\author		<sansoveria@postech.ac.kr>
\author		Donghyeon Lee
\version	2020.12.12
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CODEUtils.h"
//---------------------------------------------------------------------------
// cLowPassFilter class
//---------------------------------------------------------------------------

//===========================================================================
/*!
	This method sets cut off frequency of low pass filter.

	\param freq is cut off frequency.
*/
//===========================================================================
void cLowPassFilter::setCutOffFreq(double freq) {
	_cutOffFreq = freq;
}

//===========================================================================
/*!
	This method sets timestep of discrete system.

	\param dT is timestep.
*/
//===========================================================================
void cLowPassFilter::setTimeStep(double dT) {
	_delT = dT;
}

//===========================================================================
/*!
	This method resets private member variables (input_prev, oupput_prev).

	\param dT is timestep.
*/
//===========================================================================
void cLowPassFilter::reset() {
	_input_prev = cVector3d(0.0, 0.0, 0.0);
	_output_prev = cVector3d(0.0, 0.0, 0.0);
	_initial_step = true;
}

//===========================================================================
/*!
	This method applies low pass filter.
	If the filter is applied for the first time (_initial_step == true),
	the filter returns the input vector
*/
//===========================================================================
void cLowPassFilter::filter(cVector3d input, cVector3d& output) {
	if (_initial_step) {
		_input_prev = input;
		_output_prev = input;
		_initial_step = false;
	}

	double tau = 1.0 / (2.0*M_PI*_cutOffFreq);
	output = (2.0*tau - _delT) / (2.0*tau + _delT)*_output_prev + _delT / (2 * tau + _delT)*(input + _input_prev);
	
	_input_prev.copyfrom(input);
	_output_prev.copyfrom(output);
}

//---------------------------------------------------------------------------
// cFilteredDerivative class
//---------------------------------------------------------------------------

//===========================================================================
/*!
	This method sets cut off frequency of low pass filter.

	\param freq is cut off frequency.
*/
//===========================================================================
void cFilteredDerivative::setCutOffFreq(double freq) {
	_cutOffFreq = freq;
}

//===========================================================================
/*!
	This method sets timestep of discrete system.

	\param dT is timestep.
*/
//===========================================================================
void cFilteredDerivative::setTimeStep(double dT) {
	_delT = dT;
}

//===========================================================================
/*!
	This method resets private member variables (input_prev, oupput_prev).

	\param dT is timestep.
*/
//===========================================================================
void cFilteredDerivative::reset() {
	_input_prev = cVector3d(0.0, 0.0, 0.0);
	_output_prev = cVector3d(0.0, 0.0, 0.0);
	_initial_step = true;
}

//===========================================================================
/*!
	This method calculates filtered derivative.
	If the filter is applied for the first time (_initial_step == true),
	the filter returns the zero derivative
*/
//===========================================================================
void cFilteredDerivative::filter(cVector3d input, cVector3d& output) {
	if (_initial_step) {
		_input_prev = input;
		_output_prev = cVector3d(0.0, 0.0, 0.0);
		_initial_step = false;
	}

	double tau = 1.0 / (2.0*M_PI*_cutOffFreq);
	output = (2.0*tau - _delT) / (2.0*tau + _delT)*_output_prev + 2.0 / (2.0*tau + _delT)*(input - _input_prev);

	_input_prev.copyfrom(input);
	_output_prev.copyfrom(output);
}