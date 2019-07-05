/** @file *//********************************************************************************************************

                                                  PIDController.cpp

						                    Copyright 2003, John J. Bolton
	--------------------------------------------------------------------------------------------------------------

	$Header: //depot/Libraries/PIDController/PIDController.cpp#1 $

	$NoKeywords: $

 ********************************************************************************************************************/

#include "PIDController.h"


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

/// @param	kp	P parameter.
/// @param	ki	I parameter.
/// @param	kd	D parameter.

PIDController::PIDController( float kp, float ki, float kd )
	: m_P( kp ), m_I( ki ), m_D( kd )
{
	m_E0 = 0.f;
	m_E1 = 0.f;
	m_E2 = 0.f;
	m_ControlValue = 0.f;
}


/********************************************************************************************************************/
/*																													*/
/*																													*/
/********************************************************************************************************************/

/// @param	sv	Set value. This is the target value.
/// @param	pv	Process value. This is the measured (or current) value.
/// @param	dt	The time elapsed since the last update.

void PIDController::Update( float sv, float pv, float dt )
{
	// If the elapsed time is zero (or less) ignore this update.

	if ( dt <= 0.f )
	{
		return;
	}

//	Pk = Kp * (ek - ek-1)
//	Ik = Ki * T * ek
//	Dk = (Kd/T) * (ek - 2*ek-1 + ek-2) 
//
//	CVk = CVk-1 + Pk + Ik + Dk 
//
// where ek = SPk - PVk, and T is the sampling interval.

	// Only the last 3 error values are needed.

	m_E2 = m_E1;
	m_E1 = m_E0;
	m_E0 = sv - pv;

	float const	e01	= m_E0 - m_E1;
	float const	e12	= m_E1 - m_E2;
	float const	p	= m_P * e01;
	float const	i	= m_I * m_E0 * dt;
	float const	d	= m_D * ( e01 - e12 ) / dt;

	m_ControlValue += p + i + d; 
}
