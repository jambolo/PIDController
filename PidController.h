#pragma once

/** @file *//********************************************************************************************************

                                                   PIDController.h

						                    Copyright 2003, John J. Bolton
	--------------------------------------------------------------------------------------------------------------

	$Header: //depot/Libraries/PIDController/PIDController.h#3 $

	$NoKeywords: $

 ********************************************************************************************************************/

//! @mainpage
//! @section	Basics	PID Controller Basics
//! PID stands for Proportional-Integral-Derivative. This is a type of feedback controller whose output, a control
//! variable (CV), is generally based on the error between some user-defined set point (SP) and some measured
//! process variable (PV). Each element of the PID controller refers to a particular action taken on the error: 
//!
//! Proportional:	error multiplied by a gain, Kp. This is an adjustable amplifier. In many systems Kp is
//!					responsible for process stability: too low and the PV can drift away; too high and the PV can
//!					oscillate. 
//!
//! Integral:		the integral of error multiplied by a gain, Ki. In many systems Ki is responsible for driving
//!					error to zero, but to set Ki too high is to invite oscillation or instability or integrator
//!					windup or actuator saturation. 
//!
//! Derivative:		the rate of change of error multiplied by a gain, Kd. In many systems Kd is responsible for
//!					system response: too low and the PV will oscillate; too high and the PV will respond
//!					sluggishly. The designer should also note that derivative action amplifies any noise in the
//!					error signal.
//!
//! Tuning of a PID involves the adjustment of Kp, Ki, and Kd to achieve some user-defined "optimal" character of
//! system response. 
//!
//! Although many architectures exist for control systems, the PID controller is mature and well-understood by
//! practitioners. For these reasons, it is often the first choice for new controller design. It satisfies Occam's
//! Razor in being the simplest solution for most cases. 
//!
//! A simple digital implementation of a PID controller, in which rectangular integration is assumed, is as
//! follows: 
//! @verbatim
//!
//!	Pk = Kp * (ek - ek-1)
//!	Ik = Ki * T * ek
//!	Dk = (Kd/T) * (ek - 2*ek-1 + ek-2) 
//!
//!	CVk = CVk-1 + Pk + Ik + Dk 
//!
//!	where ek = SPk - PVk, and T is the sampling interval.  @endverbatim
//!
//! This is referred to as two-degree-of-freedom (two-DOF) design. Notice for this implementation it's assumed that
//! all three controller actions involve ek. Many practitioners try to avoid this: applying Kp and/or Kd to the set
//! point can magnify transient errors introduced by rapid changes to the SP. Consider the alternative
//! implementation, in which Kp and Kd are applied only to the PV: 
//! @verbatim
//!
//!	Pk = - Kp * (PVk - PVk-1)
//!	Ik = Ki * T * ek
//!	Dk = -(Kd/T) (PVk - 2*PVk-1 + PVk-2) @endverbatim 
//!
//! ...which is one-DOF.
//!
//!
//! Source: http://www.tcnj.edu/~rgraham/PID-tuning.html

/********************************************************************************************************************/

//! A PID controller.

class PIDController
{
public:

	//! Constructor
	PIDController( float kp, float ki, float kd );

	//! Updates the controller.
	void Update( float sv, float pv, float dt );

	//! Returns the current control value.
	float GetControlValue() const					{ return m_ControlValue; }

private:

	float	m_ControlValue;		//!< Current control value
	float	m_P;				//!< P parameter
	float	m_I;				//!< I parameter
	float	m_D;				//!< D parameter
	float	m_E0, m_E1, m_E2;	//!< Error history
};
