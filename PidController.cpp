#include "PIDController.h"

//! @param	kp	P parameter.
//! @param	ki	I parameter.
//! @param	kd	D parameter.

PIDController::PIDController(float kp, float ki, float kd)
    : p_(kp)
    , i_(ki)
    , d_(kd)
{
    e0_           = 0.f;
    e1_           = 0.f;
    e2_           = 0.f;
    controlValue_ = 0.f;
}

//! @param	sv	Set value. This is the target value.
//! @param	pv	Process value. This is the measured (or current) value.
//! @param	dt	The time elapsed since the last update.

void PIDController::Update(float sv, float pv, float dt)
{
    // If the elapsed time is zero (or less) ignore this update.

    if (dt <= 0.f)
        return;

//	Pk = Kp * (ek - ek-1)
//	Ik = Ki * T * ek
//	Dk = (Kd/T) * (ek - 2*ek-1 + ek-2)
//
//	CVk = CVk-1 + Pk + Ik + Dk
//
// where ek = SPk - PVk, and T is the sampling interval.

    // Only the last 3 error values are needed.

    e2_ = e1_;
    e1_ = e0_;
    e0_ = sv - pv;

    float e01 = e0_ - e1_;
    float e12 = e1_ - e2_;
    float p   = p_ * e01;
    float i   = i_ * e0_ * dt;
    float d   = d_ * (e01 - e12) / dt;

    controlValue_ += p + i + d;
}
