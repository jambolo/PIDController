#if !defined(PIDCONTROLLER_H_INCLUDED)
#define PIDCONTROLLER_H_INCLUDED

#pragma once

//! A PID controller.

class PIDController
{
public:

    PIDController(float kp, float ki, float kd);

    //! Updates the controller.
    void Update(float sv, float pv, float dt);

    //! Returns the current control value.
    float GetControlValue() const { return controlValue_; }

private:

    float controlValue_;    // Current control value
    float p_;               // P parameter
    float i_;               // I parameter
    float d_;               // D parameter
    float e0_, e1_, e2_;    // Error history
};

#endif // !defined(PIDCONTROLLER_H_INCLUDED)
