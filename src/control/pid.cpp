#include "pid.h"

PID::PID(float kp_, float ki_, float kd_)
: kp(kp_), ki(ki_), kd(kd_), integral(0.0f),
  pTerm_(0), iTerm_(0), dTerm_(0), error_(0), output_(0) {}

float PID::compute(float targetAngle, float currentAngle, float currentRate, float dt_sec) {
    if (dt_sec <= 0.0f) dt_sec = 0.001f;

    error_ = targetAngle - currentAngle;

    // Integral (time-scaled)
    integral += error_ * dt_sec;

    // Terms (store for printing)
    pTerm_ = kp * error_;
    iTerm_ = ki * integral;
    dTerm_ = - kd * currentRate;     // D from gyro rate (damping)

    float out = pTerm_ + iTerm_ + dTerm_;

    // Optional controller clamp + light anti-windup backoff
    const float OUT_CLAMP = 400.0f;
    if (out >  OUT_CLAMP) { out =  OUT_CLAMP; integral -= error_ * dt_sec; }
    if (out < -OUT_CLAMP) { out = -OUT_CLAMP; integral -= error_ * dt_sec; }

    output_ = out;                  // store final output for printing
    return output_;
}

void PID::reset() { integral = 0.0f; pTerm_=iTerm_=dTerm_=error_=output_=0.0f; }

// Example gains (tune these)
PID pidPitch(0.40f, 0.10f, 0.50f);


