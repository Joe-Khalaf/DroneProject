#include "pid.h"

PID::PID(float kp_, float ki_, float kd_)
: kp(kp_), ki(ki_), kd(kd_), integral(0.0f) {}

float PID::compute(float targetAngle, float currentAngle, float currentRate, float dt_sec) {
    if (dt_sec <= 0.0f) dt_sec = 0.001f;     // dt guard
    float error  = targetAngle - currentAngle;
    integral    += error * dt_sec;
    float Pterm  = kp * error;
    float Iterm  = ki * integral;
    float Dterm  = -kd * currentRate;        // D from gyro rate (damping)
    return Pterm + Iterm + Dterm;
}

void PID::reset() { integral = 0.0f; }

PID pidPitch(0.4f, 0.0f, 0.6f);  // start here; adjust as you’ve been doing

