#include "pid.h"
#include <Arduino.h>

// Constructor definition
PID::PID(float kp, float ki, float kd, float dt) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _dt = dt;
    _prevError = 0.0f;
    _integral = 0.0f;
}

// Compute PID output
float PID::compute(float setpoint, float measured) {
    float error = setpoint - measured;
    _integral += error * _dt;
    float derivative = (error - _prevError) / _dt;
    _prevError = error;
    return _kp * error + _ki * _integral + _kd * derivative;
}

// Reset the PID internal state
void PID::reset() {
    _prevError = 0.0f;
    _integral = 0.0f;
}

// Global PID instances
PID pidRoll(0, 0, 0, 0);
PID pidPitch(0, 0, 0, 0);
PID pidYaw(0, 0, 0, 0);

void initPID() {
    // Initialize PID controllers here
}

void updatePID() {
    // Use compute() for control loops here
}
