#pragma once

class PID {
public:
    PID(float kp_, float ki_, float kd_);
    float compute(float targetAngle, float currentAngle, float currentRate, float dt_sec);
    void reset();

private:
    float kp, ki, kd;
    float integral;
};

// Global roll PID instance
extern PID pidPitch;
