// pid.cpp
#include "pid.h"

// ——— PID Class Method Definitions —————————————————————

PID::PID(float kp_, float ki_, float kd_)
  : kp(kp_), ki(ki_), kd(kd_), prevError(0), integral(0) {}

float PID::compute(float target, float actual) {
  float error     = target - actual;
  integral       += error;
  float derivative = error - prevError;
  prevError       = error;
  return kp * error + ki * integral + kd * derivative;
}

void PID::reset() {
  prevError = 0;
  integral  = 0;
}

// ——— Global PID Instances ————————————————————————————

PID pidRoll(1.2f, 0.0f, 0.03f);
PID pidPitch(1.2f, 0.0f, 0.03f);
PID pidYaw(1.0f, 0.0f, 0.02f);

// ——— Initialization —————————————————————————————————

void initPID() {
  pidRoll.reset();
  pidPitch.reset();
  pidYaw.reset();
}

// ——— Axis-specific Helpers ——————————————————————————

float getRollCorrection(float targetRate, float currentRate) {
  return pidRoll.compute(targetRate, currentRate);
}

float getPitchCorrection(float targetRate, float currentRate) {
  return pidPitch.compute(targetRate, currentRate);
}

float getYawCorrection(float targetRate, float currentRate) {
  return pidYaw.compute(targetRate, currentRate);
}

