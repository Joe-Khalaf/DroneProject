#pragma once

/**
 * @file pid.h
 * @brief PID controller class and helper functions for roll, pitch, and yaw.
 */

// ——— PID Controller Class —————————————————————————————————
class PID {
public:
  /**
   * @brief Construct a PID controller with given gains.
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  PID(float kp, float ki, float kd);

  /**
   * @brief Compute the PID output
   * @param target Desired setpoint (e.g., rate in °/s)
   * @param actual Current measurement (e.g., gyro rate in °/s)
   * @return Control output (e.g., PWM correction)
   */
  float compute(float target, float actual);

  /**
   * @brief Reset internal PID state (integrator and previous error)
   */
  void reset();

private:
  float kp;
  float ki;
  float kd;
  float prevError;
  float integral;
};

// ——— Global PID Instances —————————————————————————————————
extern PID pidRoll;
extern PID pidPitch;
extern PID pidYaw;

// ——— Initialization ————————————————————————————————————
/**
 * @brief Initialize (reset) all PID controllers. Call once in setup().
 */
void initPID();

// ——— Axis-specific Correction Helpers —————————————————————
/**
 * @brief Compute roll-axis correction
 * @param targetRate Desired roll rate (°/s)
 * @param currentRate Measured roll rate (°/s)
 * @return PID correction for roll
 */
float getRollCorrection(float targetRate, float currentRate);

/**
 * @brief Compute pitch-axis correction
 * @param targetRate Desired pitch rate (°/s)
 * @param currentRate Measured pitch rate (°/s)
 * @return PID correction for pitch
 */
float getPitchCorrection(float targetRate, float currentRate);

/**
 * @brief Compute yaw-axis correction
 * @param targetRate Desired yaw rate (°/s)
 * @param currentRate Measured yaw rate (°/s)
 * @return PID correction for yaw
 */
float getYawCorrection(float targetRate, float currentRate);



