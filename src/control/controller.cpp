// controller.cpp
// Reads RC inputs, applies cascade PID (angle + rate loops), mixes motor outputs, and drives ESCs.

#include <Arduino.h>
#include <Servo.h>
#include "controller.h"
#include "elrs_crsf/crsf.h"
#include "sensors.h"
#include "pid.h"

// ——— Constants ————————————————————————————————
static const int ESC_MIN    = 1050;  // ESC idle PWM
static const int ESC_MAX    = 2000;  // ESC max PWM
static const float MAX_ANGLE_CMD = 30.0f; // stick deflection → ±30° tilt

// ——— ESC Objects ————————————————————————————
static Servo esc0, esc1, esc2, esc3;

// ——— Angle-loop PID Controllers ————————————————————
// Tune these for smooth self-leveling response
static PID anglePIDRoll(4.0f, 0.0f, 0.2f);
static PID anglePIDPitch(4.0f, 0.0f, 0.2f);

/**
 * @brief Initialize ESCs and reset all PID states
 */
void initCTRL() {
  esc0.attach(0);   // Front Left  (CW)
  esc1.attach(1);   // Front Right (CCW)
  esc2.attach(25);  // Rear Right  (CW)
  esc3.attach(33);  // Rear Left   (CCW)

  initPID();           // rate PIDs from pid.cpp
  anglePIDRoll.reset();  // reset angle-loop PIDs
  anglePIDPitch.reset();
}

/**
 * @brief Called each loop: reads RC + sensors, computes cascade PID, and updates motors
 */
void updateCTRL() {
  // 1. Read and map RC inputs
  int rawRoll     = rcChannels[0];
  int rawPitch    = rcChannels[1];
  int rawThrottle = rcChannels[2];
  int rawYaw      = rcChannels[3];

  // Desired angles from stick for self-leveling (outer loop)
  float desiredRollAng  = map(rawRoll,     172, 1811, -MAX_ANGLE_CMD,  MAX_ANGLE_CMD);
  float desiredPitchAng = map(rawPitch,    172, 1811, -MAX_ANGLE_CMD,  MAX_ANGLE_CMD);

  // Desired yaw rate (no self-heading control) and throttle
  float desiredYawRate  = map(rawYaw,      172, 1811, -50.0f,         50.0f);  // ±50°/s
  int   throttlePW      = map(rawThrottle, 172, 1811, 1000,          2000);

  // 2. Read sensor feedback
  float rollAngle  = getRoll();   // accel-based absolute angle (°)
  float pitchAngle = getPitch();  
  float rollRate   = gyroX;       // gyro-based rate (°/s)
  float pitchRate  = gyroY;
  float yawRate    = gyroZ;

  // 3. Outer loop: angle PID → rate targets
  float rollRateTarget  = anglePIDRoll.compute(desiredRollAng,  rollAngle);
  float pitchRateTarget = anglePIDPitch.compute(desiredPitchAng, pitchAngle);

  // 4. Inner loop: rate PID → corrections
  float rollCorr  = getRollCorrection(rollRateTarget,  rollRate);
  float pitchCorr = getPitchCorrection(pitchRateTarget, pitchRate);
  float yawCorr   = getYawCorrection(desiredYawRate,  yawRate);

  // 5. Mix motors (throttle + rate corrections)
  int m0 = throttlePW + int(pitchCorr +  rollCorr - yawCorr); // Front Left
  int m1 = throttlePW + int(pitchCorr -  rollCorr + yawCorr); // Front Right
  int m2 = throttlePW + int(-pitchCorr - rollCorr - yawCorr); // Rear Right
  int m3 = throttlePW + int(-pitchCorr + rollCorr + yawCorr); // Rear Left

  // 6. Constrain and output
  m0 = constrain(m0, ESC_MIN, ESC_MAX);
  m1 = constrain(m1, ESC_MIN, ESC_MAX);
  m2 = constrain(m2, ESC_MIN, ESC_MAX);
  m3 = constrain(m3, ESC_MIN, ESC_MAX);
  esc0.writeMicroseconds(m0);
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);

  // 7. Debugging
  Serial.print("AngCmd R/P:"); Serial.print(desiredRollAng,1); Serial.print("/"); Serial.print(desiredPitchAng,1);
  Serial.print(" AngFB R/P:"); Serial.print(rollAngle,1);    Serial.print("/"); Serial.print(pitchAngle,1);
  Serial.print(" RTgt R/P:"); Serial.print(rollRateTarget,1);Serial.print("/"); Serial.print(pitchRateTarget,1);
  Serial.print(" RCorr R/P:");Serial.print(rollCorr,1);     Serial.print("/"); Serial.print(pitchCorr,1);
  Serial.print(" YCmd:");     Serial.print(desiredYawRate,1);
  Serial.print(" YCorr:");    Serial.print(yawCorr,1);
  Serial.print(" Thr:");      Serial.print(throttlePW);
  Serial.print(" M0:");       Serial.print(m0);
  Serial.print(" M1:");       Serial.print(m1);
  Serial.print(" M2:");       Serial.print(m2);
  Serial.print(" M3:");       Serial.println(m3);
}
