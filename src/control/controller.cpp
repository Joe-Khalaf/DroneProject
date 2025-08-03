#include <Arduino.h>
#include <Servo.h>
#include "controller.h"
#include "elrs_crsf/crsf.h"

// ESC objects
static Servo esc0, esc1, esc2, esc3;

void initCTRL() {
  esc0.attach(0);   // Front Left  (CW)
  esc1.attach(1);   // Front Right (CCW)
  esc2.attach(25);  // Rear Right  (CW)
  esc3.attach(33);  // Rear Left   (CCW)
}

void updateCTRL() {
  // Remap raw RC values
  int roll     = map(rcChannels[0], 172, 1811, -500, 500);   // CH0
  int pitch    = map(rcChannels[1], 172, 1811, -500, 500);   // CH1
  int throttle = map(rcChannels[2], 172, 1811, 1000, 2000);  // CH2
  int yaw      = map(rcChannels[3], 172, 1811, -500, 500);   // CH3

  // Motor mixing (X configuration)
  int m0 = throttle + pitch + roll - yaw; // esc0 = Front Left
  int m1 = throttle + pitch - roll + yaw; // esc1 = Front Right
  int m2 = throttle - pitch - roll - yaw; // esc2 = Rear Right
  int m3 = throttle - pitch + roll + yaw; // esc3 = Rear Left

  // Constrain to ESC-safe PWM range
  m0 = constrain(m0, 1000, 2000);
  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);

  // Send PWM to ESCs
  esc0.writeMicroseconds(m0);
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);

  // Debug print
  Serial.print("  M0="); Serial.print(m0);
  Serial.print(" M1="); Serial.print(m1);
  Serial.print(" M2="); Serial.print(m2);
  Serial.print(" M3="); Serial.println(m3);
}

