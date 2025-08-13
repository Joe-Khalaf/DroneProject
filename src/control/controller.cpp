#include <Arduino.h>
#include <Servo.h>
#include "controller.h"
#include "sensors.h"  // getRoll(); extern float gyroX;
#include "pid.h"

extern float gyroY; // from sensors.cpp

Servo escLeft;
Servo escRight;

unsigned long lastTime;
float setpoint = 0.0; // hold level
int baseThrottle = 1200;

void initCTRL() {
    escLeft.attach(0);   // change pin if needed
    escRight.attach(1);  // change pin if needed

    escLeft.writeMicroseconds(1000);
    escRight.writeMicroseconds(1000);
    delay(3000);

    lastTime = millis();
    pidPitch.reset();
}

void updateCTRL() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    float angle = getPitch();
    float rate  = gyroY;

    float output = pidPitch.compute(setpoint, angle, rate, dt);

    int leftPWM  = baseThrottle - output;
    int rightPWM = baseThrottle + output;

    leftPWM  = leftPWM  < 1000 ? 1000 : (leftPWM  > 2000 ? 2000 : leftPWM);
    rightPWM = rightPWM < 1000 ? 1000 : (rightPWM > 2000 ? 2000 : rightPWM);


    escLeft.writeMicroseconds(leftPWM);
    escRight.writeMicroseconds(rightPWM);

    Serial.print("Angle: "); Serial.print(angle);
    Serial.print("  Rate: "); Serial.print(rate);
    Serial.print("  Output: "); Serial.println(output);

}
