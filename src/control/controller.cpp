#include <Arduino.h>
#include <Servo.h>
#include "controller.h"
#include "sensors.h"  // getRoll(); extern float gyroX;
#include "pid.h"

extern float gyroY; // from sensors.cpp

Servo escLeft;
Servo escRight;

unsigned long lastTime;
float setpoint = 0.0;
int baseThrottle = 1400;

void initCTRL() {
    escLeft.attach(0);
    escRight.attach(1);

    escLeft.writeMicroseconds(1000);
    escRight.writeMicroseconds(1000);
    delay(3000);

    lastTime = millis();
    pidPitch.reset();
    setpoint = getPitch();

}

void updateCTRL() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;   // avoid zero/negative first tick
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

// --- pretty debug print (readable + motor thrust) ---
    static unsigned long lastHdr = 0;

// Compute motor deltas and % of 1000–2000 µs range
    int   dL = leftPWM  - baseThrottle;
    int   dR = rightPWM - baseThrottle;
    float leftPct  = (leftPWM  - 1000) * 0.1f;  // 1000..2000 -> 0..100 %
    float rightPct = (rightPWM - 1000) * 0.1f;

    unsigned long nowMs = millis();
    if (nowMs - lastHdr > 1500) {
    Serial.println(F("  e     P      I      D     out  |  ang(deg)  rate(dps)  |  L(µs)  ΔL  %L   R(µs)  ΔR  %R"));
    lastHdr = nowMs;
}

    Serial.print(pidPitch.error(), 2);   Serial.print(' ');
    Serial.print(pidPitch.pTerm(), 2);   Serial.print(' ');
    Serial.print(pidPitch.iTerm(), 2);   Serial.print(' ');
    Serial.print(pidPitch.dTerm(), 2);   Serial.print(' ');
    Serial.print(pidPitch.output(), 2);  Serial.print("  |  ");

    Serial.print(angle, 1);              Serial.print("       ");
    Serial.print(rate, 1);               Serial.print("       |  ");

    Serial.print(leftPWM);               Serial.print("   ");
    if (dL >= 0) Serial.print('+');      Serial.print(dL);  Serial.print("  ");
    Serial.print(leftPct, 0);            Serial.print("%   ");

    Serial.print(rightPWM);              Serial.print("   ");
    if (dR >= 0) Serial.print('+');      Serial.print(dR);  Serial.print("  ");
    Serial.print(rightPct, 0);           Serial.println('%');


}
