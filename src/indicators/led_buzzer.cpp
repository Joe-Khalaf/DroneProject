#include <Arduino.h>
#include "led_buzzer.h"
#include "config.h"

void initLEDs() {
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);

  // Blink orange LED (R + G) and beep buzzer 3 times
  for (int i = 0; i < 3; i++) {
    tone(PIN_BUZZER, 1000);                  // Square wave at 3kHz
    analogWrite(PIN_LED_R, 125);             // Half brightness red
    analogWrite(PIN_LED_G, 225);             // Half brightness green (orange)
    delay(200);

    noTone(PIN_BUZZER);                      // Stop buzzer
    analogWrite(PIN_LED_R, 255);               // Turn off red
    analogWrite(PIN_LED_G, 255);               // Turn off green
    delay(200);

  analogWrite(PIN_LED_R, 125);             // Half brightness red
  analogWrite(PIN_LED_G, 225);             // Half brightness green (orange)
  }
}

void updateLEDs() {
  // Optional: implement heartbeat blink, status LED, etc.
}

