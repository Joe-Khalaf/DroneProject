#include <Arduino.h>
#include <Servo.h>

#define BUZZER_PIN     2   // Gate of N-MOSFET controlling passive buzzer
#define LED_GREEN_PIN  4   // Your green status LED

Servo esc;

void setup() {
  // — Pin & serial setup —
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  // Ensure green LED in “off” state
  analogWrite(LED_GREEN_PIN, 255);
  delay(500);

  // — Do-do-do-do-do … do-do jingle —
  // 5 short “do”s (200 ms beep + 200 ms gap)
  for (int i = 0; i < 5; i++) {
    analogWrite(LED_GREEN_PIN, 225);
    tone(BUZZER_PIN, 2000);
    delay(200);
    noTone(BUZZER_PIN);
    analogWrite(LED_GREEN_PIN, 255);
    delay(200);
  }
  // Pause before the final two
  delay(500);
  // 2 longer “do”s (400 ms beep + 400 ms gap)
  for (int i = 0; i < 2; i++) {
    analogWrite(LED_GREEN_PIN, 225);
    tone(BUZZER_PIN, 2000);
    delay(400);
    noTone(BUZZER_PIN);
    analogWrite(LED_GREEN_PIN, 255);
    delay(400);
  }

  // Hold green LED on while we finish boot
  analogWrite(LED_GREEN_PIN, 225);
  while (!Serial) { /* wait for USB-Serial */ }
  Serial.println("Booting...");
  digitalWrite(LED_BUILTIN, HIGH);

  // — Initialize ESC on pin 10 —
  esc.attach(10);
  esc.writeMicroseconds(1000);  // zero throttle
}

void loop() {
  // Blink the onboard LED at 2 Hz
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  // ESC throttle bump
  esc.writeMicroseconds(1200);
  delay(1000);
  esc.writeMicroseconds(1000);
  delay(1000);
}
