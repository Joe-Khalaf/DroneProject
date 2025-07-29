#include <Arduino.h>
#include <Servo.h>

#define CRSF_ADDRESS             0xC8
#define CRSF_FRAME_RC_CHANNELS   0x16
#define CRSF_MAX_CHANNELS        16

//–– Globals for CRSF parsing
uint8_t  crsfPacket[64];
uint8_t  packetIndex = 0;
bool     receiving   = false;
uint16_t rcChannels[CRSF_MAX_CHANNELS];

//–– Servo for ESC on pin 6
Servo esc0, esc1, esc2, esc3;

void setup() {
  Serial.begin(115200);
  Serial4.begin(420000);

  // attach ESC signal pin
  esc0.attach(0);  
  esc1.attach(1);
  esc2.attach(25);
  esc3.attach(33);

  Serial.println("🔧 ELRS + PWM 4 Motor Test Starting…");
}

//— parse a complete CRSF frame
void parseCRSF(uint8_t *data, uint8_t len) {
  if (data[2] != CRSF_FRAME_RC_CHANNELS) return;

  // unpack 16 channels, 11 bits each
  uint8_t  *payload = data + 3;
  uint32_t bits     = 0;
  uint8_t  bitCount = 0, ch = 0;
  for (int i = 0; i < 22 && ch < CRSF_MAX_CHANNELS; i++) {
    bits     |= uint32_t(payload[i]) << bitCount;
    bitCount += 8;
    while (bitCount >= 11 && ch < CRSF_MAX_CHANNELS) {
      rcChannels[ch++] = bits & 0x7FF;
      bits >>= 11;
      bitCount -= 11;
    }
  }
  //Print out all 4 channels
  Serial.print("CH0="); Serial.print(rcChannels[0]);
  Serial.print(" CH1="); Serial.print(rcChannels[1]);
  Serial.print(" CH2(th)="); Serial.print(rcChannels[2]);
  Serial.print(" CH3="); Serial.print(rcChannels[3]);

  // Remap raw values of each channel
 int roll     = map(rcChannels[0], 172, 1811, -500, 500);   // CH0
 int pitch    = map(rcChannels[1], 172, 1811, -500, 500);   // CH1
 int throttle = map(rcChannels[2], 172, 1811, 1000, 2000);  // CH2
 int yaw      = map(rcChannels[3], 172, 1811, -500, 500);   // CH3

 //Motor directional combinations **CHOOSE CORRECT PINS IN HARDWARE FOR EACH**
 int m0 = throttle + pitch + roll - yaw; // esc0 = Front Left  (CW)
 int m1 = throttle + pitch - roll + yaw; // esc1 = Front Right (CCW)
 int m2 = throttle - pitch - roll - yaw; // esc2 = Rear Right  (CW)
 int m3 = throttle - pitch + roll + yaw; // esc3 = Rear Left   (CCW)

 //Constraints so ESC's dont blow up
 m0 = constrain(m0, 1000, 2000);
 m1 = constrain(m1, 1000, 2000);
 m2 = constrain(m2, 1000, 2000);
 m3 = constrain(m3, 1000, 2000);

 //PWM signals to each ESC to motors
 esc0.writeMicroseconds(m0);
 esc1.writeMicroseconds(m1);
 esc2.writeMicroseconds(m2);
 esc3.writeMicroseconds(m3);

 //Make sure we can see it
 Serial.print("  M0="); Serial.print(m0);
 Serial.print(" M1="); Serial.print(m1);
 Serial.print(" M2="); Serial.print(m2);
 Serial.print(" M3="); Serial.println(m3);
}

//This loop basically says "is there new information? Okay lets parse it and return" its like a software based interrupt but with a UART signal instead of pin signals
void loop() {
  // Read and frame bytes from Serial4
  while (Serial4.available()) {
    uint8_t b = Serial4.read();

    if (!receiving) {
      if (b == CRSF_ADDRESS) {
        receiving   = true;
        packetIndex = 0;
        crsfPacket[packetIndex++] = b;
      }
    } else {
      crsfPacket[packetIndex++] = b;
      // once we've collected (length + 2) bytes, parse
      if (packetIndex > 2 && packetIndex >= crsfPacket[1] + 2) {
        receiving = false;
        parseCRSF(crsfPacket, crsfPacket[1] + 2);
      }
    }
  }
}