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

  // Debug: print first 4 channels
  Serial.print("CH0="); Serial.print(rcChannels[0]);
  Serial.print(" CH1="); Serial.print(rcChannels[1]);
  Serial.print(" CH2(th)="); Serial.print(rcChannels[2]);
  Serial.print(" CH3="); Serial.print(rcChannels[3]);

  // —— THROTTLE → PWM ——
  // Map raw 11-bit throttle (≈172–1811) → 1000–2000 µs
  int raw   = rcChannels[2];
  int pulse = map(raw, 172, 1811, 1000, 2000);
  pulse = constrain(pulse, 1000, 2000);

  esc0.writeMicroseconds(pulse);
  esc1.writeMicroseconds(pulse);
  esc2.writeMicroseconds(pulse);
  esc3.writeMicroseconds(pulse);

  Serial.print("  PWM(us)="); Serial.println(pulse);
}

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