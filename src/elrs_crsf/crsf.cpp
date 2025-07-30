#include <Arduino.h>
#include "crsf.h"

#define CRSF_ADDRESS             0xC8
#define CRSF_FRAME_RC_CHANNELS   0x16
#define CRSF_MAX_CHANNELS        16

static uint8_t  crsfPacket[64];
static uint8_t  packetIndex = 0;
static bool     receiving   = false;
uint16_t rcChannels[CRSF_MAX_CHANNELS];

void initCRSF() {
  Serial4.begin(420000);
}

void updateCRSF() {
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

      if (packetIndex > 2 && packetIndex >= crsfPacket[1] + 2) {
        receiving = false;
        parseCRSF(crsfPacket, crsfPacket[1] + 2);
      }
    }
  }
}

void parseCRSF(uint8_t *data, uint8_t len) {
  if (data[2] != CRSF_FRAME_RC_CHANNELS) return;

  uint8_t  *payload = data + 3;
  uint32_t bits     = 0;
  uint8_t  bitCount = 0, ch = 0;

  for (int i = 0; i < 22 && ch < CRSF_MAX_CHANNELS; i++) {
    bits |= uint32_t(payload[i]) << bitCount;
    bitCount += 8;

    while (bitCount >= 11 && ch < CRSF_MAX_CHANNELS) {
      rcChannels[ch++] = bits & 0x7FF;
      bits >>= 11;
      bitCount -= 11;
    }
  }

  // Debug print
  Serial.print("CH0="); Serial.print(rcChannels[0]);
  Serial.print(" CH1="); Serial.print(rcChannels[1]);
  Serial.print(" CH2="); Serial.print(rcChannels[2]);
  Serial.print(" CH3="); Serial.println(rcChannels[3]);
}

