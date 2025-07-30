#ifndef CRSF_H
#define CRSF_H

#include <Arduino.h>

extern uint16_t rcChannels[16];

void initCRSF();
void updateCRSF();
void parseCRSF(uint8_t *data, uint8_t len);

#endif

