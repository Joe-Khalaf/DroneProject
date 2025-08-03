#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_BME280.h>
#include <MPU6500_WE.h>

#pragma once

void initSensors();
void updateSensors();  // <--- Add this line

extern float gyroX;
extern float gyroY;
extern float gyroZ;

float getPitch();
float getRoll();

#endif
