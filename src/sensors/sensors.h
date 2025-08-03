#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_BME280.h>
#include <MPU6500_WE.h>

void initSensors();
void updateSensors();  // <--- Add this line

#endif
