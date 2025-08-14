// sensors.cpp
#include "sensors.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <MPU6500_WE.h>
#include <math.h>

// ——— Global gyro readings for PID ———————————————————
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

// Gyro bias offsets (measured at startup)
static float gyroXOffset = 0;
static float gyroYOffset = 0;
static float gyroZOffset = 0;

// ——— Gyro Smoothing Config ——————————————————————
constexpr int GYRO_AVG_WINDOW = 1;
static float gyroXBuffer[GYRO_AVG_WINDOW] = {0};
static float gyroYBuffer[GYRO_AVG_WINDOW] = {0};
static float gyroZBuffer[GYRO_AVG_WINDOW] = {0};
static int gyroIndex = 0;

// ——— Sensor objects ————————————————————————————————
static Adafruit_BME280 bme;
static MPU6500_WE     mpu(MPU6500_ADDRESS);

// ——— Constants ——————————————————————————————————————
constexpr float RAD_TO_DEGREES = 57.295779513F;

// ——— Internal roll/pitch state ———————————————————————
static float currentPitch = 0.0f;
static float currentRoll  = 0.0f;

void initSensors() {
  Wire.begin();

  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println(F("ERROR: BME280 not found!"));
    while (1);
  }

  if (!mpu.init()) {
    Serial.println(F("ERROR: MPU6500 not found!"));
    while (1);
  }

  // Let the library calibrate offsets in the hardware
  mpu.autoOffsets();

  // Optional filtering & sample rate
  mpu.enableGyrDLPF();
  mpu.setSampleRateDivider(5);

  // Full-scale ranges
  mpu.setGyrRange(MPU6500_GYRO_RANGE_250);  // ±250 °/s
  mpu.setAccRange(MPU6500_ACC_RANGE_4G);     // ±4 g

  delay(500);  // Allow sensor to settle

  // xyzFloat bias = mpu.getGyrValues();
  // gyroXOffset = bias.x;
  // gyroYOffset = bias.y;
  // gyroZOffset = bias.z;
}

void updateSensors() {
  // —— Environmental reads ——
  float temp     = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(1013.25);

  // —— IMU reads ——
  // Acceleration in G (scaled by library)
  xyzFloat accRaw = mpu.getGValues();

  // Gyro rates in °/s (scaled by library)
  xyzFloat g = mpu.getGyrValues();   // already offset-corrected by autoOffsets
  gyroX = (gyroX * 4 + g.x) / 5.0f;  // keep your 5-sample average if you like
  gyroY = (gyroY * 4 + g.y) / 5.0f;
  gyroZ = (gyroZ * 4 + g.z) / 5.0f;


  // xyzFloat gyrVal = mpu.getGyrValues();
  // float gx = gyrVal.x - gyroXOffset;
  // float gy = gyrVal.y - gyroYOffset;
  // float gz = gyrVal.z - gyroZOffset;

  // // ——— Store into circular buffer ———
  // gyroXBuffer[gyroIndex] = gx;
  // gyroYBuffer[gyroIndex] = gy;
  // gyroZBuffer[gyroIndex] = gz;

  // // ——— Compute average ———
  // float sumX = 0, sumY = 0, sumZ = 0;
  // for (int i = 0; i < GYRO_AVG_WINDOW; ++i) {
  //   sumX += gyroXBuffer[i];
  //   sumY += gyroYBuffer[i];
  //   sumZ += gyroZBuffer[i];
  // }

  // gyroX = sumX / GYRO_AVG_WINDOW;
  // gyroY = sumY / GYRO_AVG_WINDOW;
  // gyroZ = sumZ / GYRO_AVG_WINDOW;

  // ——— Update buffer index ———
  gyroIndex = (gyroIndex + 1) % GYRO_AVG_WINDOW;


  // —— Convert acceleration to m/s² ——
  float ax = accRaw.x * G_TO_MPS2;
  float ay = accRaw.y * G_TO_MPS2;
  float az = accRaw.z * G_TO_MPS2;

  // —— Compute roll & pitch from accel (degrees) ——
  // currentRoll  = atan2f(accRaw.y, accRaw.z) * RAD_TO_DEGREES;
  currentPitch = atan2f(-accRaw.x, sqrtf(accRaw.y*accRaw.y + accRaw.z*accRaw.z)) * RAD_TO_DEGREES;

  // —— (Optional) Print everything out ——  
  // Serial.println(F("===== Sensor Readings ====="));
  // Serial.print(F("Temp (°C): "));  Serial.println(temp, 2);
  // Serial.print(F("Press (hPa): "));Serial.println(pressure, 1);
  // Serial.print(F("Alt  (m): "));   Serial.println(altitude, 1);
  // Serial.println();

  // Serial.println(F("Accel per axis:"));
  // Serial.print(F("  X: ")); Serial.print(accRaw.x,3); Serial.print(F(" g, "));
  //                          Serial.print(ax,2);      Serial.println(F(" m/s²"));
  // Serial.print(F("  Y: ")); Serial.print(accRaw.y,3); Serial.print(F(" g, "));
  //                          Serial.print(ay,2);      Serial.println(F(" m/s²"));
  // Serial.print(F("  Z: ")); Serial.print(accRaw.z,3); Serial.print(F(" g, "));
  //                          Serial.print(az,2);      Serial.println(F(" m/s²"));
  // Serial.println();

  // Serial.print(F("Roll (°): "));  Serial.println(currentRoll,1);
  // Serial.print(F("Pitch(°): "));  Serial.println(currentPitch,1);
  // Serial.println();

  // Serial.print(F("Gyro °/s [X,Y,Z]: "));
  // Serial.print(gyroX,1); Serial.print(F(", "));
  // Serial.print(gyroY,1); Serial.print(F(", "));
  // Serial.println(gyroZ,1);
  // Serial.println();
}

float getPitch() { return currentPitch; }
float getRoll()  { return currentRoll; }
