#include "sensors.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <MPU6500_WE.h>
#include <math.h>

// ——— Sensor objects ————————————————————————————————
static Adafruit_BME280 bme;
static MPU6500_WE     mpu(MPU6500_ADDRESS);

// ——— Constants ——————————————————————————————————————
constexpr float RAD_TO_DEGREES = 57.295779513F;

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
    mpu.autoOffsets();
    mpu.enableGyrDLPF();
    mpu.setSampleRateDivider(5);
    mpu.setGyrRange(MPU6500_GYRO_RANGE_250);
    mpu.setAccRange(MPU6500_ACC_RANGE_4G);
}

void updateSensors() {
    // —— Environmental reads ——
    float temp     = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0F;
    float altitude = bme.readAltitude(1013.25);

    // —— IMU reads ——
    xyzFloat accRaw = mpu.getGValues();    // Raw in Gs
    xyzFloat gyro   = mpu.getGyrRawValues();    // Angular velocity in °/s

    // —— Convert acceleration to m/s² ——
    float ax_mps2 = accRaw.x * G_TO_MPS2;
    float ay_mps2 = accRaw.y * G_TO_MPS2;
    float az_mps2 = accRaw.z * G_TO_MPS2;

    // —— Calculate magnitude of acceleration ——
    float accelMag_mps2 = sqrtf(ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2);
    float accelMag_g    = accelMag_mps2 / G_TO_MPS2;

    // —— Roll & Pitch from acceleration (degrees) ——
    float roll  = atan2f(accRaw.y, accRaw.z) * RAD_TO_DEGREES;
    float pitch = atan2f(-accRaw.x, sqrtf(accRaw.y * accRaw.y + accRaw.z * accRaw.z)) * RAD_TO_DEGREES;

    // —— Print output ——
    Serial.println(F("===== Sensor Readings ====="));
    
    // Environmental
    Serial.print(F("Temp     (°C): "));  Serial.println(temp, 2);
    Serial.print(F("Pressure (hPa): ")); Serial.println(pressure, 1);
    Serial.print(F("Altitude   (m): ")); Serial.println(altitude, 1);
    Serial.println();

    // Acceleration per axis
    Serial.println(F("Accel per axis:"));
    Serial.print  (F("  X: ")); Serial.print(accRaw.x, 3); Serial.print(F(" g, "));
                        Serial.print(ax_mps2, 2); Serial.println(F(" m/s²"));
    Serial.print  (F("  Y: ")); Serial.print(accRaw.y, 3); Serial.print(F(" g, "));
                        Serial.print(ay_mps2, 2); Serial.println(F(" m/s²"));
    Serial.print  (F("  Z: ")); Serial.print(accRaw.z, 3); Serial.print(F(" g, "));
                        Serial.print(az_mps2, 2); Serial.println(F(" m/s²"));
    Serial.println();

    // Magnitude
    Serial.print(F("Accel Magnitude: ")); Serial.print(accelMag_g, 2); Serial.print(F(" g, "));
    Serial.print(accelMag_mps2, 2); Serial.println(F(" m/s²"));
    Serial.println();

    // Orientation
    Serial.print(F("Roll     (°): ")); Serial.println(roll, 1);
    Serial.print(F("Pitch    (°): ")); Serial.println(pitch, 1);
    Serial.println();

    // Gyroscope
    Serial.print(F("Gyro °/s [X,Y,Z]: "));
    Serial.print(gyro.x, 1); Serial.print(F(", "));
    Serial.print(gyro.y, 1); Serial.print(F(", "));
    Serial.println(gyro.z, 1);
    Serial.println();
}




