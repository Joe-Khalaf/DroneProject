#include "Arduino.h"
#include "sensors/sensors.h"
#include "telemetry/rfm95.h"
#include "gps/ublox_gps.h"
#include "indicators/led_buzzer.h"
#include "control/controller.h"
#include "elrs_crsf/crsf.h"
#include "ui/ui.h"
#include "sd/sd.h"
#include "control/pid.h"

void setup() {
    initLEDs();
    initSensors();
    initRFM95();
    initGPS();
    initCTRL();
    initCRSF();
    initUI();
    initSD();
    initPID();
}

void loop() {
    updateLEDs();
    updateSensors();
    updateGPS();
    updateRFM95();
    updateCTRL();
    updateCRSF();
    updateUI();
    updateSD();
    updatePID();
    delay(50);
}
