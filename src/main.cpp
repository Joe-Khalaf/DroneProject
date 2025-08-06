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
    initCTRL();
    initCRSF();
    initPID();

    // initRFM95();
    // initGPS();
    // initUI();
    // initSD();
}

void loop() {
    updateCTRL();
    updateSensors();
    updateCRSF();

    // updateLEDs();
    // updateGPS();
    // updateRFM95();
    // updateUI();
    // updateSD();

    delay(100);
}
