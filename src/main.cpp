#include "Arduino.h"
#include "sensors/sensors.h"
#include "telemetry/rfm95.h"
#include "gps/ublox_gps.h"
#include "indicators/led_buzzer.h"
#include "control/pwm_esc.h"
#include "elrs_crsf/crsf.h"
#include "ui/ui.h"

void setup() {
    initLEDs();
    initSensors();
    initRFM95();
    initGPS();
    initPWM();
    initCRSF();
    initUI();
}

void loop() {
    updateLEDs();
    updateSensors();
    updateGPS();
    updateRFM95();
    updatePWM();
    updateCRSF();
    updateUI();
    delay(100);
}
