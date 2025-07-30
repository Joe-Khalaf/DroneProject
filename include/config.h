#ifndef CONFIG_H
#define CONFIG_H

// === Sensor Addresses ===
#define BME280_ADDRESS     0x76
#define MPU6500_ADDRESS    0x68

// === LoRa RFM95 ===
#define RFM95_CS     37
#define RFM95_RST    35
#define RFM95_DIO0   30
#define RFM95_FREQ   915.0

// === SPI Pins ===
#define SPI_SCK      13
#define SPI_MOSI     11
#define SPI_MISO     12

// === GPS ===
#define GPS_RX_PIN   29
#define GPS_TX_PIN   28

// === Status LED and Buzzer ===
#define PIN_LED_R    3
#define PIN_LED_G    4
#define PIN_LED_B    5
#define PIN_BUZZER   2

// === Pyro Channels ===
#define PYRO1_PIN    8
#define PYRO2_PIN    7

// === Other Constants ===
#define G_TO_MPS2    9.81
#define TELEMETRY_INTERVAL_MS 250

#endif
