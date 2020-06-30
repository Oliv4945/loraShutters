#ifndef configuration_h
#define configuration_h

#include <stdint.h>
#include <Arduino.h>
#include "hal/hal.h"
#include "lmic.h"
#include "LowPower.h"

// Defines
#define USE_WATCHDOG

// Enum


// Constants
#define PIN_VCC A3
#define PIN_SHUTTER_DOWN 4
#define PIN_SHUTTER_UP 5
const uint8_t lppChannelVoltage = 0;
const uint16_t default_sleep_between_tx = 2; // 5 * 60;
const float battery_calibration = 0.00723496999;
const float full_shutter_movement = 23;
const period_t time_remote_button_pressed = SLEEP_120MS;

// Pins
const uint8_t pinSx1276NSS  = 10;

// Structures


// LoRaWAN configuration
// Little-endian format, LSB first
// For TTN issued APPEUI, the last bytes should be 0xD5, 0xB3,0x70.
static const u1_t PROGMEM APPEUI[8] = { };
static const u1_t PROGMEM DEVEUI[8] = { };
// Big endian format
static const u1_t PROGMEM APPKEY[16] = { };
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = pinSx1276NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {2, 2, 2}
};

#endif