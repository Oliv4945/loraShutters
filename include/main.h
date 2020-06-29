#ifndef main_h
#define main_h

#include <stdint.h>
#include "CayenneLPP.h"
#include "lmic.h"
#include "configuration.h"

// Enumerations


// Globals
uint16_t txInterval;
uint8_t old_state_percent;

// Objects
static osjob_t sendjob;
CayenneLPP lpp(51);

// Functions
void onEvent (ev_t ev);
void do_send(osjob_t* j);
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

float readVoltage_f(int pin);
void low_power_delay_s(float txInterval);

void shutter_move( uint8_t pin, bool wait_full_movement );
void shutter_close_partially(uint8_t state_percent);
void shutter_open_partially(uint8_t state_percent);
void shutter_manage(uint8_t value);


#endif