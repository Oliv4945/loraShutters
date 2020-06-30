#include <Arduino.h>
#include <SPI.h>
#include "main.h"
#include "configuration.h"
#include "Wire.h"
#include "LowPower.h"
#include "CayenneLPP.h"
#include "lmic.h"
#include "hal/hal.h"
#include "lmic/oslmic.h"
#ifdef USE_WATCHDOG
#include <avr/wdt.h>
#endif

// Arduino setup procedure
// Called once at startup
void setup() {
#ifdef USE_WATCHDOG
    // Watchdog
    wdt_enable(WDTO_8S);
    wdt_reset();
#endif
    

    /*
    pinMode(PIN_SHUTTER_DOWN, OUTPUT);
    digitalWrite(PIN_SHUTTER_DOWN, HIGH);
    pinMode(PIN_SHUTTER_UP, OUTPUT);
    digitalWrite(PIN_SHUTTER_UP, HIGH);
    */

    pinMode(PIN_SHUTTER_DOWN, INPUT);
    pinMode(PIN_SHUTTER_UP, INPUT);

    // Serial init
    Serial.begin(57600);
    Serial.print(F("Starting\n"));

    sleep_between_tx = default_sleep_between_tx;

    // LMIC init
#ifdef USE_WATCHDOG
    wdt_reset();
#endif
    // Init LoRaWAN stack
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 3 / 100);
    // LMIC_setDrTxpow(DR_SF12, 14);
    LMIC_setAdrMode(1);
    // Start job
    do_send(&sendjob);
}

// Called by Arduino after setup()
void loop() {
    // local variables
    old_state_percent = 255;

    // Actual loop
    while (1) {

#ifdef USE_WATCHDOG
        // Clear watchdog
        wdt_reset();
#endif
        // Update mac
        os_runloop_once();
    }
}


void onEvent (ev_t ev) {
#ifdef USE_WATCHDOG
    wdt_reset();
#endif
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            /*{
              
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (uint8_t i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (uint8_t i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }*/
            // Disable link check validation.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen > 0) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
                // Fport 1
                if (LMIC.frame[LMIC.dataBeg-1] == 1) {
                    sleep_between_tx = (LMIC.frame[LMIC.dataBeg] << 8) + LMIC.frame[LMIC.dataBeg + 1];
                    Serial.print("New sleep time:");
                    Serial.println(sleep_between_tx);
                // Fport 2
                } else if (LMIC.frame[LMIC.dataBeg-1] == 2) {
                    shutter_manage(LMIC.frame[LMIC.dataBeg]);
                }
            }
            // Enter in sleep mode
            delay(1000); // TBC - Remove it, just to flush serial rx
            low_power_delay_s( (float) sleep_between_tx );

            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            // Or send now
            do_send(&sendjob);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        /*case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;*/
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}


void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        lpp.reset();
        lpp.addAnalogOutput(lppChannelVoltage, readVoltage_f(PIN_VCC));
        // LMIC_setTxData2 (u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed)
        uint8_t ackUp = false;
        LMIC_setTxData2(2, lpp.getBuffer(), lpp.getSize(), ackUp);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

float readVoltage_f(int pin) {
    float voltage;
    // Vin = analogRead(pin)*3.3/1024*2*417/406; // Read value*3.3/1024, Times 2 for resistors, *417/406 for calibration
    voltage = ((float) analogRead(pin)) * battery_calibration; // TBC - Add calibration value in configuration.h
    delay(1); // Time for ADC to settle
    voltage = ((float) analogRead(pin)) * battery_calibration;
    return voltage;
}


void shutter_move( uint8_t pin, bool wait_full_movement ) {
    delay(1000);
    pinMode( pin, OUTPUT );
    digitalWrite( pin, LOW );
    LowPower.powerDown( time_remote_button_pressed, ADC_OFF, BOD_OFF );
    digitalWrite( pin, HIGH );
    pinMode( pin, INPUT );
    if ( wait_full_movement == true ) {
        delay( full_shutter_movement * 1000 );
    }
}

void shutter_close_partially( uint8_t state_percent ) {
    // Fully close the shutter
    shutter_move( PIN_SHUTTER_DOWN, true );

    shutter_move( PIN_SHUTTER_UP, false ); // Partially open it
    delay( full_shutter_movement * state_percent / 100 * 1000 );
    shutter_move( PIN_SHUTTER_UP, false ); // Stop the movement
}

void shutter_open_partially( uint8_t state_percent ) {
    // Fully open the shutter
    shutter_move( PIN_SHUTTER_UP, true );

    shutter_move( PIN_SHUTTER_DOWN, false ); // Partially close it
    delay( full_shutter_movement * state_percent / 100 * 1000 );
    shutter_move( PIN_SHUTTER_DOWN, false ); // Partially close it
}

void shutter_manage( uint8_t state_percent ) {
    /*
    // TODO: Needs feedback from RC button to be activated
    if ( old_state_percent == state_percent ) {
        Serial.println( F("Same state") );
        return;
    }
    */
    if ( state_percent > 100 ) {
        Serial.println( F("Invalid value") );
    } else if ( state_percent == 0 ) {
        Serial.println( F( "Closing" ) );
        shutter_move(PIN_SHUTTER_DOWN, true);
    } else if ( state_percent == 100 ) {
        Serial.println( F( "Opening" ) );
        shutter_move(PIN_SHUTTER_UP, true);
    } else if ( state_percent < 50 ) {
        Serial.println( F( "Partially closing" ) );
        shutter_close_partially( state_percent );
    } else {
        Serial.println( F( "Partially opening" ) );
        shutter_open_partially( state_percent );
    }
    old_state_percent = state_percent;
    delay(1000);
}

void low_power_delay_s(float delay_float) {
    // Integer as ( tx_interval_ms - sleep_time ) can be lower than 0
    int16_t sleep_time = 0;
    int16_t tx_interval_s = (int16_t) delay_float;

    int16_t tx_interval_ms = ( delay_float - ((float) tx_interval_s) ) * 1000;
    ASSERT( tx_interval_ms < 1000 );

#ifdef USE_WATCHDOG
    wdt_reset();
    wdt_disable();
#endif
    
    while ( sleep_time < tx_interval_s ) {
        switch ( tx_interval_s - sleep_time ) {
            case 1:
                LowPower.powerDown( SLEEP_1S, ADC_OFF, BOD_OFF );
                sleep_time += 1;
                break;
            case 2:
            case 3:
                LowPower.powerDown( SLEEP_2S, ADC_OFF, BOD_OFF );
                sleep_time += 2;
                break;
            case 4:
            case 5:
            case 6:
            case 7:
                LowPower.powerDown( SLEEP_4S, ADC_OFF, BOD_OFF );
                sleep_time += 4;
                break;
            default:
                LowPower.powerDown( SLEEP_8S, ADC_OFF, BOD_OFF );
                sleep_time += 8;
        }
    }

    sleep_time = 0;
    while ( ( tx_interval_ms - sleep_time ) >= 500 ) {
        sleep_time += 500;
        LowPower.powerDown( SLEEP_500MS, ADC_OFF, BOD_OFF );
    }
    while ( ( tx_interval_ms - sleep_time ) >= 250 ) {
        sleep_time += 250;
        LowPower.powerDown( SLEEP_250MS, ADC_OFF, BOD_OFF );
    }
    while ( ( tx_interval_ms - sleep_time ) >= 120 ) {
        sleep_time += 120;
        LowPower.powerDown( SLEEP_120MS, ADC_OFF, BOD_OFF );
    }
    while ( ( tx_interval_ms - sleep_time ) >= 60 ) {
        LowPower.powerDown( SLEEP_60MS, ADC_OFF, BOD_OFF );
        sleep_time += 60;
    }
    while ( ( tx_interval_ms - sleep_time ) >=  30 ) {
        LowPower.powerDown( SLEEP_30MS, ADC_OFF, BOD_OFF );
        sleep_time += 30;
    }
    ASSERT( ( tx_interval_ms - sleep_time ) < 30 );

#ifdef USE_WATCHDOG
    wdt_enable(WDTO_4S);
    wdt_reset();
#endif
}