//
// Created by wilyarti on 21/4/20.
//

//#include "main.h"
#include <BME280I2C.h>
#include "LoraEncoder.h"
#include "Wire.h"
#include "secrets.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//#define AUTOCONFIGMODE
//#define AIRCONFIG       //Select one
//#define WATERCONFIG   //Or the other

#if (defined(AUTOCONFIGMODE) && defined(AIRCONFIG) && defined(WATERCONFIG))
#error "Please only select one autoconfig mode."
#elif (defined(AUTOCONFIGMODE) && (!defined(AIRCONFIG) && !defined(WATERCONFIG)))
#error "Please select a config mode or disable autoconfig."
#elif (!defined(AUTOCONFIGMODE) && (defined(AIRCONFIG) || defined(WATERCONFIG)))
#error "Please enable AUTOCONFIG MODE before enabling AIRCONFIG or WATERCONFIG."
#endif


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t * buf) {}

void os_getDevEui(u1_t * buf) {}

void os_getDevKey(u1_t * buf) {}

// Pin mapping
const lmic_pinmap lmic_pins = {
        .nss = 10,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 9,
        .dio = {2, 5, LMIC_UNUSED_PIN},
};

BME280I2C bme;

static osjob_t sendjob;

const int BUFFERSIZE = 26;
byte mydata[BUFFERSIZE];
int AirValue = 565; //564  //you need to replace this value with Value_1
int WaterValue = 290;  //you need to replace this value with Value_2
int soilMoistureValue = 0;
int soilmoisturepercent = 0;
boolean SENT_FLAG;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60 * 15;

long readVcc() {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0) ;
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)); // measuring

    uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    long result = (high << 8) | low;

    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result; // Vcc in millivolts
}

void do_send(osjob_t *j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
#ifdef AUTOCONFIGMODE
        while (1) {
            digitalWrite(3, HIGH);              // Turn On Moisture Sensor
            delay(100);
            soilMoistureValue = analogRead(A1);  //put Sensor insert into soil
            digitalWrite(3, LOW);               // Turn Off Moisture Sensor
            Serial.println(soilMoistureValue);
            soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
#ifdef AIRCONFIG
            if (soilmoisturepercent < 5) {
              Serial.println("Decrementing percentage");
              AirValue -= 1;
              Serial.print("AirValue: ");
              Serial.println(AirValue);
            } else if (soilmoisturepercent > 5 ) {
              Serial.println("Incrementing percentage");
              AirValue += 1;
              Serial.print("AirValue: ");
              Serial.println(AirValue);
            }
#endif

#ifdef WATERCONFIG
            if (soilmoisturepercent < 95) {
                Serial.println("Incrementing percentage");
                WaterValue += 1;
                Serial.print("WaterValue: ");
                Serial.println(WaterValue);
            } else if (soilmoisturepercent > 95) {
                Serial.println("Decrementing percentage");
                WaterValue -= 1;
                Serial.print("WaterValue: ");
                Serial.println(WaterValue);
            }
#endif
#endif
        digitalWrite(3, HIGH);              // Turn On Moisture Sensor
        delay(100);
        soilMoistureValue = analogRead(A1);  //put Sensor insert into soil
        Serial.println(soilMoistureValue);
        soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
        digitalWrite(3, LOW);               // Turn Off Moisture Sensor
        if (soilmoisturepercent > 100) {
            Serial.print("Soil Moisture: ");
            Serial.print(soilmoisturepercent);
            Serial.println(" %");
            soilmoisturepercent = 100;
            delay(250);
        } else if (soilmoisturepercent < 0) {
            Serial.print("Soil Moisture: ");
            Serial.print(soilmoisturepercent);
            Serial.println(" %");
            soilmoisturepercent = 0;
            delay(250);
        } else if (soilmoisturepercent >= 0 && soilmoisturepercent <= 100) {
            Serial.print("Soil Moisture: ");
            Serial.print(soilmoisturepercent);
            Serial.println(" %");
            delay(250);
        }
        //BMP280
        float temp(NAN), hum(NAN), pres(NAN);

        BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
        BME280::PresUnit presUnit(BME280::PresUnit_hPa);

        bme.read(pres, temp, hum, tempUnit, presUnit);
        uint16_t internalVoltage = readVcc();

        // Read ext battery
        int battVoltage = analogRead(A0);
        float volt = battVoltage / 1023.0;
        volt = volt * 17.6 * 1000; // millivolts
        uint16_t v;
        v = (int) volt;


        Serial.print("Battery Voltage: ");
        Serial.println(v);

        Serial.print("Temp: ");
        Serial.println(temp);
        Serial.print("Pressure: ");
        Serial.println(pres);

        uint16_t pressure;
        pres = pres * 50;
        pressure = (int) pres;
        Serial.print("Pressure: ");
        Serial.println(pressure);

        /*
           Field1: Temperature
           Field2: Pressure
           Field3: Humidity
           Field4: Battery Voltage
           Field5: Internal Voltage
           Field6: Soil Moisture
           Field7: Soil Moisture Value
           Field8:
        */
        for (int i = 0; i < BUFFERSIZE; i++) {
            mydata[i] = 0;
        }
        LoraEncoder encoder(mydata);
        encoder.writeTemperature(temp);
        encoder.writeUint16(pressure);
        encoder.writeHumidity(hum);
        encoder.writeUint16(v);
        encoder.writeUint16(internalVoltage);
        encoder.writeHumidity(soilmoisturepercent);
        encoder.writeUint16(soilMoistureValue);

        Serial.print("Payload: ");
        for (int i = 0; i < BUFFERSIZE; i++) {
            Serial.print(mydata[i], HEX);
        }
        Serial.println();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        Serial.println(F("Packet queued"));
    }
}

// Next TX is scheduled after TX_COMPLETE event.
void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
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
            if (LMIC.dataLen) {
                Serial.println(F("Received "));
                Serial.println(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));
    Wire.begin();

    // BMP280
    while (!bme.begin()) {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }
    // bme.chipID(); // Deprecated. See chipModel().
    switch (bme.chipModel()) {
        case BME280::ChipModel_BME280:
            Serial.println("Found BME280 sensor! Success.");
            break;
        case BME280::ChipModel_BMP280:
            Serial.println("Found BMP280 sensor! No Humidity available.");
            break;
        default:
            Serial.println("Found UNKNOWN sensor! Error!");
    }
#ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
#endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

    LMIC_selectSubBand(1);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    // Start job
    do_send(&sendjob);
}


void loop() {
    os_runloop_once();
}
