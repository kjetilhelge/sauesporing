// Forsøk på å legge til lowpower
// v3 fungerer når noden er kopla til usb-lader, men ikkje når den er kopla til PC......
// då får ein første gps-fix i serial monitor, og ved neste oppvåkning står gps berre og lyser

//This is the first commit of an Arduino Sketch for an GPS-Tracker to work with The Things Network.
//This Sketch uses TinyGPS, the Arduino LMIC-Library by Matthijs Kooijmanand and the hartware serial of an Adafruit Feater LoRa.
//I use sample Code from Pieter Hensen https://github.com/Teumaat/ttn-tracker
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <CayenneLPP.h>


// use low power sleep; comment next line to not use low power sleep
#define SLEEP

#ifdef SLEEP
#include "LowPower.h"
bool next = false;
#endif

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
// const unsigned TX_INTERVAL = 56; //multiple of 8
unsigned long TX_INTERVAL = 56; //multiple of 8

TinyGPS gps;
CayenneLPP lpp(51);
const int GPSPowerPin = 10;
unsigned long startTime;
unsigned long syncTime;

unsigned long mnd, dag, minutt;

static const PROGMEM u1_t NWKSKEY[16] = { 0x98, 0x9D, 0x4B, 0x43, 0xCA, 0x16, 0x40, 0xBF, 0xFB, 0x45, 0xBE, 0xA8, 0xF5, 0xF9, 0x35, 0x13 };
static const u1_t PROGMEM APPSKEY[16] = { 0xCB, 0xF2, 0x2B, 0x3D, 0x1D, 0x8A, 0x76, 0x4E, 0x7B, 0x1C, 0xE0, 0x85, 0xEA, 0xCE, 0xF0, 0x96 };
static const u4_t DEVADDR = 0x26011B50 ; // <-- Change this address for every node!

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//uint8_t coords[6];
static osjob_t sendjob;


// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {7, 6, LMIC_UNUSED_PIN},
};

void get_coords () {
  digitalWrite(GPSPowerPin, HIGH);
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  float flat, flon;
  unsigned long age;
  unsigned long fix_age, time, date;
  startTime = millis();

  for (int i = 0; i < 120; i ++)
  {
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;) {
      while (Serial1.available()) {
        char c = Serial1.read();
        Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        if (gps.encode(c)) { // Did a new valid sentence come in?
          newData = true;
        }
      }
    }


    if ( newData ) {
      gps.f_get_position(&flat, &flon, &age);
      flat = (flat == TinyGPS::GPS_INVALID_F_ANGLE ) ? 0.0 : flat;
      flon = (flon == TinyGPS::GPS_INVALID_F_ANGLE ) ? 0.0 : flon;
      gps.get_datetime(&date, &time, fix_age);

      syncTime = ((millis() - startTime) / 1000);

      // mnd = (date / 100) % 100;
      // dag = (date / 10000);

      minutt = (time / 10000) % 100;

      if ( minutt > 25 ) {
        TX_INTERVAL = 64; // multiple of 8
        //Serial.println(minutt);
        //Serial.println(TX_INTERVAL);
        //delay(200);
      }
      else
      {
        TX_INTERVAL = 128; //multiple of 8
        //Serial.println(TX_INTERVAL);
        //delay(200);
      }


      break;

    }

    //delay(1000);
  }


  gps.stats(&chars, &sentences, &failed);

  lpp.reset();
  lpp.addGPS(1, flat, flon, syncTime);
  // lpp.addTemperature(2, syncTime);


  // slå av straum til GPS etterpå

  digitalWrite(GPSPowerPin, LOW);
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    get_coords();
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);

    Serial.println(F("Packet queued"));

    Serial.println("syncTime");
    Serial.println(syncTime);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
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

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
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
      next = true;
    //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    // break;

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

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Starting"));

  Serial1.begin(9600);

  // Definer styringspin for straum til GPS

  pinMode(GPSPowerPin, OUTPUT);


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
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  // LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {

#ifndef SLEEP

  os_runloop_once();

#else
  extern volatile unsigned long timer0_overflow_count;

  if (next == false) {

    os_runloop_once();

  } else {

    int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
    Serial.flush(); // give the serial print chance to complete
    for (int i = 0; i < sleepcycles; i++) {
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

      // LMIC uses micros() to keep track of the duty cycle, so
      // hack timer0_overflow for a rude adjustment:
      cli();
      timer0_overflow_count += 8 * 64 * clockCyclesPerMicrosecond();
      sei();
    }
    next = false;
    // Start job
    do_send(&sendjob);
  }

#endif
}

