#pragma once
// ============================================================
// calib.h — persisted site calibration, in emulated EEPROM.
//
// The AS3935 has to be calibrated where it is installed: antenna
// resonance depends on what is near it, and the useful noise floor and
// watchdog thresholds depend on the local RF environment. That takes
// about two minutes of measurement, which is far too long to repeat on
// every boot of a solar node that is expected to brown out.
//
// So calibrate once, store the result, and reuse it. A brownout at 3 am
// then costs a few seconds of restart, not two minutes of the node
// sitting deaf to lightning while it re-tunes.
//
// STORAGE WARNING: on STM32F4 there is no real EEPROM. The Arduino
// EEPROM emulation lives in one 8 kB flash sector, and EEPROM.put()
// calls eeprom_write_byte() per byte — each of which erases and rewrites
// the WHOLE sector. Writing this struct that way would be ~20 sector
// erases against a ~10,000 cycle endurance budget. Always use the
// buffered API: fill, write bytes into the buffer, flush once.
// ============================================================

#include <Arduino.h>

#define CALIB_MAGIC   0x57583031UL   // "WX01"
#define CALIB_VERSION 1

struct NodeCalib {
    uint32_t magic;
    uint8_t  version;
    uint8_t  tuningCap;     // AS3935 reg 0x08 [3:0], 0..15
    uint32_t lcoHz;         // measured antenna frequency at that cap
    uint8_t  noiseFloor;    // reg 0x01 [6:4], 0..7
    uint8_t  watchdog;      // reg 0x01 [3:0], 0..15
    uint8_t  spikeReject;   // reg 0x02 [3:0], 0..15
    int8_t   tempC;         // die temperature when calibrated
    uint32_t unixTime;      // when, 0 if clock was not trusted
    uint16_t crc;           // over everything above
};

bool calibLoad(NodeCalib &out);
bool calibSave(NodeCalib &c);        // fills in crc
void calibInvalidate();

// True if the stored calibration is too old or was taken at a very
// different temperature to now. Antenna resonance moves with
// temperature, and this antenna already sits at the end of its trim
// range, so a large thermal delta is worth re-checking.
bool calibShouldRefresh(const NodeCalib &c, int8_t nowTempC);
