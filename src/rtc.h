#pragma once
// ============================================================
// rtc.h — DS3231, the node's time authority.
//
// GPS is the disciplining source, not the authority. The receiver needs
// minutes of clear sky to reacquire after any power interruption, and
// this node runs on solar. The RTC has been measured holding time on its
// backup cell across full outages, so it carries the clock and GPS
// corrects it when convenient.
//
// Every call is bounded. A hung I2C transaction must not stall the loop
// on a roof-mounted node.
// ============================================================

#include <Arduino.h>

struct RtcTime {
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
};

bool rtcBegin();

// Reads time. valid is false if the transaction failed.
bool rtcRead(RtcTime &out);

// Unix epoch seconds, UTC. Returns 0 if the clock could not be read.
uint32_t rtcUnix();

bool rtcSet(const RtcTime &t);
bool rtcSetUnix(uint32_t unix);

// Oscillator Stop Flag: set by hardware whenever the oscillator has
// stopped, and STICKY until cleared in software. Set means the time is
// not trustworthy. It stays set from the module's first ever power-up,
// so it must be cleared once before it means anything.
bool rtcOscStopped();
bool rtcClearOscFlag();

bool rtcDieTemp(float &out);

uint32_t toUnix(const RtcTime &t);
void     fromUnix(uint32_t unix, RtcTime &out);
