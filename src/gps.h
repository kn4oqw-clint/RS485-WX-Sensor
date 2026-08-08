#pragma once
// ============================================================
// gps.h — GT-U7 / u-blox 7, used only to discipline the RTC.
//
// Position is not wanted. The node does not move. All this exists for is
// to correct DS3231 drift occasionally, and it must degrade gracefully:
// the mesh side also syncs node clocks, so a GPS that never gets a fix
// is an inconvenience, not a failure.
//
// Power discipline, in order of preference:
//   1. Hard power gate on PIN_GPS_PWR (MOSFET not yet populated).
//   2. UBX-RXM-PMREQ software sleep.
//
// Whenever the module is asleep or unpowered, USART1 TX must be driven
// to high-Z first. Left driven, it back-powers the module through its
// ESD protection diodes, which both wastes power and stops the module
// actually resetting.
// ============================================================

#include <Arduino.h>
#include "rtc.h"

void gpsBegin();

// Feed the parser. Call often; it never blocks.
void gpsPoll();

bool     gpsHasFix();
bool     gpsTimeValid();          // valid UTC date+time, fix not required
uint32_t gpsUnix();               // 0 if no valid time
uint8_t  gpsSatellites();

// Configure a 1 Hz timepulse that runs even without a fix. Must be
// re-sent after every power cycle or wake — it lives in RAM, not in the
// module's battery-backed config.
void gpsConfigTimepulse();

void gpsSleep();
void gpsWake();
bool gpsIsAsleep();

// True on the PPS rising edge since the last call. The edge marks the
// exact top of the second, so the RTC can be set on it rather than at
// some arbitrary point mid-second.
bool gpsPpsTick();
