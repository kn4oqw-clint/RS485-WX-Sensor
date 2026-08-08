#pragma once
// ============================================================
// as3935_cal.h — site calibration for the AS3935.
//
// Two stages, roughly two minutes total:
//
//   1. Antenna tuning (~15 s). The LCO tank must resonate within 3.5%
//      of 500 kHz. Register 0x08 bit 7 puts the oscillator on the IRQ
//      pin divided by 16, so we sweep all 16 tuning-cap values and
//      count edges to find the closest. Adding capacitance only ever
//      LOWERS the frequency, so if the antenna already sits below
//      500 kHz at cap 0 there is no upward trim available — the sweep
//      reports that rather than silently picking the least-bad value.
//
//   2. Environment tuning (~100 s). Noise floor and watchdog threshold
//      depend entirely on what is near the installed node, which is why
//      this cannot be done on a bench. Start over-sensitive and raise
//      each threshold only until the interrupt rate is sane. Starting
//      permissive and lowering would take longer to converge and risks
//      settling on a threshold that misses real strikes.
//
// The result is persisted, so this runs once per site rather than once
// per boot. See calib.h.
// ============================================================

#include <Arduino.h>
#include "as3935.h"
#include "calib.h"

// Runs the full sweep. log may be NULL. Returns false if the antenna
// cannot be brought into spec — the node still runs, but lightning
// distance estimates from an out-of-spec antenna are not trustworthy.
bool as3935Calibrate(AS3935 &dev, NodeCalib &out, Print *log);

// Programs stored values into the device. Always call this, whether the
// calibration was just measured or loaded from EEPROM.
void as3935ApplyCalib(AS3935 &dev, const NodeCalib &c);

// Measures the antenna at one tuning-cap setting. Exposed so the board
// test can sweep without duplicating the edge-counting logic.
uint32_t as3935MeasureLco(AS3935 &dev, uint8_t tuningCap, uint16_t gateMs);
