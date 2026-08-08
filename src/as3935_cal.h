#pragma once
// ============================================================
// as3935_cal.h — site calibration for the AS3935.
//
// Split by who does it better:
//
//   ANTENNA resonance -> AS3935MI::calibrateResonanceFrequency().
//     Getting this right is fiddly in ways that are not obvious: the LCO
//     appears on the IRQ pin at ~31 kHz, and any approach that counts
//     edges in a fixed window silently reports a LOWER frequency when it
//     misses edges — indistinguishable from a genuinely detuned antenna.
//     The library counts a fixed number of edges and times them, so a
//     failure times out instead of lying. A hand-rolled version of this
//     cost several rounds of debugging before being replaced.
//
//   ENVIRONMENT (noise floor, watchdog) -> our code, below.
//     No library does this: it depends entirely on the RF environment at
//     the installed site, which is exactly why it cannot be done on a
//     bench. Start over-sensitive, raise each threshold only until the
//     interrupt rate is sane.
//
// Results are persisted (see calib.h) so this runs once per site rather
// than on every boot of a node that is expected to brown out.
// ============================================================

#include <Arduino.h>
#include <AS3935SPI.h>
#include "calib.h"

// Runs both stages. log may be NULL. Returns false if the antenna could
// not be tuned to within 3.5% of 500 kHz — the node still runs, but
// lightning distance estimates should not be trusted.
bool as3935Calibrate(AS3935SPI &dev, NodeCalib &out, Print *log);

// Programs stored values into the device. Call this whether the
// calibration was just measured or loaded from EEPROM.
void as3935ApplyCalib(AS3935SPI &dev, const NodeCalib &c);
