#pragma once
// ============================================================
// bme.h — BME680 temperature / humidity / pressure, no BSEC.
//
// BSEC is deliberately not used on this board. Its algorithm rejects
// the out-of-range gas resistance produced by a hotplate that will not
// heat, returns BSEC_E_DOSTEPS_VALUELIMITS, and then emits NO outputs at
// all — including temperature and pressure, which are otherwise fine.
// One dead sub-sensor takes the whole reading with it.
//
// Reading the part directly through the Bosch driver with the gas
// heater DISABLED sidesteps that, and is the better choice regardless:
//
//   - The gas hotplate runs at 320 C on the same die as the temperature
//     sensor. BSEC exists partly to model and subtract that self-heating.
//     Never switching it on means there is nothing to subtract, so the
//     temperature is closer to true ambient — which is the entire point
//     of a Stevenson screen.
//   - It removes the largest transient current draw on the board, which
//     matters on solar.
//   - No multi-day calibration ramp, no state to persist across the
//     brownouts this node is expected to suffer.
//
// If the BME680 is ever replaced and IAQ is wanted back, this is where
// to reintroduce BSEC — but keep temperature and pressure on this path
// so a future gas failure cannot take them down again.
// ============================================================

#include <Arduino.h>

struct BmeReading {
    float temperature;      // degrees C
    float humidity;         // % RH
    float pressure;         // hPa
    bool  valid;
};

bool bmeBegin();
bool bmeRead(BmeReading &out);
