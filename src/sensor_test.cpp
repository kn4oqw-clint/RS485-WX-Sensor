// ============================================================
// sensor_test.cpp — BME680 / BSEC validation
//
// The board test proves the BME680 is present and has calibration data.
// It does NOT prove the compensation math produces believable numbers,
// because that all happens inside BSEC. This does.
//
// Two independent checks:
//   1. Range plausibility on every BSEC output.
//   2. Cross-check BME680 temperature against the DS3231 die temperature.
//      Two unrelated sensors, same enclosure, same air. They will not
//      agree exactly — the DS3231 die runs warm and the BME680 self-heats
//      from its gas heater — but they must track each other. If they
//      disagree wildly, one of them is lying.
//
// Build:  pio run -e sensortest -t upload
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "config.h"
#include "bsec.h"
extern "C" {
#include "bme68x.h"
}

Bsec bme;
static bool bmeOk = false;

// ============================================================
// Direct forced-mode read, bypassing BSEC entirely.
//
// BSEC returning BSEC_E_DOSTEPS_VALUELIMITS (-2) means the raw values fed
// into it are outside the physical ranges it accepts. That points at the
// sensor or the bus, not the algorithm — so read the part ourselves with
// the Bosch driver and see what it actually reports. If these numbers are
// sane, the problem is in how BSEC is being driven. If they are garbage,
// it is the sensor or the SPI path.
// ============================================================
static struct bme68x_dev  rawDev;
static struct bme68x_conf rawConf;

static int8_t rawSpiRead(uint8_t reg, uint8_t *data, uint32_t len, void *intf) {
    (void)intf;
    SPI.beginTransaction(SPISettings(BME680_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_BME680_CS, LOW);
    SPI.transfer(reg);
    for (uint32_t i = 0; i < len; i++) data[i] = SPI.transfer(0x00);
    digitalWrite(PIN_BME680_CS, HIGH);
    SPI.endTransaction();
    return 0;
}

static int8_t rawSpiWrite(uint8_t reg, const uint8_t *data, uint32_t len, void *intf) {
    (void)intf;
    SPI.beginTransaction(SPISettings(BME680_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_BME680_CS, LOW);
    SPI.transfer(reg);
    for (uint32_t i = 0; i < len; i++) SPI.transfer(data[i]);
    digitalWrite(PIN_BME680_CS, HIGH);
    SPI.endTransaction();
    return 0;
}

static void rawDelayUs(uint32_t period, void *intf) { (void)intf; delayMicroseconds(period); }

static bool rawForcedRead() {
    CONSOLE.println(F("\n---- Direct BME680 read (no BSEC) ----"));

    rawDev.intf     = BME68X_SPI_INTF;
    rawDev.read     = rawSpiRead;
    rawDev.write    = rawSpiWrite;
    rawDev.delay_us = rawDelayUs;
    rawDev.intf_ptr = NULL;
    rawDev.amb_temp = 25;

    int8_t rslt = bme68x_init(&rawDev);
    CONSOLE.print(F("  bme68x_init -> "));
    CONSOLE.print(rslt);
    CONSOLE.print(F("   chip_id 0x"));
    CONSOLE.print(rawDev.chip_id, HEX);
    CONSOLE.print(F("   variant "));
    CONSOLE.println(rawDev.variant_id);
    if (rslt != BME68X_OK) {
        CONSOLE.println(F("  init failed — SPI path is broken"));
        return false;
    }

    rawConf.os_hum  = BME68X_OS_2X;
    rawConf.os_temp = BME68X_OS_8X;
    rawConf.os_pres = BME68X_OS_4X;
    rawConf.filter  = BME68X_FILTER_SIZE_3;
    rawConf.odr     = BME68X_ODR_NONE;
    rslt = bme68x_set_conf(&rawConf, &rawDev);
    CONSOLE.print(F("  set_conf -> ")); CONSOLE.println(rslt);

    struct bme68x_heatr_conf hConf;
    hConf.enable     = BME68X_ENABLE;
    hConf.heatr_temp = 320;
    hConf.heatr_dur  = 150;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &hConf, &rawDev);
    CONSOLE.print(F("  set_heatr_conf -> ")); CONSOLE.println(rslt);

    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &rawDev);
    CONSOLE.print(F("  set_op_mode -> ")); CONSOLE.println(rslt);

    uint32_t dur = bme68x_get_meas_dur(BME68X_FORCED_MODE, &rawConf, &rawDev) + hConf.heatr_dur * 1000;
    delayMicroseconds(dur);
    delay(10);

    struct bme68x_data d;
    uint8_t n = 0;
    rslt = bme68x_get_data(BME68X_FORCED_MODE, &d, &n, &rawDev);
    CONSOLE.print(F("  get_data -> ")); CONSOLE.print(rslt);
    CONSOLE.print(F("   fields: ")); CONSOLE.println(n);

    if (rslt != BME68X_OK || n == 0) {
        CONSOLE.println(F("  no data returned"));
        return false;
    }

    CONSOLE.print(F("  status 0x")); CONSOLE.print(d.status, HEX);
    CONSOLE.print(F("  (new_data=")); CONSOLE.print((d.status & 0x80) ? 1 : 0);
    CONSOLE.print(F(" heat_stab=")); CONSOLE.print((d.status & 0x10) ? 1 : 0);
    CONSOLE.print(F(" gas_valid=")); CONSOLE.print((d.status & 0x20) ? 1 : 0);
    CONSOLE.println(F(")"));

    CONSOLE.print(F("  T = "));   CONSOLE.print(d.temperature, 2);
    CONSOLE.print(F(" C   RH = ")); CONSOLE.print(d.humidity, 2);
    CONSOLE.print(F(" %   P = "));  CONSOLE.print(d.pressure / 100.0f, 2);
    CONSOLE.print(F(" hPa   gas = ")); CONSOLE.print(d.gas_resistance, 0);
    CONSOLE.println(F(" ohm"));

    bool sane = d.temperature > -40.0f && d.temperature < 85.0f &&
                d.humidity   >=  0.0f && d.humidity   <= 100.0f &&
                d.pressure   > 30000  && d.pressure   < 110000;

    CONSOLE.println(sane ? F("  VALUES ARE SANE — sensor and SPI are fine,")
                         : F("  VALUES ARE GARBAGE — sensor or SPI path fault."));
    if (sane) CONSOLE.println(F("  so the -2 is about how BSEC is being driven."));
    return sane;
}

static uint32_t sampleCount  = 0;
static float    tMin =  999.0f, tMax = -999.0f;
static float    pMin = 9999.0f, pMax =    0.0f;
static float    hMin =  999.0f, hMax = -999.0f;

// ---- DS3231 die temperature, for the cross-check ------------
static bool readRTCTemp(float &out) {
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x11);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)ADDR_DS3231, (uint8_t)2) != 2) return false;
    int8_t  hi = (int8_t)Wire.read();
    uint8_t lo = Wire.read() >> 6;
    out = hi + lo * 0.25f;
    return true;
}

static const char *accuracyText(uint8_t a) {
    switch (a) {
        case 0:  return "unreliable, still stabilising";
        case 1:  return "low, calibrating";
        case 2:  return "medium";
        case 3:  return "high, fully calibrated";
        default: return "?";
    }
}

static void checkBsecStatus() {
    static bsec_library_return_t lastBsec = BSEC_OK;
    static int8_t               lastBme  = BME68X_OK;
    if (bme.bsecStatus == lastBsec && bme.bme68xStatus == lastBme) return;
    lastBsec = bme.bsecStatus;
    lastBme  = bme.bme68xStatus;
    if (bme.bsecStatus != BSEC_OK) {
        CONSOLE.print(F("  BSEC status: "));
        CONSOLE.print(bme.bsecStatus);
        CONSOLE.println(bme.bsecStatus < BSEC_OK ? F("  (ERROR)") : F("  (warning)"));
    }
    if (bme.bme68xStatus != BME68X_OK) {
        CONSOLE.print(F("  BME68x status: "));
        CONSOLE.print(bme.bme68xStatus);
        CONSOLE.println(bme.bme68xStatus < BME68X_OK ? F("  (ERROR)") : F("  (warning)"));
    }
}

void setup() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    CONSOLE.begin(115200);
    uint32_t t0 = millis();
    while (!CONSOLE && millis() - t0 < 10000) { }
    delay(200);

    CONSOLE.println();
    CONSOLE.println(F("============================================================"));
    CONSOLE.println(F(" BME680 / BSEC validation — " NODE_ID));
    CONSOLE.println(F(" built " __DATE__ " " __TIME__));
    CONSOLE.println(F("============================================================"));

    // CS lines idle high before the shared bus comes up.
    pinMode(PIN_BME680_CS, OUTPUT);
    pinMode(PIN_AS3935_CS, OUTPUT);
    digitalWrite(PIN_BME680_CS, HIGH);
    digitalWrite(PIN_AS3935_CS, HIGH);
    delay(10);

    SPI.begin();

    rawForcedRead();

    Wire.setSCL(PIN_I2C_SCL);
    Wire.setSDA(PIN_I2C_SDA);
    Wire.begin();
    Wire.setClock(I2C_HZ);

    CONSOLE.println(F("\n---- BSEC init ----"));
    bme.begin(PIN_BME680_CS, SPI);
    CONSOLE.print(F("  BSEC library version "));
    CONSOLE.print(bme.version.major);      CONSOLE.print('.');
    CONSOLE.print(bme.version.minor);      CONSOLE.print('.');
    CONSOLE.print(bme.version.major_bugfix); CONSOLE.print('.');
    CONSOLE.println(bme.version.minor_bugfix);
    checkBsecStatus();

    if (bme.bsecStatus != BSEC_OK) {
        CONSOLE.println(F("  BSEC FAILED TO INIT — stopping."));
        return;
    }

    bsec_virtual_sensor_t sensorList[] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };
    bme.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkBsecStatus();

    if (bme.bsecStatus == BSEC_OK) {
        bmeOk = true;
        CONSOLE.println(F("  subscribed, LP mode (one sample every 3 s)"));
    }

    float rtcT;
    CONSOLE.print(F("  DS3231 die temp: "));
    if (readRTCTemp(rtcT)) { CONSOLE.print(rtcT, 2); CONSOLE.println(F(" C")); }
    else                   CONSOLE.println(F("unavailable"));

    CONSOLE.println(F("\n  IAQ accuracy climbs 0 -> 3 over tens of minutes."));
    CONSOLE.println(F("  Gas readings mean nothing until it reaches 1."));
    CONSOLE.println(F("  Temperature and pressure are valid immediately.\n"));
}

void loop() {
    if (!bmeOk) {
        digitalWrite(PIN_LED, LOW); delay(100);
        digitalWrite(PIN_LED, HIGH); delay(900);
        return;
    }

    if (!bme.run()) return;
    checkBsecStatus();

    sampleCount++;
    float tC   = bme.temperature;
    float rh   = bme.humidity;
    float hPa  = bme.pressure / 100.0f;

    if (tC  < tMin) tMin = tC;   if (tC  > tMax) tMax = tC;
    if (rh  < hMin) hMin = rh;   if (rh  > hMax) hMax = rh;
    if (hPa < pMin) pMin = hPa;  if (hPa > pMax) pMax = hPa;

    CONSOLE.print(F("["));
    CONSOLE.print(sampleCount);
    CONSOLE.print(F("]  T="));   CONSOLE.print(tC, 2);
    CONSOLE.print(F(" C  RH=")); CONSOLE.print(rh, 2);
    CONSOLE.print(F(" %  P=")); CONSOLE.print(hPa, 2);
    CONSOLE.print(F(" hPa  gas="));
    CONSOLE.print((unsigned long)bme.gasResistance);
    CONSOLE.print(F(" ohm  sIAQ="));
    CONSOLE.print(bme.staticIaq, 1);
    CONSOLE.print(F("(acc "));
    CONSOLE.print((int)bme.staticIaqAccuracy);
    CONSOLE.println(F(")"));

    // ---- Plausibility ---------------------------------------
    bool bad = false;
    if (tC  < -40.0f || tC  >  85.0f) { CONSOLE.println(F("  !! temperature out of sensor range")); bad = true; }
    if (rh  <   0.0f || rh  > 100.0f) { CONSOLE.println(F("  !! humidity out of range")); bad = true; }
    if (hPa < 300.0f || hPa > 1100.0f){ CONSOLE.println(F("  !! pressure out of range")); bad = true; }
    if (bme.gasResistance == 0)       { CONSOLE.println(F("  !! gas resistance zero — heater not running?")); bad = true; }

    // Sea-level pressure is 1013 hPa; anything far off at normal altitude
    // means the compensation is wrong, not the weather.
    if (!bad && (hPa < 800.0f || hPa > 1100.0f)) {
        CONSOLE.println(F("  ?  pressure is valid but unusual for low altitude"));
    }

    // ---- Cross-check against the DS3231 every 10th sample ----
    if (sampleCount % 10 == 0) {
        float rtcT;
        if (readRTCTemp(rtcT)) {
            float diff = tC - rtcT;
            CONSOLE.print(F("  cross-check: BME680 "));
            CONSOLE.print(tC, 2);
            CONSOLE.print(F(" C vs DS3231 die "));
            CONSOLE.print(rtcT, 2);
            CONSOLE.print(F(" C  delta "));
            CONSOLE.print(diff, 2);
            CONSOLE.println(F(" C"));

            if (fabsf(diff) > 15.0f) {
                CONSOLE.println(F("  !! sensors disagree badly — one is lying"));
            } else if (fabsf(diff) > 8.0f) {
                CONSOLE.println(F("  ?  larger gap than expected; the DS3231 die"));
                CONSOLE.println(F("     runs warm but not usually by this much"));
            } else {
                CONSOLE.println(F("  OK — two independent sensors agree"));
            }
        }

        CONSOLE.print(F("  ranges so far:  T "));
        CONSOLE.print(tMin, 2); CONSOLE.print(F(" .. ")); CONSOLE.print(tMax, 2);
        CONSOLE.print(F("   RH ")); CONSOLE.print(hMin, 2); CONSOLE.print(F(" .. ")); CONSOLE.print(hMax, 2);
        CONSOLE.print(F("   P ")); CONSOLE.print(pMin, 2); CONSOLE.print(F(" .. ")); CONSOLE.print(pMax, 2);
        CONSOLE.println();
        CONSOLE.print(F("  IAQ accuracy: "));
        CONSOLE.println(accuracyText(bme.staticIaqAccuracy));
        CONSOLE.println();
    }

    digitalWrite(PIN_LED, LOW);
    delay(15);
    digitalWrite(PIN_LED, HIGH);
}
