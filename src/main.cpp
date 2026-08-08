// ============================================================
// main.cpp — Stevenson enclosure weather node
//
// Aggregates sensors for ten minutes, then sends one framed report over
// USART2 to a RAK4631 hub, which relays it onto the MeshCore mesh.
//
//   Sensors  -> SPI1 shared: BME680 (MODE0) + AS3935 (MODE1)
//   Time     -> DS3231 on I2C1, disciplined occasionally by GPS
//   Uplink   -> USART2, one-way framed binary
//   Debug    -> USB CDC
//
// Design constraints that shaped this:
//   - Solar powered. Idle low, do not busy-wait, keep the GPS asleep.
//   - Roof mounted. Physical access needs a ladder, so IWDG is on and
//     nothing may block forever.
//   - Brownout safe. All state is rebuilt from a cold start; the RTC
//     carries time across outages that flatten the GPS.
//
// See config.h for pins, tuning and open hardware items.
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <IWatchdog.h>

#include "config.h"
#include "as3935.h"
#include "bme.h"
#include "rtc.h"
#include "gps.h"
#include "frame.h"
#include "calib.h"
#include "as3935_cal.h"
#include "stats.h"

// ---- Debug -------------------------------------------------
#define DEBUG 1
#if DEBUG
  #define DBG(...)   CONSOLE.print(__VA_ARGS__)
  #define DBGLN(...) CONSOLE.println(__VA_ARGS__)
#else
  #define DBG(...)
  #define DBGLN(...)
#endif

// ---- Peripherals -------------------------------------------
static AS3935 lightning(PIN_AS3935_CS, SPI);

static RollingStats<SAMPLE_WINDOW_LEN> wTemp, wRh, wPress;

// ---- Health ------------------------------------------------
static bool bmeOk = false, as3935Ok = false, rtcOk = false;
static bool gpsSyncedThisSession = false;
static NodeCalib calib;
static bool calibValid = false;

// ---- Lightning ---------------------------------------------
static volatile bool lightningIrq = false;
static void onLightningIrq() { lightningIrq = true; }

static uint16_t winStrikes = 0, winDisturbers = 0;
static uint8_t  winNearestKm = 0;
static uint32_t lastStrikeMs = 0;

// ---- Scheduling --------------------------------------------
static uint32_t lastSampleMs = 0;
static uint32_t lastReportMs = 0;
static uint32_t lastGpsSyncMs = 0;
static uint32_t gpsWakeMs = 0;
static bool     gpsSyncInProgress = false;

// ============================================================
// Supply voltage from the MCU's internal reference. Free telemetry,
// and the only power measurement available until the INA226 is fitted.
// ============================================================
static uint16_t readVddMv() {
    analogReadResolution(12);
    uint32_t raw = analogRead(AVREF);
    if (!raw) return 0;
    uint16_t cal = *((__IO uint16_t *)0x1FFF7A2AUL);   // VREFINT_CAL @ 3.3V
    if (!cal || cal == 0xFFFF) return 0;
    return (uint16_t)((3300UL * cal) / raw);
}

// ============================================================
// Lightning
// ============================================================
static void serviceLightning() {
    if (!lightningIrq) return;
    lightningIrq = false;

    // Datasheet: wait 2 ms after the IRQ edge before reading the source
    // register. Read sooner and it returns 0x00 without clearing.
    delay(3);
    uint8_t src = lightning.getInterruptSource();

    if (src == AS3935_INT_LIGHTNING) {
        uint8_t km = lightning.getDistanceEstimate();
        winStrikes++;
        lastStrikeMs = millis();
        // 0x3F means "out of range" — not a distance.
        if (km > 0 && km < 0x3F && (winNearestKm == 0 || km < winNearestKm))
            winNearestKm = km;
        DBG(F("LIGHTNING ")); DBG(km); DBGLN(F(" km"));
    } else if (src == AS3935_INT_DISTURBER) {
        winDisturbers++;
    } else if (src == AS3935_INT_NOISE) {
        // Noise trips mean the floor is set too low for this site. Count
        // them with disturbers so the hub can see the tuning is wrong.
        winDisturbers++;
    }
}

// ============================================================
// Sampling
// ============================================================
static void takeSample() {
    BmeReading r;
    if (!bmeRead(r)) {
        bmeOk = false;
        DBGLN(F("BME read failed"));
        return;
    }
    bmeOk = true;

    uint32_t now = rtcOk ? rtcUnix() : (millis() / 1000);
    if (now == 0) now = millis() / 1000;

    wTemp.add(r.temperature, now);
    wRh.add(r.humidity, now);
    wPress.add(r.pressure, now);

    DBG(F("sample T=")); DBG(r.temperature, 2);
    DBG(F(" RH="));      DBG(r.humidity, 2);
    DBG(F(" P="));       DBG(r.pressure, 2);
    DBG(F(" n="));       DBGLN(wTemp.size());
}

// ============================================================
// Reporting
// ============================================================
static void sendReport(bool clearWindow) {
    WeatherPayload p = {};

    p.unixTime = rtcOk ? rtcUnix() : 0;
    p.node     = NODE_NUM;
    p.samples  = wTemp.size();

    p.flags = 0;
    if (rtcOk && !rtcOscStopped())        p.flags |= FLAG_RTC_VALID;
    if (gpsHasFix())                      p.flags |= FLAG_GPS_FIX;
    if (gpsSyncedThisSession)             p.flags |= FLAG_GPS_SYNCED;
    if (bmeOk)                            p.flags |= FLAG_BME_OK;
    if (as3935Ok)                         p.flags |= FLAG_AS3935_OK;
    // FLAG_IAQ_VALID stays clear: the gas heater on this board does not
    // reach temperature, so BSEC is not run at all. See bme.h.
    if (winStrikes && (millis() - lastStrikeMs) < LIGHTNING_HOLD_MS)
        p.flags |= FLAG_STORM_ACTIVE;

    if (!wTemp.empty()) {
        p.tempNow   = scaleI16(wTemp.latest(), 100.0f);
        p.tempMin   = scaleI16(wTemp.min(),    100.0f);
        p.tempMax   = scaleI16(wTemp.max(),    100.0f);
        p.tempTrend = scaleI16(wTemp.trendPerHour(), 100.0f);

        p.rhNow     = scaleU16(wRh.latest(), 100.0f);
        p.rhMin     = scaleU16(wRh.min(),    100.0f);
        p.rhMax     = scaleU16(wRh.max(),    100.0f);
        p.rhTrend   = scaleI16(wRh.trendPerHour(), 100.0f);

        p.pressNow   = scaleU32(wPress.latest(), 100.0f);
        p.pressMin   = scaleU32(wPress.min(),    100.0f);
        p.pressMax   = scaleU32(wPress.max(),    100.0f);
        p.pressTrend = scaleI16(wPress.trendPerHour(), 100.0f);
    }

    p.strikes    = winStrikes;
    p.nearestKm  = winNearestKm;
    p.disturbers = winDisturbers;
    p.vddMv      = readVddMv();
    p.uptimeMin  = (uint16_t)(millis() / 60000UL);

    uint8_t buf[FRAME_MAX_LEN];
    size_t  n = frameBuildWeather(buf, sizeof(buf), p);
    if (!n) { DBGLN(F("frame build failed")); return; }

    SerialUplink.write(buf, n);
    SerialUplink.flush();

#if DEBUG
    DBG(F("\n--- report, ")); DBG(n); DBGLN(F(" bytes ---"));
    DBG(F("  t=")); DBG(wTemp.latest(), 2);
    DBG(F("C  rh=")); DBG(wRh.latest(), 1);
    DBG(F("%  p=")); DBG(wPress.latest(), 2);
    DBG(F("hPa  ptrend=")); DBG(wPress.trendPerHour(), 3);
    DBGLN(F(" hPa/hr"));
    DBG(F("  strikes=")); DBG(winStrikes);
    DBG(F(" nearest=")); DBG(winNearestKm);
    DBG(F("km disturbers=")); DBGLN(winDisturbers);
    DBG(F("  vdd=")); DBG(p.vddMv); DBGLN(F(" mV"));
    frameDump(CONSOLE, buf, n);
#endif

    // Counters are per-window; the stats windows roll on their own so
    // trends survive across reports. A report forced from the console
    // leaves them alone — testing the link should not destroy data.
    if (clearWindow) {
        winStrikes = winDisturbers = 0;
        winNearestKm = 0;
    }
}

// ============================================================
// GPS time discipline
//
// Runs rarely and never blocks. The state machine exists so a GPS that
// cannot get a fix costs nothing beyond its own power budget — the node
// keeps reporting on RTC time throughout.
// ============================================================
static void serviceGpsSync() {
    if (!gpsSyncInProgress) {
        if (millis() - lastGpsSyncMs < GPS_SYNC_INTERVAL_MS) return;
        DBGLN(F("GPS: waking for time sync"));
        gpsWake();
        gpsWakeMs = millis();
        gpsSyncInProgress = true;
        return;
    }

    gpsPoll();

    // Time without a fix is good enough to correct a clock to the
    // second, and arrives far sooner than a position solution.
    if (gpsTimeValid()) {
        uint32_t g = gpsUnix();
        if (g > 1700000000UL) {          // sanity: after Nov 2023
            uint32_t before = rtcUnix();

            // Setting on the PPS edge puts us at the top of the second
            // instead of somewhere random within it.
            uint32_t waitStart = millis();
            while (!gpsPpsTick() && millis() - waitStart < 1200) { /* brief */ }

            rtcSetUnix(g + 1);           // the edge marks the NEXT second
            gpsSyncedThisSession = true;
            rtcOk = true;

            DBG(F("GPS: RTC set, was ")); DBG(before);
            DBG(F(" now ")); DBGLN(g + 1);

            gpsSleep();
            gpsSyncInProgress = false;
            lastGpsSyncMs = millis();
            return;
        }
    }

    if (millis() - gpsWakeMs > GPS_FIX_TIMEOUT_MS) {
        DBGLN(F("GPS: no time before timeout, back to sleep"));
        gpsSleep();
        gpsSyncInProgress = false;
        lastGpsSyncMs = millis();
    }
}


// ============================================================
// Debug console. Compiled out entirely when DEBUG is 0 — the flight
// build must not expose a way to trigger a two-minute calibration or a
// reset on a node that is up a ladder.
// ============================================================
#if DEBUG
static void printStatus() {
    RtcTime t;
    DBGLN(F("\n---- status ----"));
    DBG(F("  uptime   ")); DBG(millis() / 1000); DBGLN(F(" s"));
    if (rtcOk && rtcRead(t)) {
        DBG(F("  rtc      ")); DBG(t.year); DBG('-'); DBG(t.month); DBG('-');
        DBG(t.day); DBG(' '); DBG(t.hour); DBG(':'); DBG(t.minute);
        DBG(':'); DBG(t.second); DBGLN(rtcOscStopped() ? F("  OSF SET") : F("  UTC"));
    } else {
        DBGLN(F("  rtc      unavailable"));
    }
    DBG(F("  bme680   ")); DBGLN(bmeOk ? F("ok (gas heater disabled)") : F("FAILED"));
    DBG(F("  as3935   ")); DBGLN(as3935Ok ? F("ok") : F("FAILED"));
    if (calibValid) {
        DBG(F("  calib    cap=")); DBG(calib.tuningCap);
        DBG(F(" lco="));   DBG(calib.lcoHz);
        DBG(F(" floor=")); DBG(calib.noiseFloor);
        DBG(F(" wd="));    DBG(calib.watchdog);
        DBG(F(" @"));      DBG(calib.tempC); DBGLN(F("C"));
    } else {
        DBGLN(F("  calib    NONE STORED"));
    }
    DBG(F("  samples  ")); DBG(wTemp.size()); DBG('/'); DBGLN(SAMPLE_WINDOW_LEN);
    DBG(F("  window   strikes=")); DBG(winStrikes);
    DBG(F(" disturbers=")); DBGLN(winDisturbers);
    DBG(F("  vdd      ")); DBG(readVddMv()); DBGLN(F(" mV"));
    DBG(F("  gps      "));
    if (gpsIsAsleep()) DBGLN(F("asleep"));
    else { DBG(gpsSatellites()); DBGLN(gpsHasFix() ? F(" sats, FIX") : F(" sats, no fix")); }
    DBG(F("  uplink   USART2 PA2/PA3 @ ")); DBG(UPLINK_BAUD); DBGLN(F(" 8N1"));
    DBG(F("  next rpt ")); DBG((REPORT_INTERVAL_MS - (millis() - lastReportMs)) / 1000);
    DBGLN(F(" s"));
}

static void runCalibration() {
    if (!as3935Ok) { DBGLN(F("AS3935 not present")); return; }

    // The calibration routine drives the IRQ pin itself.
    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));

    float dieT = 20.0f;
    rtcDieTemp(dieT);

    memset(&calib, 0, sizeof(calib));
    bool antennaOk = as3935Calibrate(lightning, calib, &CONSOLE);
    calib.tempC    = (int8_t)dieT;
    calib.unixTime = rtcOk ? rtcUnix() : 0;

    calibValid = calibSave(calib);
    DBGLN(calibValid ? F("calibration saved") : F("calibration save FAILED"));
    if (!antennaOk) DBGLN(F("WARNING: antenna out of spec"));

    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onLightningIrq, RISING);
    lastSampleMs = millis();
}

static void printHelp() {
    DBGLN(F("\n  r  send a report now (does not clear the window)"));
    DBGLN(F("  s  status"));
    DBGLN(F("  c  run site calibration now (~2 min) and save"));
    DBGLN(F("  i  invalidate stored calibration (re-runs next boot)"));
    DBGLN(F("  g  wake GPS and sync the RTC now"));
    DBGLN(F("  x  reset"));
    DBGLN(F("  ?  this help"));
}

static void serviceConsole() {
    // 1200-baud touch resets the board, so a host can restart it without
    // touching the hardware. A plain reset, not a bootloader jump.
    if (CONSOLE.baud() == 1200) {
        DBGLN(F("1200 baud touch — resetting"));
        CONSOLE.flush();
        delay(100);
        NVIC_SystemReset();
    }
    if (!CONSOLE.available()) return;
    switch (CONSOLE.read()) {
        case 'r': sendReport(false); break;
        case 's': printStatus(); break;
        case 'c': runCalibration(); break;
        case 'i': calibInvalidate(); calibValid = false;
                  DBGLN(F("stored calibration erased")); break;
        case 'g': lastGpsSyncMs = millis() - GPS_SYNC_INTERVAL_MS;
                  DBGLN(F("GPS sync requested")); break;
        case 'x': DBGLN(F("resetting")); CONSOLE.flush(); delay(100);
                  NVIC_SystemReset(); break;
        case '?': printHelp(); break;
        default:  break;
    }
}
#else
static void serviceConsole() {}
#endif

// ============================================================
void setup() {
#if DEBUG
    CONSOLE.begin(115200);
    uint32_t t0 = millis();
    while (!CONSOLE && millis() - t0 < 3000) { }
    DBGLN(F("\n=== " NODE_ID " weather node ==="));
    DBGLN(F("built " __DATE__ " " __TIME__));
#endif

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // CS lines idle high before the shared bus comes up, so neither
    // device sees the other's traffic during init.
    pinMode(PIN_BME680_CS, OUTPUT);
    pinMode(PIN_AS3935_CS, OUTPUT);
    digitalWrite(PIN_BME680_CS, HIGH);
    digitalWrite(PIN_AS3935_CS, HIGH);
    delay(10);
    SPI.begin();

    SerialUplink.begin(UPLINK_BAUD);

    // ---- RTC ----
    rtcOk = rtcBegin();
    if (rtcOk) {
        RtcTime t;
        if (rtcRead(t)) {
            DBG(F("RTC ")); DBG(t.year); DBG('-'); DBG(t.month); DBG('-');
            DBG(t.day); DBG(' '); DBG(t.hour); DBG(':'); DBG(t.minute);
            DBG(':'); DBGLN(t.second);
        } else {
            rtcOk = false;
        }
        if (rtcOscStopped()) DBGLN(F("RTC: OSF set — time not trusted"));
    } else {
        DBGLN(F("RTC FAILED"));
    }

    // ---- BME680 ----
    bmeOk = bmeBegin();
    DBGLN(bmeOk ? F("BME680 OK (gas heater disabled)") : F("BME680 FAILED"));

    // ---- AS3935 ----
    as3935Ok = lightning.begin();
    pinMode(PIN_AS3935_IRQ, INPUT);

    if (as3935Ok) {
        DBGLN(F("AS3935 OK"));

        // Calibration is a two-minute measurement of THIS site, so the
        // watchdog has to be running through it and the lightning ISR
        // must stay detached — the calibration code drives that pin.
        IWatchdog.begin(IWDG_TIMEOUT_MS * 1000UL);

        float dieT = 20.0f;
        rtcDieTemp(dieT);

        calibValid = calibLoad(calib);
        if (calibValid && calibShouldRefresh(calib, (int8_t)dieT)) {
            DBGLN(F("stored calibration is from a very different temperature"));
            calibValid = false;
        }

        if (calibValid) {
            DBG(F("calibration loaded: cap=")); DBG(calib.tuningCap);
            DBG(F(" lco="));    DBG(calib.lcoHz);
            DBG(F(" floor="));  DBG(calib.noiseFloor);
            DBG(F(" wd="));     DBGLN(calib.watchdog);
            as3935ApplyCalib(lightning, calib);
        } else {
            DBGLN(F("no usable calibration — running site calibration"));
            memset(&calib, 0, sizeof(calib));
            bool antennaOk = as3935Calibrate(lightning, calib,
                                             DEBUG ? &CONSOLE : (Print *)NULL);
            calib.tempC    = (int8_t)dieT;
            calib.unixTime = rtcOk ? rtcUnix() : 0;

            if (calibSave(calib)) {
                calibValid = true;
                DBGLN(F("calibration saved"));
            } else {
                DBGLN(F("calibration save FAILED — will re-run next boot"));
            }
            if (!antennaOk)
                DBGLN(F("WARNING: antenna out of spec, distances unreliable"));
        }
    } else {
        DBGLN(F("AS3935 FAILED"));
    }

    // Lightning ISR attaches only after calibration is finished with the pin.
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onLightningIrq, RISING);

    // ---- GPS ----
    gpsBegin();
    gpsSleep();                     // wake only when a sync is due
    lastGpsSyncMs = millis() - GPS_SYNC_INTERVAL_MS;   // sync soon after boot

    // ---- Boot frame ----
    {
        uint8_t buf[FRAME_MAX_LEN];
        uint8_t flags = (bmeOk ? FLAG_BME_OK : 0) | (as3935Ok ? FLAG_AS3935_OK : 0) |
                        (rtcOk && !rtcOscStopped() ? FLAG_RTC_VALID : 0);
        size_t n = frameBuildBoot(buf, sizeof(buf), NODE_NUM,
                                  rtcOk ? rtcUnix() : 0, flags, readVddMv());
        if (n) { SerialUplink.write(buf, n); SerialUplink.flush(); }
    }

    // Take one sample immediately so the first report is never empty.
    takeSample();
    lastSampleMs = millis();
    lastReportMs = millis();

    // Already started before calibration if the AS3935 came up; IWDG
    // cannot be re-initialised once running.
    if (!as3935Ok) IWatchdog.begin(IWDG_TIMEOUT_MS * 1000UL);
    DBGLN(F("running — press ? for commands\n"));
}

void loop() {
    IWatchdog.reload();

    serviceConsole();
    serviceLightning();

    if (millis() - lastSampleMs >= SAMPLE_INTERVAL_MS) {
        lastSampleMs = millis();
        takeSample();
    }

    if (millis() - lastReportMs >= REPORT_INTERVAL_MS) {
        lastReportMs = millis();
        sendReport(true);
    }

    serviceGpsSync();

    // Brief heartbeat: visible, and cheap enough not to matter on solar.
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink > 5000) {
        lastBlink = millis();
        digitalWrite(PIN_LED, LOW);
        delay(10);
        digitalWrite(PIN_LED, HIGH);
    }

    // Idle until the next interrupt rather than spinning. Real STOP mode
    // is the stretch goal, but it fights USB CDC — measure before going
    // further, per the handoff.
    __WFI();
}
