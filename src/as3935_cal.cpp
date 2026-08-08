// ============================================================
// as3935_cal.cpp — antenna and environment calibration
// ============================================================

#include <IWatchdog.h>
#include "as3935_cal.h"
#include "config.h"

// ---- Frequency measurement ---------------------------------
// Count a FIXED NUMBER of edges and measure how long it took, rather
// than counting edges in a fixed window. This is how the mature AS3935MI
// library does it, and the reason matters:
//
//   A fixed window cannot distinguish "half the edges were missed" from
//   "the antenna resonates at half that frequency" — both produce a
//   plausible-looking number. Fixed sample count instead either reaches
//   the target and gives a trustworthy period, or times out and reports
//   a clean failure. There is no middle state that lies to you.
//
// At the default divider of 16 a healthy 500 kHz antenna appears as
// 31.25 kHz, so 1000 samples completes in about 32 ms.
static volatile uint32_t edgeCount  = 0;
static volatile uint32_t calEndUs   = 0;
static uint32_t          calTarget  = AS3935_LCO_SAMPLES;

static void onEdge() {
    if (edgeCount < calTarget) {
        edgeCount++;
    } else if (calEndUs == 0) {
        calEndUs = micros();
    }
}

uint32_t as3935MeasureLco(AS3935 &dev, uint8_t tuningCap, uint16_t /*unused*/) {
    // Registers first, with no interrupt attached. The IRQ pin toggles at
    // tens of kHz once DISP_LCO is on, and servicing that while driving
    // the bus is what the reference library warns against.
    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));

    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0x7F, 0x00);   // LCO off
    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0xF0, tuningCap & 0x0F);
    dev.maskRegisterBits(AS3935_REG_INT_MASK_ANT, 0x3F, 0x00);  // LCO_FDIV = /16
    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0x7F, 0x80);   // LCO on

    // Plain INPUT, never INPUT_PULLUP: the AS3935 drives this pin.
    pinMode(PIN_AS3935_IRQ, INPUT);
    delay(5);                                   // let the signal appear

    IWatchdog.reload();
    edgeCount = 0;
    calEndUs  = 0;
    uint32_t startUs = micros();
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onEdge, RISING);

    // Expected duration at the target frequency, with generous slack.
    uint32_t expectedMs = (calTarget * AS3935_LCO_DIVIDER) / 500UL;   // ms
    uint32_t deadline   = millis() + expectedMs * 4 + 30;
    while (calEndUs == 0 && (int32_t)(millis() - deadline) < 0) { }

    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));
    IWatchdog.reload();

    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0x7F, 0x00);   // LCO off

    if (calEndUs == 0) return 0;                // never reached the count

    uint32_t elapsedUs = calEndUs - startUs;
    if (elapsedUs == 0) return 0;

    // samples/second * divider = antenna frequency
    return (uint32_t)(((uint64_t)calTarget * 1000000ULL * AS3935_LCO_DIVIDER)
                      / elapsedUs);
}

// Counts interrupts by source over a window at the current settings.
// ---- Stage 1: antenna --------------------------------------
static bool tuneAntenna(AS3935 &dev, NodeCalib &out, Print *log) {
    if (log) log->println(F("  [1/2] antenna sweep"));

    // LCO_FDIV (reg 0x03 bits 7:6) = 00 -> divide by 16. Once, up front:
    // a wrong divider silently scales every reading in the sweep.
    dev.maskRegisterBits(AS3935_REG_INT_MASK_ANT, 0x3F, 0x00);
    delay(3);

    uint8_t  bestCap = 0;
    uint32_t bestHz  = 0;
    uint32_t bestErr = 0xFFFFFFFF;

    for (uint8_t cap = 0; cap < 16; cap++) {
        // Retry rather than average. A failed pass reads exactly zero,
        // and averaging a good pass with a zero halves the answer — which
        // is how a healthy 488 kHz antenna came back as 244 kHz garbage.
        // Take the best of up to three attempts instead.
        uint32_t hz = 0;
        for (uint8_t attempt = 0; attempt < 3 && hz == 0; attempt++)
            hz = as3935MeasureLco(dev, cap, 0);

        uint32_t err = hz > AS3935_LCO_TARGET_HZ ? hz - AS3935_LCO_TARGET_HZ
                                                 : AS3935_LCO_TARGET_HZ - hz;
        if (log) {
            log->print(F("    cap ")); log->print(cap);
            log->print(F("  ")); log->print(hz);
            log->print(F(" Hz  err "));
            log->print(hz ? (err * 100.0f) / AS3935_LCO_TARGET_HZ : 100.0f, 2);
            log->println(F(" %"));
        }
        if (hz > 0 && err < bestErr) { bestErr = err; bestCap = cap; bestHz = hz; }
    }

    out.tuningCap = bestCap;
    out.lcoHz     = bestHz;

    if (bestHz == 0) {
        if (log) log->println(F("    NO LCO OUTPUT — antenna or IRQ wiring"));
        return false;
    }

    float errPct = (bestErr * 100.0f) / AS3935_LCO_TARGET_HZ;
    if (log) {
        log->print(F("    best cap ")); log->print(bestCap);
        log->print(F(" -> ")); log->print(bestHz);
        log->print(F(" Hz, err ")); log->print(errPct, 2);
        log->println(F(" %"));
    }

    if (errPct > AS3935_LCO_TOLERANCE * 100.0f) {
        if (log) {
            log->println(F("    OUT OF SPEC — distance estimates unreliable."));
            log->println(F("    Caps only lower the frequency; if the antenna"));
            log->println(F("    is already low there is no trim left."));
        }
        return false;
    }
    return true;
}

// ---- Stage 2: environment ----------------------------------
static volatile bool irqFlag = false;
static void onIrqFlag() { irqFlag = true; }

static void observe(AS3935 &dev, uint16_t seconds,
                    uint16_t &noise, uint16_t &disturber, uint16_t &strikes) {
    noise = disturber = strikes = 0;
    dev.readRegister(AS3935_REG_INT_MASK_ANT);      // drain anything pending

    pinMode(PIN_AS3935_IRQ, INPUT);
    irqFlag = false;
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onIrqFlag, RISING);

    uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)seconds * 1000UL) {
        IWatchdog.reload();
        if (!irqFlag) continue;
        irqFlag = false;
        delay(3);                                   // datasheet: 2 ms
        uint8_t src = dev.readRegister(AS3935_REG_INT_MASK_ANT) & 0x0F;
        if      (src == AS3935_INT_NOISE)     noise++;
        else if (src == AS3935_INT_DISTURBER) disturber++;
        else if (src == AS3935_INT_LIGHTNING) strikes++;
    }
    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));
}

static void tuneEnvironment(AS3935 &dev, NodeCalib &out, Print *log) {
    if (log) log->println(F("  [2/2] environment"));

    uint16_t noise, dist, strikes;

    // Noise floor first: a floor that is too low produces a constant
    // stream of noise interrupts that would swamp the watchdog tuning
    // below, so it has to settle before disturbers are meaningful.
    uint8_t floorLevel = 0;
    for (; floorLevel < 8; floorLevel++) {
        dev.setNoiseFloor(floorLevel);
        observe(dev, AS3935_CAL_NOISE_WINDOW_S, noise, dist, strikes);
        if (log) {
            log->print(F("    noise floor ")); log->print(floorLevel);
            log->print(F(" -> ")); log->print(noise);
            log->println(F(" noise events"));
        }
        if (noise <= AS3935_CAL_NOISE_LIMIT) break;
    }
    if (floorLevel > 7) floorLevel = 7;
    out.noiseFloor = floorLevel;

    // Then the watchdog. Disturbers are man-made transients; some are
    // unavoidable, so the target is a rate, not zero.
    uint8_t wd = 1;
    for (; wd < 11; wd++) {
        dev.setWatchdogThreshold(wd);
        observe(dev, AS3935_CAL_WD_WINDOW_S, noise, dist, strikes);
        float perMin = (dist * 60.0f) / AS3935_CAL_WD_WINDOW_S;
        if (log) {
            log->print(F("    watchdog ")); log->print(wd);
            log->print(F(" -> ")); log->print(perMin, 1);
            log->println(F(" disturbers/min"));
        }
        if (perMin <= AS3935_CAL_DISTURBER_PER_MIN) break;
    }
    if (wd > 10) wd = 10;
    out.watchdog    = wd;
    out.spikeReject = AS3935_SPIKE_REJECT;
}

// ---- Entry point -------------------------------------------
bool as3935Calibrate(AS3935 &dev, NodeCalib &out, Print *log) {
    if (log) log->println(F("AS3935 site calibration (about 2 min)"));

    if (AS3935_OUTDOOR_MODE) dev.setOutdoor(); else dev.setIndoor();
    dev.setMinLightningEvents(AS3935_MIN_STRIKES);
    dev.setSpikeRejection(AS3935_SPIKE_REJECT);

    bool antennaOk = tuneAntenna(dev, out, log);
    tuneEnvironment(dev, out, log);

    // Leave the device programmed with what we just measured.
    as3935ApplyCalib(dev, out);

    if (log) {
        log->print(F("  result: cap=")); log->print(out.tuningCap);
        log->print(F(" lco=")); log->print(out.lcoHz);
        log->print(F(" floor=")); log->print(out.noiseFloor);
        log->print(F(" wd=")); log->println(out.watchdog);
    }
    return antennaOk;
}

void as3935ApplyCalib(AS3935 &dev, const NodeCalib &c) {
    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0xF0, c.tuningCap & 0x0F);
    dev.setNoiseFloor(c.noiseFloor);
    dev.setWatchdogThreshold(c.watchdog);
    dev.setSpikeRejection(c.spikeReject);
    if (AS3935_OUTDOOR_MODE) dev.setOutdoor(); else dev.setIndoor();
    dev.setMinLightningEvents(AS3935_MIN_STRIKES);
    dev.readRegister(AS3935_REG_INT_MASK_ANT);      // clear any pending
}
