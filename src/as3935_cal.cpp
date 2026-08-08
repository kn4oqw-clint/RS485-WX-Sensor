// ============================================================
// as3935_cal.cpp — antenna and environment calibration
// ============================================================

#include <IWatchdog.h>
#include "as3935_cal.h"
#include "config.h"

// ---- Edge counting on the IRQ pin --------------------------
static volatile uint32_t edgeCount = 0;
static void onEdge() { edgeCount++; }

// The lightning ISR must be detached while the pin is carrying the LCO
// or interrupt counts, and restored afterwards by the caller.
static void countEdges(uint16_t gateMs) {
    // The gate is a few hundred ms against an 8 s watchdog, so feeding it
    // either side is enough. Reloading inside the window was the only
    // difference from the board-test version that measured correctly.
    IWatchdog.reload();
    edgeCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onEdge, RISING);
    delay(gateMs);
    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));
    IWatchdog.reload();
}

uint32_t as3935MeasureLco(AS3935 &dev, uint8_t tuningCap, uint16_t gateMs) {
    // Toggling DISP_LCO off and straight back on does not reliably
    // restart the oscillator: measured back-to-back passes alternate
    // between a good count and exactly zero. Give it a clean off period
    // and a generous settle before counting.
    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0x7F, 0x00);   // ensure off
    delay(10);
    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0xF0, tuningCap & 0x0F);
    delay(10);

    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0x7F, 0x80);   // DISP_LCO on
    delay(50);                                               // let it settle

    countEdges(gateMs);

    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0x7F, 0x00);   // DISP_LCO off
    delay(3);

    // edges/sec * divider = antenna frequency
    return (uint32_t)((uint64_t)edgeCount * 1000ULL / gateMs) * AS3935_LCO_DIVIDER;
}

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
        uint32_t raw = 0;
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            uint32_t m = as3935MeasureLco(dev, cap, AS3935_LCO_GATE_MS);
            if (m > hz) { hz = m; raw = edgeCount; }
            if (hz > 0) break;          // a good reading is enough
        }

        uint32_t err = hz > AS3935_LCO_TARGET_HZ ? hz - AS3935_LCO_TARGET_HZ
                                                 : AS3935_LCO_TARGET_HZ - hz;
        if (log) {
            log->print(F("    cap ")); log->print(cap);
            log->print(F("  edges ")); log->print(raw);
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
// Counts interrupts by source over a window at the current settings.
static void observe(AS3935 &dev, uint16_t seconds,
                    uint16_t &noise, uint16_t &disturber, uint16_t &strikes) {
    noise = disturber = strikes = 0;
    dev.readRegister(AS3935_REG_INT_MASK_ANT);      // drain anything pending

    edgeCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onEdge, RISING);

    uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)seconds * 1000UL) {
        IWatchdog.reload();
        if (!edgeCount) continue;
        edgeCount = 0;
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
