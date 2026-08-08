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
uint32_t as3935LastEdges   = 0;     // diagnostics for the last measurement
uint32_t as3935LastUs      = 0;
uint8_t  as3935LastReg03   = 0;
uint8_t  as3935LastReg08   = 0;
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

    // LCO_FDIV lives in reg 0x03 bits [7:6]: 00=/16, 01=/32, 10=/64,
    // 11=/128. Write it, then READ IT BACK — an assumed divider silently
    // scales every reading, and a /64 result looks like a real but wrong
    // frequency rather than an error.
    dev.maskRegisterBits(AS3935_REG_INT_MASK_ANT, 0x3F, 0x00);
    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0x7F, 0x80);   // LCO on

    as3935LastReg03 = dev.readRegister(AS3935_REG_INT_MASK_ANT);
    as3935LastReg08 = dev.readRegister(AS3935_REG_DISP_LCO);

    // Plain INPUT, never INPUT_PULLUP: the AS3935 drives this pin.
    pinMode(PIN_AS3935_IRQ, INPUT);
    delay(5);                                   // let the signal appear

    IWatchdog.reload();
    edgeCount = 0;
    calEndUs  = 0;
    uint32_t startUs = micros();
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onEdge, RISING);

    // The LCO runs at ~31 kHz — a 32 us period. Every other interrupt on
    // the board (GPS UART, uplink UART, USB, PPS) is capable of delaying
    // this handler past the next edge, and a missed edge stretches the
    // measured period instead of registering as an error. Give this line
    // the top priority for the duration of the count.
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);

    // Expected duration at the target frequency, with generous slack.
    // Long enough for the slowest divider (/128 -> ~3.9 kHz -> 256 ms),
    // so a timeout means "no signal", not "wrong divider".
    uint32_t deadline = millis() + 600;
    while (calEndUs == 0 && (int32_t)(millis() - deadline) < 0) { }

    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);      // back to normal
    IWatchdog.reload();

    dev.maskRegisterBits(AS3935_REG_DISP_LCO, 0x7F, 0x00);   // LCO off

    as3935LastEdges = edgeCount;
    as3935LastUs    = calEndUs ? (calEndUs - startUs) : 0;

    if (calEndUs == 0) return 0;                // never reached the count

    uint32_t elapsedUs = calEndUs - startUs;
    if (elapsedUs == 0) return 0;

    // Divider read back from the device, not assumed.
    uint32_t divider = 16UL << ((as3935LastReg03 >> 6) & 0x03);
    return (uint32_t)(((uint64_t)calTarget * 1000000ULL * divider) / elapsedUs);
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
            log->print(F("  r03=0x")); log->print(as3935LastReg03, HEX);
            log->print(F(" r08=0x")); log->print(as3935LastReg08, HEX);
            log->print(F(" div=")); log->print(16UL << ((as3935LastReg03 >> 6) & 3));
            log->print(F(" edges=")); log->print(as3935LastEdges);
            log->print(F(" us=")); log->print(as3935LastUs);
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
