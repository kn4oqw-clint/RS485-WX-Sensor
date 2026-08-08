// ============================================================
// as3935_cal.cpp — antenna (library) + environment (ours)
// ============================================================

#include <IWatchdog.h>
#include "as3935_cal.h"
#include "config.h"

// Encode a strike count into the register's 2-bit field.
static uint8_t minStrikesCode(uint8_t n) {
    if (n <= 1) return AS3935MI::AS3935_MNL_1;
    if (n <= 5) return AS3935MI::AS3935_MNL_5;
    if (n <= 9) return AS3935MI::AS3935_MNL_9;
    return AS3935MI::AS3935_MNL_16;
}

// ---- Stage 1: antenna --------------------------------------
//
// The LCO is driven through the library's public API, but counted here.
// AS3935MI::calibrateResonanceFrequency() sees zero edges on this board
// — its ISR never fires — while this loop counts 1000 every time on the
// same pin. Rather than keep chasing that, drive the device with the
// library (which gets the register semantics right) and do the counting
// locally, where it is known to work.
//
// Fixed sample count, timed. NOT edges-in-a-window: a window that misses
// edges reports a LOWER frequency, which is indistinguishable from a
// genuinely detuned antenna. This way a failure times out and reports
// zero instead of a plausible lie.
static volatile uint32_t edgeCount = 0;
static volatile uint32_t calEndUs  = 0;
static const uint32_t    CAL_SAMPLES = 1000;

static void onEdge() {
    if (edgeCount < CAL_SAMPLES) edgeCount++;
    else if (calEndUs == 0)      calEndUs = micros();
}

static uint32_t measureLco(AS3935SPI &dev, uint8_t cap) {
    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));

    // Registers with no ISR attached: servicing a 31 kHz interrupt while
    // driving the bus is what the library's own comments warn against.
    dev.displayLcoOnIrq(false);
    dev.writeAntennaTuning(cap);
    // Divider 32, not 16. At /16 the LCO lands at ~31 kHz and roughly 5%
    // of edges are lost to ISR overhead, which under-reports the
    // frequency and pushed a healthy antenna below the 3.5% window.
    // /32 halves the interrupt rate. AS3935MI defaults to /32 on
    // non-ESP platforms for exactly this reason.
    dev.writeDivisionRatio(AS3935MI::AS3935_DR_32);
    dev.displayLcoOnIrq(true);

    pinMode(PIN_AS3935_IRQ, INPUT);     // plain INPUT: the AS3935 drives it
    delay(20);                          // let the oscillator settle

    IWatchdog.reload();
    edgeCount = 0;
    calEndUs  = 0;
    uint32_t startUs = micros();
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onEdge, RISING);

    uint32_t deadline = millis() + 600; // covers even /128 at 3.9 kHz
    while (calEndUs == 0 && (int32_t)(millis() - deadline) < 0) { }

    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));
    IWatchdog.reload();
    dev.displayLcoOnIrq(false);

    if (calEndUs == 0) return 0;
    uint32_t elapsedUs = calEndUs - startUs;
    if (elapsedUs == 0) return 0;

    return (uint32_t)(((uint64_t)CAL_SAMPLES * 1000000ULL * 32ULL) / elapsedUs);
}

static bool tuneAntenna(AS3935SPI &dev, NodeCalib &out, Print *log) {
    if (log) log->println(F("  [1/2] antenna sweep"));

    dev.setInterruptMode(AS3935MI::AS3935_INTERRUPT_DETACHED);

    // Discard a warm-up reading. The first measurement after the
    // oscillator is first enabled reads far low — cap 0 came back at
    // 29 kHz against a true ~460 kHz — and without this the sweep
    // permanently writes off whichever cap happens to be measured first.
    measureLco(dev, 0);

    uint8_t  bestCap = 0;
    uint32_t bestHz  = 0;
    uint32_t bestErr = 0xFFFFFFFF;

    for (uint8_t cap = 0; cap < 16; cap++) {
        uint32_t hz = 0;
        for (uint8_t attempt = 0; attempt < 3 && hz == 0; attempt++)
            hz = measureLco(dev, cap);

        uint32_t err = hz > 500000UL ? hz - 500000UL : 500000UL - hz;
        if (log) {
            log->print(F("    cap ")); log->print(cap);
            log->print(F("  "));       log->print(hz);
            log->print(F(" Hz  err "));
            log->print(hz ? (err * 100.0f) / 500000.0f : 100.0f, 2);
            log->println(F(" %"));
        }
        if (hz > 0 && err < bestErr) { bestErr = err; bestCap = cap; bestHz = hz; }
    }

    out.tuningCap = bestCap;
    out.lcoHz     = bestHz;
    dev.writeAntennaTuning(bestCap);

    if (bestHz == 0) {
        if (log) log->println(F("    NO LCO OUTPUT — antenna or IRQ wiring"));
        return false;
    }

    float errPct = (bestErr * 100.0f) / 500000.0f;
    if (log) {
        log->print(F("    best cap ")); log->print(bestCap);
        log->print(F(" -> "));          log->print(bestHz);
        log->print(F(" Hz, err "));     log->print(errPct, 2);
        log->println(F(" %"));
    }

    // RCO calibration must follow the resonance calibration, per the
    // datasheet — the RC oscillators are trimmed against the antenna.
    if (!dev.calibrateRCO() && log)
        log->println(F("    RCO calibration reported failure"));

    if (errPct > AS3935_LCO_TOLERANCE * 100.0f) {
        if (log) {
            log->println(F("    OUT OF SPEC — distance estimates unreliable."));
            log->println(F("    Caps only lower the frequency; if the antenna"));
            log->println(F("    already sits low there is no trim left."));
        }
        return false;
    }
    return true;
}

// ---- Stage 2: environment ----------------------------------
static volatile bool irqFlag = false;
static void onIrqFlag() { irqFlag = true; }

static void observe(AS3935SPI &dev, uint16_t seconds,
                    uint16_t &noise, uint16_t &disturber, uint16_t &strikes) {
    noise = disturber = strikes = 0;

    dev.setInterruptMode(AS3935MI::AS3935_INTERRUPT_DETACHED);
    dev.readInterruptSource();                  // drain anything pending

    pinMode(PIN_AS3935_IRQ, INPUT);
    irqFlag = false;
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onIrqFlag, RISING);

    uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)seconds * 1000UL) {
        IWatchdog.reload();
        if (!irqFlag) continue;
        irqFlag = false;
        delay(3);                               // datasheet: 2 ms before read
        switch (dev.readInterruptSource()) {
            case AS3935MI::AS3935_INT_NH: noise++;     break;
            case AS3935MI::AS3935_INT_D:  disturber++; break;
            case AS3935MI::AS3935_INT_L:  strikes++;   break;
            default: break;
        }
    }
    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));
}

static void tuneEnvironment(AS3935SPI &dev, NodeCalib &out, Print *log) {
    if (log) log->println(F("  [2/2] environment"));

    uint16_t noise, dist, strikes;

    // Noise floor first. A floor set too low produces a constant stream
    // of noise interrupts that would swamp the watchdog measurement
    // below, so it has to settle before disturber rates mean anything.
    uint8_t floorLevel = 0;
    for (; floorLevel < 8; floorLevel++) {
        dev.writeNoiseFloorThreshold(floorLevel);
        observe(dev, AS3935_CAL_NOISE_WINDOW_S, noise, dist, strikes);
        if (log) {
            log->print(F("    noise floor ")); log->print(floorLevel);
            log->print(F(" -> "));             log->print(noise);
            log->println(F(" noise events"));
        }
        if (noise <= AS3935_CAL_NOISE_LIMIT) break;
    }
    if (floorLevel > 7) floorLevel = 7;
    out.noiseFloor = floorLevel;

    // Then the watchdog. Some man-made transients are unavoidable, so
    // the target is a rate, not zero.
    uint8_t wd = 1;
    for (; wd < 11; wd++) {
        dev.writeWatchdogThreshold(wd);
        observe(dev, AS3935_CAL_WD_WINDOW_S, noise, dist, strikes);
        float perMin = (dist * 60.0f) / AS3935_CAL_WD_WINDOW_S;
        if (log) {
            log->print(F("    watchdog ")); log->print(wd);
            log->print(F(" -> "));          log->print(perMin, 1);
            log->println(F(" disturbers/min"));
        }
        if (perMin <= AS3935_CAL_DISTURBER_PER_MIN) break;
    }
    if (wd > 10) wd = 10;
    out.watchdog    = wd;
    out.spikeReject = AS3935_SPIKE_REJECT;
}

// ---- Entry point -------------------------------------------
bool as3935Calibrate(AS3935SPI &dev, NodeCalib &out, Print *log) {
    if (log) log->println(F("AS3935 site calibration"));

    dev.writeAFE(AS3935_OUTDOOR_MODE ? AS3935MI::AS3935_OUTDOORS
                                     : AS3935MI::AS3935_INDOORS);
    dev.writeMinLightnings(minStrikesCode(AS3935_MIN_STRIKES));
    dev.writeSpikeRejection(AS3935_SPIKE_REJECT);

    bool antennaOk = tuneAntenna(dev, out, log);
    tuneEnvironment(dev, out, log);

    as3935ApplyCalib(dev, out);

    if (log) {
        log->print(F("  result: cap=")); log->print(out.tuningCap);
        log->print(F(" lco="));          log->print(out.lcoHz);
        log->print(F(" floor="));        log->print(out.noiseFloor);
        log->print(F(" wd="));           log->println(out.watchdog);
    }
    return antennaOk;
}

void as3935ApplyCalib(AS3935SPI &dev, const NodeCalib &c) {
    dev.writeAntennaTuning(c.tuningCap);
    dev.writeNoiseFloorThreshold(c.noiseFloor);
    dev.writeWatchdogThreshold(c.watchdog);
    dev.writeSpikeRejection(c.spikeReject);
    dev.writeAFE(AS3935_OUTDOOR_MODE ? AS3935MI::AS3935_OUTDOORS
                                     : AS3935MI::AS3935_INDOORS);
    dev.writeMinLightnings(minStrikesCode(AS3935_MIN_STRIKES));
    dev.readInterruptSource();                  // clear anything pending
}
