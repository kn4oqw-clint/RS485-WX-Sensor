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
static bool tuneAntenna(AS3935SPI &dev, NodeCalib &out, Print *log) {
    if (log) log->println(F("  [1/2] antenna (AS3935MI)"));

    // The library drives the IRQ pin and manages its own ISR during
    // measurement, so nothing of ours may be attached here.
    dev.setInterruptMode(AS3935MI::AS3935_INTERRUPT_DETACHED);

    // NOTE: EXTI priority is raised globally via -D EXTI_IRQ_PRIO=0 in
    // platformio.ini, NOT here. STM32duino re-applies EXTI_IRQ_PRIO
    // inside every attachInterrupt(), so setting it around this call
    // would be silently undone the moment the library attaches its own
    // ISR. See the comment in platformio.ini.

    // Confirm the pin actually carries the oscillator before trusting
    // any frequency from it. ~14 ms, and it fails loudly instead of
    // returning a plausible wrong number.
    IWatchdog.reload();
    if (!dev.checkIRQ()) {
        if (log) log->println(F("    checkIRQ FAILED — no LCO on the IRQ pin"));
        return false;
    }

    IWatchdog.reload();
    int32_t freq = 0;
    bool ok = dev.calibrateResonanceFrequency(freq);
    IWatchdog.reload();

    out.tuningCap = dev.readAntennaTuning();
    out.lcoHz     = (freq > 0) ? (uint32_t)freq : 0;

    if (log) {
        log->print(F("    cap ")); log->print(out.tuningCap);
        log->print(F(" -> "));     log->print(out.lcoHz);
        log->print(F(" Hz  err "));
        long err = (long)out.lcoHz - 500000L;
        if (err < 0) err = -err;
        log->print((err * 100.0f) / 500000.0f, 2);
        log->println(F(" %"));
    }

    if (!ok && log) {
        log->println(F("    OUT OF SPEC — distance estimates unreliable."));
        log->println(F("    Caps only lower the frequency; if the antenna"));
        log->println(F("    already sits low there is no trim left."));
    }

    // RCO calibration must follow resonance calibration, per the
    // datasheet — the RC oscillators are trimmed against the antenna.
    if (!dev.calibrateRCO() && log)
        log->println(F("    RCO calibration reported failure"));

    return ok;
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
