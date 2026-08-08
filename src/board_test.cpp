// ============================================================
// board_test.cpp — hardware bring-up test
//
// Probes every peripheral on the node and reports what it finds on the
// USB CDC console. Nothing here is flight firmware; it exists to answer
// "is this board wired correctly" before any of the real logic lands.
//
// Deliberately has no library dependencies beyond Wire/SPI. When a probe
// fails you want to be sure the failure is the hardware and not somebody
// else's driver.
//
// Build:  pio run -e boardtest -t upload
// Watch:  pio device monitor -e boardtest
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "config.h"

// ---- Result tracking ---------------------------------------
enum Result { PASS, WARN, FAIL, SKIP };

struct TestRow {
    const char *name;
    Result      result;
    char        detail[48];
};

static TestRow rows[16];
static uint8_t rowCount = 0;

static void record(const char *name, Result r, const char *detail) {
    if (rowCount >= 16) return;
    rows[rowCount].name   = name;
    rows[rowCount].result = r;
    strncpy(rows[rowCount].detail, detail, sizeof(rows[0].detail) - 1);
    rows[rowCount].detail[sizeof(rows[0].detail) - 1] = '\0';
    rowCount++;
}

static const char *resultText(Result r) {
    switch (r) {
        case PASS: return "PASS";
        case WARN: return "WARN";
        case FAIL: return "FAIL";
        default:   return "SKIP";
    }
}

static void banner(const char *title) {
    CONSOLE.println();
    CONSOLE.print(F("---- "));
    CONSOLE.print(title);
    CONSOLE.println(F(" ----------------------------------"));
}

// Avoid %f in printf — newlib-nano is linked without float support in
// vsnprintf unless -u _printf_float is passed, and silently prints
// garbage otherwise. print(value, decimals) always works.
static void printHex8(uint8_t v) {
    if (v < 0x10) CONSOLE.print('0');
    CONSOLE.print(v, HEX);
}

// ============================================================
// SPI helpers — one SPISettings per device, applied per transaction
// ============================================================

static const SPISettings BME_SPI(BME680_SPI_HZ, MSBFIRST, SPI_MODE0);
static const SPISettings LGT_SPI(AS3935_SPI_HZ, MSBFIRST, SPI_MODE1);

static uint8_t bmeRead(uint8_t reg) {
    SPI.beginTransaction(BME_SPI);
    digitalWrite(PIN_BME680_CS, LOW);
    SPI.transfer(reg | 0x80);           // bit7 = 1 -> read
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(PIN_BME680_CS, HIGH);
    SPI.endTransaction();
    return v;
}

static void bmeWrite(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(BME_SPI);
    digitalWrite(PIN_BME680_CS, LOW);
    SPI.transfer(reg & 0x7F);           // bit7 = 0 -> write
    SPI.transfer(val);
    digitalWrite(PIN_BME680_CS, HIGH);
    SPI.endTransaction();
}

static uint8_t as3935Read(uint8_t reg) {
    SPI.beginTransaction(LGT_SPI);
    digitalWrite(PIN_AS3935_CS, LOW);
    delayMicroseconds(1);
    SPI.transfer((reg & 0x3F) | 0x40);  // bits[7:6] = 01 -> read
    uint8_t v = SPI.transfer(0x00);
    delayMicroseconds(1);
    digitalWrite(PIN_AS3935_CS, HIGH);
    SPI.endTransaction();
    return v;
}

static void as3935Write(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(LGT_SPI);
    digitalWrite(PIN_AS3935_CS, LOW);
    delayMicroseconds(1);
    SPI.transfer(reg & 0x3F);           // bits[7:6] = 00 -> write
    SPI.transfer(val);
    delayMicroseconds(1);
    digitalWrite(PIN_AS3935_CS, HIGH);
    SPI.endTransaction();
}

static void as3935Mask(uint8_t reg, uint8_t keepMask, uint8_t val) {
    as3935Write(reg, (as3935Read(reg) & keepMask) | val);
}

// ============================================================
// Test 1 — BME680 over SPI1
// ============================================================
static void testBME680() {
    banner("BME680 (SPI1, CS PA4, MODE0)");

    uint8_t id = bmeRead(0xD0);         // chip ID register
    CONSOLE.print(F("  chip ID 0xD0 = 0x"));
    printHex8(id);
    CONSOLE.println();

    if (id != 0x61) {
        // Might be parked on memory page 1. Soft-reset returns it to page 0.
        CONSOLE.println(F("  not 0x61 — soft reset and retry"));
        bmeWrite(0xE0, 0xB6);
        delay(10);
        id = bmeRead(0xD0);
        CONSOLE.print(F("  chip ID after reset = 0x"));
        printHex8(id);
        CONSOLE.println();
    }

    char d[48];
    if (id == 0x61) {
        // Calibration data is device-specific; all-zero or all-FF means we
        // are reading the bus, not the part.
        uint8_t c1 = bmeRead(0x8A), c2 = bmeRead(0x8B), c3 = bmeRead(0xE1);
        CONSOLE.print(F("  calib bytes: "));
        printHex8(c1); CONSOLE.print(' ');
        printHex8(c2); CONSOLE.print(' ');
        printHex8(c3); CONSOLE.println();

        bool calibOk = !((c1 == 0x00 && c2 == 0x00 && c3 == 0x00) ||
                         (c1 == 0xFF && c2 == 0xFF && c3 == 0xFF));
        if (calibOk) {
            record("BME680", PASS, "chip ID 0x61, calib data present");
        } else {
            record("BME680", WARN, "ID ok but calib blank - check MISO");
        }
    } else if (id == 0x00 || id == 0xFF) {
        snprintf(d, sizeof(d), "no response (0x%02X) - MISO/CS/power?", id);
        record("BME680", FAIL, d);
    } else {
        snprintf(d, sizeof(d), "wrong ID 0x%02X (expected 0x61)", id);
        record("BME680", FAIL, d);
    }
}

// ============================================================
// Test 2 — AS3935 over SPI1
// ============================================================
static void testAS3935() {
    banner("AS3935 (SPI1, CS PB0, MODE1)");

    as3935Write(0x3C, 0x96);            // preset defaults
    delay(3);

    // CALIB_RCO is not complete until TRCO has been displayed on the IRQ
    // pin and switched off again. Issue the command alone and the chip
    // leaves IRQ asserted with an INT source of 0x00 — which reads exactly
    // like a stuck line and cannot be cleared by reading reg 0x03.
    as3935Write(0x3D, 0x96);            // CALIB_RCO
    delay(3);
    as3935Mask(0x08, 0xDF, 0x20);       // DISP_TRCO on
    delay(3);
    as3935Mask(0x08, 0xDF, 0x00);       // DISP_TRCO off
    delay(3);
    as3935Read(0x03);                   // drain the calibration interrupt
    delay(3);

    uint8_t afe = as3935Read(0x00);
    CONSOLE.print(F("  reg 0x00 (AFE gain) = 0x"));
    printHex8(afe);
    CONSOLE.println(F("   [0x24 = indoor default]"));

    CONSOLE.print(F("  reg dump 0x00-0x08:"));
    for (uint8_t r = 0x00; r <= 0x08; r++) {
        CONSOLE.print(' ');
        printHex8(as3935Read(r));
    }
    CONSOLE.println();

    uint8_t trco = as3935Read(0x3A);
    uint8_t srco = as3935Read(0x3B);
    CONSOLE.print(F("  TRCO calib 0x3A = 0x")); printHex8(trco);
    CONSOLE.print(F("   SRCO calib 0x3B = 0x")); printHex8(srco);
    CONSOLE.println();

    char d[48];
    if (afe == 0x00 || afe == 0xFF) {
        snprintf(d, sizeof(d), "no response (0x%02X) - check SI tied low", afe);
        record("AS3935", FAIL, d);
        return;
    }

    // Prove the part is writable, not just readable.
    as3935Mask(0x01, 0x8F, 5 << 4);     // noise floor = 5
    uint8_t back = (as3935Read(0x01) >> 4) & 0x07;
    as3935Mask(0x01, 0x8F, AS3935_NOISE_FLOOR << 4);

    if (back != 5) {
        snprintf(d, sizeof(d), "readable but write failed (got %u)", back);
        record("AS3935", FAIL, d);
        return;
    }

    // Bit 6 of the calib registers is the "calibration failed" flag.
    if ((trco & 0x40) || (srco & 0x40)) {
        record("AS3935", WARN, "RC calibration reported failure");
    } else {
        snprintf(d, sizeof(d), "AFE 0x%02X, r/w ok, RC calib ok", afe);
        record("AS3935", PASS, d);
    }
}

// ============================================================
// Test 3 — shared SPI bus, mode switching
//
// The whole reason this test exists: BME680 wants MODE0 and AS3935 wants
// MODE1 on the same physical bus. If beginTransaction() is not correctly
// reprogramming CPOL/CPHA, one device works and the other returns junk —
// and which one depends purely on access order.
// ============================================================
static void testSharedBus() {
    banner("Shared SPI1 bus / mode switching");

    uint8_t bmeFails = 0, lgtFails = 0;
    for (uint8_t i = 0; i < 20; i++) {
        if (bmeRead(0xD0) != 0x61)              bmeFails++;
        uint8_t a = as3935Read(0x00);
        if (a == 0x00 || a == 0xFF)             lgtFails++;
    }

    CONSOLE.print(F("  20 interleaved round-trips: BME680 errors="));
    CONSOLE.print(bmeFails);
    CONSOLE.print(F("  AS3935 errors="));
    CONSOLE.println(lgtFails);

    char d[48];
    if (bmeFails == 0 && lgtFails == 0) {
        record("SPI bus share", PASS, "20/20 interleaved reads clean");
    } else {
        snprintf(d, sizeof(d), "bme_err=%u as3935_err=%u", bmeFails, lgtFails);
        record("SPI bus share", FAIL, d);
    }
}

// ============================================================
// Test 4 — AS3935 IRQ line
// ============================================================
static volatile uint32_t irqCount = 0;
static void onLightningIRQ() { irqCount++; }

static void testIRQLine() {
    banner("AS3935 IRQ line (PA8)");

    pinMode(PIN_AS3935_IRQ, INPUT);
    delay(2);
    int idle = digitalRead(PIN_AS3935_IRQ);
    CONSOLE.print(F("  idle level = "));
    CONSOLE.println(idle ? F("HIGH  <-- expected LOW when no event pending")
                         : F("LOW   (correct)"));

    // A pending interrupt holds IRQ high until reg 0x03 is read. The
    // AS3935 raises one after power-up and after the RC calibration
    // command, so a high line here is expected, not a fault — as long as
    // reading the register drops it.
    //
    // The datasheet mandates a 2 ms wait between the IRQ edge and the INT
    // register read. Read it earlier and you get 0x00 back AND the latch
    // does not clear, which looks exactly like a wiring fault. Wait first,
    // and retry — a reset+calibrate can leave more than one pending.
    for (uint8_t attempt = 0; attempt < 3 && idle == HIGH; attempt++) {
        delay(3);
        uint8_t src = as3935Read(0x03) & 0x0F;
        delay(3);
        idle = digitalRead(PIN_AS3935_IRQ);
        CONSOLE.print(F("  INT read "));
        CONSOLE.print(attempt + 1);
        CONSOLE.print(F(": 0x03 = 0x"));
        printHex8(src);
        CONSOLE.print(F("  -> line "));
        CONSOLE.println(idle ? F("still high") : F("LOW (cleared)"));
    }

    irqCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onLightningIRQ, RISING);
    delay(1000);
    uint32_t n = irqCount;

    CONSOLE.print(F("  interrupts in 1 s = "));
    CONSOLE.println(n);

    char d[48];
    if (idle == HIGH) {
        record("AS3935 IRQ", FAIL, "stuck high after INT read - check PA8");
    } else if (n > 50) {
        snprintf(d, sizeof(d), "%lu irq/s at idle - EMI or bad tuning", n);
        record("AS3935 IRQ", WARN, d);
    } else {
        snprintf(d, sizeof(d), "idle low, %lu spurious irq in 1s", n);
        record("AS3935 IRQ", PASS, d);
    }
}

// ============================================================
// Test 5 — I2C1 bus scan + DS3231
// ============================================================
static bool i2cRecover() {
    // If a slave was interrupted mid-read it can hold SDA low forever.
    // Clocking SCL manually walks it out of that state.
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    if (digitalRead(PIN_I2C_SDA) == HIGH) return true;

    CONSOLE.println(F("  SDA stuck low — clocking bus free"));
    pinMode(PIN_I2C_SCL, OUTPUT);
    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(5);
        digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
    }
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    return digitalRead(PIN_I2C_SDA) == HIGH;
}

static uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }

static void testI2C() {
    banner("I2C1 bus (SCL PB6 / SDA PB7)");

    if (!i2cRecover()) {
        record("I2C bus", FAIL, "SDA held low - unplug devices to isolate");
        return;
    }

    Wire.setSCL(PIN_I2C_SCL);
    Wire.setSDA(PIN_I2C_SDA);
    Wire.begin();
    Wire.setClock(I2C_HZ);

    CONSOLE.println(F("  scanning 0x08..0x77"));
    uint8_t found = 0;
    bool sawRTC = false, sawINA = false;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            CONSOLE.print(F("    found 0x"));
            printHex8(addr);
            if (addr == ADDR_DS3231) { CONSOLE.print(F("  (DS3231)")); sawRTC = true; }
            if (addr == ADDR_INA226) { CONSOLE.print(F("  (INA226)")); sawINA = true; }
            CONSOLE.println();
            found++;
        }
    }

    CONSOLE.print(F("  devices found: "));
    CONSOLE.println(found);

    char d[48];
    if (found == 0) {
        record("I2C bus", FAIL, "no devices - check pullups and 3V3");
    } else {
        snprintf(d, sizeof(d), "%u device(s) responding", found);
        record("I2C bus", PASS, d);
    }
    if (sawINA) record("INA226", PASS, "present (ahead of schedule)");

    if (!sawRTC) {
        record("DS3231", FAIL, "not at 0x68");
        return;
    }

    // ---- DS3231 register read ------------------------------
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        record("DS3231", FAIL, "addressed but write failed");
        return;
    }
    if (Wire.requestFrom((uint8_t)ADDR_DS3231, (uint8_t)7) != 7) {
        record("DS3231", FAIL, "short read on time registers");
        return;
    }

    uint8_t ss = bcd2dec(Wire.read() & 0x7F);
    uint8_t mm = bcd2dec(Wire.read() & 0x7F);
    uint8_t hh = bcd2dec(Wire.read() & 0x3F);
    Wire.read();                                  // day of week
    uint8_t dd = bcd2dec(Wire.read() & 0x3F);
    uint8_t mo = bcd2dec(Wire.read() & 0x1F);
    uint8_t yy = bcd2dec(Wire.read());

    CONSOLE.print(F("  time: 20"));
    if (yy < 10) CONSOLE.print('0'); CONSOLE.print(yy); CONSOLE.print('-');
    if (mo < 10) CONSOLE.print('0'); CONSOLE.print(mo); CONSOLE.print('-');
    if (dd < 10) CONSOLE.print('0'); CONSOLE.print(dd); CONSOLE.print(' ');
    if (hh < 10) CONSOLE.print('0'); CONSOLE.print(hh); CONSOLE.print(':');
    if (mm < 10) CONSOLE.print('0'); CONSOLE.print(mm); CONSOLE.print(':');
    if (ss < 10) CONSOLE.print('0'); CONSOLE.print(ss);
    CONSOLE.println(F("  UTC"));

    // Status register 0x0F: bit7 = oscillator stop flag
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x0F);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADDR_DS3231, (uint8_t)1);
    uint8_t status = Wire.read();
    bool osf = status & 0x80;

    // Temperature registers 0x11/0x12 — die temp, also proves the part is
    // really a DS3231 and not a bare DS1307 module.
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x11);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADDR_DS3231, (uint8_t)2);
    int8_t  tHi = (int8_t)Wire.read();
    uint8_t tLo = Wire.read() >> 6;
    float   dieTemp = tHi + tLo * 0.25f;

    CONSOLE.print(F("  die temp: "));
    CONSOLE.print(dieTemp, 2);
    CONSOLE.println(F(" C"));
    CONSOLE.print(F("  status 0x0F = 0x"));
    printHex8(status);
    CONSOLE.println(osf ? F("  OSF SET - clock lost power")
                        : F("  OSF clear - clock has run continuously"));

    // Confirm it is actually ticking.
    uint8_t s1 = ss;
    delay(1200);
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADDR_DS3231, (uint8_t)1);
    uint8_t s2 = bcd2dec(Wire.read() & 0x7F);
    bool ticking = (s1 != s2);

    CONSOLE.print(F("  ticking: "));
    CONSOLE.println(ticking ? F("yes") : F("NO - oscillator halted"));

    if (!ticking) {
        record("DS3231", FAIL, "present but not ticking (EOSC bit?)");
    } else if (osf) {
        record("DS3231", WARN, "ticking but OSF set - time not trusted");
    } else if (dieTemp < -40.0f || dieTemp > 85.0f) {
        record("DS3231", WARN, "ticking, die temp implausible");
    } else {
        record("DS3231", PASS, "ticking, OSF clear, temp sane");
    }
}

// ============================================================
// Seed the DS3231 and clear OSF.
//
// OSF (status bit 7) is sticky: once the oscillator has stopped, it stays
// set forever until software clears it. So "OSF set" on a clock that has
// never been initialised tells you nothing about the backup cell — you
// have to clear it, pull all power, and see whether it comes back.
//
// Build time is only as good as the build machine's clock and ignores
// timezone. Fine for a retention test; GPS is the real time source.
// ============================================================
static uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

static void seedRTC() {
    banner("DS3231 seed + OSF clear");

    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char mstr[8] = {0};
    int  day = 1, year = 2000, hh = 0, mm = 0, ss = 0;

    sscanf(__DATE__, "%7s %d %d", mstr, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);

    const char *pos = strstr(months, mstr);
    uint8_t mon = pos ? ((pos - months) / 3) + 1 : 1;

    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x00);
    Wire.write(dec2bcd(ss));
    Wire.write(dec2bcd(mm));
    Wire.write(dec2bcd(hh));            // 24-hour: bit6 clear
    Wire.write(1);                      // day of week, unused
    Wire.write(dec2bcd(day));
    Wire.write(dec2bcd(mon));           // century bit clear
    Wire.write(dec2bcd(year % 100));
    if (Wire.endTransmission() != 0) {
        CONSOLE.println(F("  write failed"));
        return;
    }

    // Clear OSF so a later power cycle tells us something.
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x0F);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADDR_DS3231, (uint8_t)1);
    uint8_t status = Wire.read();

    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x0F);
    Wire.write(status & 0x7F);
    Wire.endTransmission();

    CONSOLE.print(F("  set to "));
    CONSOLE.print(year);   CONSOLE.print('-');
    CONSOLE.print(mon);    CONSOLE.print('-');
    CONSOLE.print(day);    CONSOLE.print(' ');
    CONSOLE.print(hh);     CONSOLE.print(':');
    CONSOLE.print(mm);     CONSOLE.print(':');
    CONSOLE.println(ss);
    CONSOLE.println(F("  OSF cleared."));
    CONSOLE.println(F("  Now: unplug ALL power for 60 s, replug, press 't'."));
    CONSOLE.println(F("    time retained + OSF clear -> backup cell good"));
    CONSOLE.println(F("    back to 2000-01-01 or OSF set -> cell dead/absent"));
}

// Set the clock from a host-supplied string, so the RTC holds real UTC
// while the GPS is out of action. Format, terminated by newline:
//
//     s2026-08-08 15:52:00
//
static void setRTCFromHost() {
    char buf[32] = {0};
    uint8_t n = 0;
    uint32_t start = millis();

    while (millis() - start < 3000 && n < sizeof(buf) - 1) {
        if (!CONSOLE.available()) continue;
        char c = CONSOLE.read();
        if (c == '\n' || c == '\r') break;
        buf[n++] = c;
    }

    int year, mon, day, hh, mm, ss;
    if (sscanf(buf, "%d-%d-%d %d:%d:%d", &year, &mon, &day, &hh, &mm, &ss) != 6) {
        CONSOLE.print(F("  bad format: '"));
        CONSOLE.print(buf);
        CONSOLE.println(F("' — expected s2026-08-08 15:52:00"));
        return;
    }

    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x00);
    Wire.write(dec2bcd(ss));
    Wire.write(dec2bcd(mm));
    Wire.write(dec2bcd(hh));
    Wire.write(1);
    Wire.write(dec2bcd(day));
    Wire.write(dec2bcd(mon));
    Wire.write(dec2bcd(year % 100));
    if (Wire.endTransmission() != 0) {
        CONSOLE.println(F("  write failed"));
        return;
    }

    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x0F);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADDR_DS3231, (uint8_t)1);
    uint8_t status = Wire.read();
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(0x0F);
    Wire.write(status & 0x7F);
    Wire.endTransmission();

    CONSOLE.print(F("  RTC set to "));
    CONSOLE.print(buf);
    CONSOLE.println(F(" UTC, OSF cleared"));
}

// ============================================================
// Test 6 — GPS on USART1
// ============================================================
// Is anything actually driving the RX line? A byte count of zero cannot
// tell "module unpowered" from "module talking at the wrong baud", but
// the electrical state of the pin can.
//
//   pulldown HIGH + pullup HIGH -> actively driven high = UART idle. Good.
//   pulldown LOW  + pullup HIGH -> floating. Nothing connected or no power.
//   pulldown LOW  + pullup LOW  -> held low. Module in reset, or TX/RX swap.
// Returns: 2 = driven high (idle UART), 1 = driven low, 0 = floating.
static uint8_t probePin(uint32_t pin, const char *label) {
    pinMode(pin, INPUT_PULLDOWN);
    delay(5);
    int withPD = digitalRead(pin);
    pinMode(pin, INPUT_PULLUP);
    delay(5);
    int withPU = digitalRead(pin);
    pinMode(pin, INPUT);
    delay(5);

    uint32_t trans = 0;
    int last = digitalRead(pin);
    uint32_t start = millis();
    while (millis() - start < 250) {
        int now = digitalRead(pin);
        if (now != last) { trans++; last = now; }
    }

    CONSOLE.print(F("  "));
    CONSOLE.print(label);
    CONSOLE.print(F(": pulldown="));
    CONSOLE.print(withPD ? F("HIGH") : F("LOW "));
    CONSOLE.print(F(" pullup="));
    CONSOLE.print(withPU ? F("HIGH") : F("LOW "));
    CONSOLE.print(F(" edges="));
    CONSOLE.print(trans);

    if (withPD == HIGH && withPU == HIGH) {
        CONSOLE.println(F("  -> DRIVEN HIGH (something is talking)"));
        return 2;
    }
    if (withPD == LOW && withPU == LOW) {
        CONSOLE.println(F("  -> DRIVEN LOW"));
        return 1;
    }
    CONSOLE.println(F("  -> FLOATING (open circuit)"));
    return 0;
}

static bool probeGPSLine() {
    // PA10 is where the GPS TX should land. PA9 is our TX to the module —
    // probing it as an input catches a TX/RX swap at the module end, which
    // would show the GPS driving PA9 instead.
    uint8_t rx = probePin(PA10, "PA10 (GPS TX in) ");
    uint8_t tx = probePin(PA9,  "PA9  (our TX out)");

    if (rx == 2) return true;

    if (tx == 2) {
        CONSOLE.println(F("  !! PA9 is driven but PA10 is not — the module's"));
        CONSOLE.println(F("  !! TX is on OUR TX pin. TX/RX are swapped."));
        return false;
    }

    CONSOLE.println(F("  PA10 floats with the module powered, so nothing is"));
    CONSOLE.println(F("  driving it. In order of likelihood:"));
    CONSOLE.println(F("    1. open joint / broken wire on the TX line"));
    CONSOLE.println(F("    2. GPS TX pin dead (module reworked?)"));
    CONSOLE.println(F("    3. module has power but is not running"));
    CONSOLE.println(F("  Isolate: wire GPS TX straight to a USB-serial"));
    CONSOLE.println(F("  adapter at 9600 and see if NMEA appears there."));
    return false;
}

// If the line is alive but 9600 gives nothing, the module may have been
// left configured at another rate. Sweep the common ones.
static uint32_t sweepGPSBaud() {
    static const uint32_t bauds[] = {9600, 4800, 38400, 57600, 115200};
    CONSOLE.println(F("  sweeping baud rates"));

    for (uint8_t i = 0; i < 5; i++) {
        SerialGPS.end();
        SerialGPS.setRx(PA10);
        SerialGPS.setTx(PA9);
        SerialGPS.begin(bauds[i]);
        delay(30);
        while (SerialGPS.available()) SerialGPS.read();

        uint16_t dollars = 0, bytes = 0;
        uint32_t start = millis();
        while (millis() - start < 1500) {
            while (SerialGPS.available()) {
                char c = SerialGPS.read();
                bytes++;
                if (c == '$') dollars++;
            }
        }
        CONSOLE.print(F("    "));
        CONSOLE.print(bauds[i]);
        CONSOLE.print(F(": "));
        CONSOLE.print(bytes);
        CONSOLE.print(F(" bytes, "));
        CONSOLE.print(dollars);
        CONSOLE.println(F(" '$'"));

        if (dollars >= 2) return bauds[i];
    }
    return 0;
}

static void testGPS() {
    banner("GPS GT-U7 (USART1, RX PA10 / TX PA9)");

    // Pin the peripheral explicitly. PB6/PB7 are also valid USART1 pins on
    // this part and they are our I2C bus — never let pinmap order decide.
    SerialGPS.setRx(PA10);
    SerialGPS.setTx(PA9);

    bool lineAlive = probeGPSLine();

    SerialGPS.begin(GPS_BAUD);
    delay(50);
    while (SerialGPS.available()) SerialGPS.read();

    CONSOLE.println(F("  listening 5 s at 9600 baud"));

    uint32_t bytes = 0, sentences = 0;
    char     line[100];
    uint8_t  len = 0;
    char     firstLine[100] = {0};
    bool     haveFix = false;
    uint8_t  sats = 0;

    uint32_t start = millis();
    while (millis() - start < 5000) {
        while (SerialGPS.available()) {
            char c = SerialGPS.read();
            bytes++;
            if (c == '\n') {
                line[len] = '\0';
                if (len > 6 && line[0] == '$') {
                    sentences++;
                    if (firstLine[0] == '\0') strncpy(firstLine, line, sizeof(firstLine) - 1);
                    // $xxGGA,time,lat,N,lon,E,fixQuality,numSats,...
                    if (strstr(line, "GGA")) {
                        char *p = line;
                        for (uint8_t f = 0; f < 6 && p; f++) p = strchr(p + 1, ',');
                        if (p && p[1] != ',' && p[1] != '0') haveFix = true;
                        if (p) {
                            char *q = strchr(p + 1, ',');
                            if (q) sats = atoi(q + 1);
                        }
                    }
                }
                len = 0;
            } else if (c != '\r' && len < sizeof(line) - 1) {
                line[len++] = c;
            }
        }
    }

    CONSOLE.print(F("  bytes received: "));   CONSOLE.println(bytes);
    CONSOLE.print(F("  NMEA sentences: "));   CONSOLE.println(sentences);
    if (firstLine[0]) {
        CONSOLE.print(F("  first sentence: "));
        CONSOLE.println(firstLine);
    }
    CONSOLE.print(F("  satellites: "));       CONSOLE.println(sats);
    CONSOLE.print(F("  fix: "));
    CONSOLE.println(haveFix ? F("YES") : F("no (cold start indoors is normal)"));

    char d[48];
    if (bytes == 0) {
        if (!lineAlive) {
            record("GPS", FAIL, "RX line dead - no 5V or TX not wired");
        } else {
            uint32_t found = sweepGPSBaud();
            if (found) {
                snprintf(d, sizeof(d), "talks at %lu baud, not 9600", found);
                record("GPS", WARN, d);
            } else {
                record("GPS", FAIL, "line alive but silent at every baud");
            }
        }
    } else if (sentences == 0) {
        snprintf(d, sizeof(d), "%lu bytes but no NMEA - baud wrong?", bytes);
        record("GPS", FAIL, d);
    } else if (haveFix) {
        snprintf(d, sizeof(d), "%lu sentences/5s, fix, %u sats", sentences, sats);
        record("GPS", PASS, d);
    } else {
        snprintf(d, sizeof(d), "%lu sentences/5s, no fix yet", sentences);
        record("GPS", PASS, d);
    }
}

static volatile uint32_t ppsCount = 0;
static void onPPS() { ppsCount++; }

// ============================================================
// UBX helpers
// ============================================================
static void sendUBX(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t head[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t ckA = 0, ckB = 0;

    for (uint8_t i = 2; i < 6; i++) { ckA += head[i]; ckB += ckA; }
    for (uint16_t i = 0; i < len; i++) { ckA += payload[i]; ckB += ckA; }

    SerialGPS.write(head, 6);
    if (len) SerialGPS.write(payload, len);
    SerialGPS.write(ckA);
    SerialGPS.write(ckB);
    SerialGPS.flush();
}

// Force a free-running 1 Hz timepulse so PPS can be verified without a fix.
//
// CFG-TP5 carries two pulse configs: freqPeriod applies when the receiver is
// NOT locked to GNSS, freqPeriodLock when it is. The default unlocked value
// is zero, which is exactly why a module with no fix shows no PPS. Setting
// the unlocked value to 1 Hz proves out the wiring and the EXTI path now,
// and the locked config still takes over once a fix arrives.
static void forcePPSUnlocked() {
    uint8_t p[32] = {0};

    p[0]  = 0;                      // tpIdx = TIMEPULSE
    p[1]  = 0;                      // version
                                    // p[2..3] reserved
                                    // p[4..5] antCableDelay = 0
                                    // p[6..7] rfGroupDelay  = 0
    p[8]  = 1;                      // freqPeriod = 1 Hz (unlocked)
    p[12] = 1;                      // freqPeriodLock = 1 Hz (locked)

    // pulseLenRatio = 100000 us = 100 ms high time, both states
    uint32_t plen = 100000;
    memcpy(&p[16], &plen, 4);
    memcpy(&p[20], &plen, 4);
                                    // p[24..27] userConfigDelay = 0

    // active | lockGnssFreq | lockedOtherSet | isFreq | isLength |
    // alignToTow | polarity(rising)
    uint32_t flags = 0x7F;
    memcpy(&p[28], &flags, 4);

    sendUBX(0x06, 0x31, p, 32);
}

static void testPPSForced() {
    banner("PPS forced 1 Hz (no fix required)");

    SerialGPS.setRx(PA10);
    SerialGPS.setTx(PA9);
    SerialGPS.begin(GPS_BAUD);
    delay(50);

    CONSOLE.println(F("  sending UBX-CFG-TP5 (1 Hz while unlocked)"));
    forcePPSUnlocked();
    delay(500);

    pinMode(PIN_GPS_PPS, INPUT);
    ppsCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), onPPS, RISING);
    CONSOLE.println(F("  counting rising edges for 6 s"));
    delay(6000);
    detachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS));

    CONSOLE.print(F("  edges: "));
    CONSOLE.println(ppsCount);

    if (ppsCount >= 5 && ppsCount <= 7) {
        CONSOLE.println(F("  PPS WIRING CONFIRMED — 1 Hz on PA0, EXTI works."));
        CONSOLE.println(F("  Accuracy is meaningless until locked, but the"));
        CONSOLE.println(F("  signal path is proven."));
    } else if (ppsCount == 0) {
        CONSOLE.println(F("  still nothing. Either PA0 is not wired to the"));
        CONSOLE.println(F("  module's PPS pin, or the module rejected the"));
        CONSOLE.println(F("  config (not a real u-blox?). Probe the pin:"));
        probePin(PIN_GPS_PPS, "PA0 (PPS in)");
    } else {
        CONSOLE.println(F("  unexpected rate — check for bounce or EMI"));
    }
}

// ============================================================
// GPS sky view — what the antenna can actually hear.
// Satellites tracked with usable SNR is the number that decides whether a
// fix is coming; a receiver by a window often sees birds but never gets
// enough of them above ~30 dBHz to solve.
// ============================================================
static void gpsSkyView() {
    banner("GPS sky view (30 s)");

    SerialGPS.setRx(PA10);
    SerialGPS.setTx(PA9);
    SerialGPS.begin(GPS_BAUD);
    delay(50);
    while (SerialGPS.available()) SerialGPS.read();

    char    line[100];
    uint8_t len = 0;
    uint8_t inView = 0, bestSnr = 0, usable = 0;
    bool    fix = false;

    uint32_t start = millis();
    while (millis() - start < 30000) {
        while (SerialGPS.available()) {
            char c = SerialGPS.read();
            if (c == '\n') {
                line[len] = '\0';
                if (strstr(line, "GSV")) {
                    // $xxGSV,numMsg,msgNum,numSV, then svid,elev,azim,snr x4
                    char *f[24] = {0};
                    uint8_t nf = 0;
                    for (char *p = line; *p && nf < 24; p++)
                        if (*p == ',') { *p = '\0'; f[nf++] = p + 1; }
                    if (nf >= 3 && f[2]) inView = atoi(f[2]);
                    uint8_t thisMsg = 0;
                    for (uint8_t i = 6; i < nf; i += 4) {
                        if (!f[i]) continue;
                        uint8_t snr = atoi(f[i]);
                        if (snr > bestSnr) bestSnr = snr;
                        if (snr >= 30) thisMsg++;
                    }
                    usable += thisMsg;
                } else if (strstr(line, "GGA")) {
                    char *p = line;
                    for (uint8_t k = 0; k < 6 && p; k++) p = strchr(p + 1, ',');
                    if (p && p[1] != ',' && p[1] != '0') fix = true;
                }
                len = 0;
            } else if (c != '\r' && len < sizeof(line) - 1) {
                line[len++] = c;
            }
        }
    }

    CONSOLE.print(F("  satellites in view: ")); CONSOLE.println(inView);
    CONSOLE.print(F("  best SNR: "));           CONSOLE.print(bestSnr);
    CONSOLE.println(F(" dBHz"));
    CONSOLE.print(F("  fix: "));
    CONSOLE.println(fix ? F("YES") : F("no"));
    CONSOLE.println();

    if (inView == 0) {
        CONSOLE.println(F("  Nothing heard. Antenna is blind — check the"));
        CONSOLE.println(F("  patch antenna connector, or it needs real sky."));
    } else if (bestSnr < 25) {
        CONSOLE.println(F("  Birds are visible but far too weak to solve."));
        CONSOLE.println(F("  A window is usually not enough. Needs open sky."));
    } else if (bestSnr < 35) {
        CONSOLE.println(F("  Marginal. A fix may come but could take many"));
        CONSOLE.println(F("  minutes — cold start needs ~30 s of clean data"));
        CONSOLE.println(F("  per satellite to download the almanac."));
    } else {
        CONSOLE.println(F("  Signal is good. A fix should arrive shortly."));
    }
}

// ============================================================
// AS3935 EMI soak — what the environment looks like at current tuning.
// Disturber floods are the single most common AS3935 complaint, and they
// are a siting/tuning problem, not a firmware one.
// ============================================================
static void as3935Soak(uint32_t seconds) {
    banner("AS3935 noise soak");
    CONSOLE.print(F("  listening "));
    CONSOLE.print(seconds);
    CONSOLE.println(F(" s at current tuning"));

    as3935Mask(0x00, 0xC1, (AS3935_OUTDOOR_MODE ? AS3935_OUTDOOR_GAIN
                                                : AS3935_INDOOR_GAIN) << 1);
    as3935Mask(0x01, 0x8F, AS3935_NOISE_FLOOR << 4);
    as3935Mask(0x01, 0xF0, AS3935_WATCHDOG_THRESH & 0x0F);
    as3935Mask(0x02, 0xF0, AS3935_SPIKE_REJECT & 0x0F);
    as3935Read(0x03);                       // drain anything pending

    uint32_t noise = 0, dist = 0, strikes = 0;
    irqCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onLightningIRQ, RISING);

    uint32_t start = millis();
    uint32_t lastReport = start;
    while (millis() - start < seconds * 1000UL) {
        if (irqCount) {
            irqCount = 0;
            delay(3);                       // datasheet: 2 ms before INT read
            uint8_t src = as3935Read(0x03) & 0x0F;
            if (src == 0x01) noise++;
            else if (src == 0x04) dist++;
            else if (src == 0x08) {
                strikes++;
                CONSOLE.print(F("  LIGHTNING dist="));
                CONSOLE.print(as3935Read(0x07) & 0x3F);
                CONSOLE.println(F(" km"));
            }
        }
        if (millis() - lastReport >= 10000) {
            lastReport = millis();
            CONSOLE.print(F("  t="));
            CONSOLE.print((millis() - start) / 1000);
            CONSOLE.print(F("s  noise="));   CONSOLE.print(noise);
            CONSOLE.print(F(" disturber=")); CONSOLE.print(dist);
            CONSOLE.print(F(" strikes="));   CONSOLE.println(strikes);
        }
    }
    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));

    CONSOLE.println();
    CONSOLE.print(F("  TOTAL over "));  CONSOLE.print(seconds);
    CONSOLE.print(F(" s: noise="));     CONSOLE.print(noise);
    CONSOLE.print(F(" disturber="));    CONSOLE.print(dist);
    CONSOLE.print(F(" strikes="));      CONSOLE.println(strikes);

    float dpm = (dist * 60.0f) / seconds;
    CONSOLE.print(F("  disturbers/min: "));
    CONSOLE.println(dpm, 1);

    if (noise > seconds / 2) {
        CONSOLE.println(F("  NOISE FLOOR TOO LOW — raise AS3935_NOISE_FLOOR."));
    } else if (dpm > 10.0f) {
        CONSOLE.println(F("  Heavy disturbers. Raise AS3935_WATCHDOG_THRESH,"));
        CONSOLE.println(F("  and suspect the GPS or a switching supply first."));
    } else if (dpm > 2.0f) {
        CONSOLE.println(F("  Moderate disturbers — acceptable, but re-check"));
        CONSOLE.println(F("  in the final enclosure."));
    } else {
        CONSOLE.println(F("  Quiet. Current tuning looks good for this site."));
    }
}

// A soak at deliberately reckless sensitivity: indoor AFE gain, noise
// floor 0, watchdog 0, spike rejection 0. Indoors, surrounded by
// switching supplies and a GPS, this SHOULD produce a flood of
// disturbers. If it stays silent, the receive path is deaf and a clean
// result at normal tuning means nothing.
static void as3935SoakSensitive(uint32_t seconds) {
    banner("AS3935 max-sensitivity soak");
    CONSOLE.println(F("  indoor gain, noise floor 0, watchdog 0, spike 0"));
    CONSOLE.println(F("  expecting a FLOOD — silence here means deaf"));

    as3935Mask(0x00, 0xC1, AS3935_INDOOR_GAIN << 1);
    as3935Mask(0x01, 0x8F, 0 << 4);
    as3935Mask(0x01, 0xF0, 0);
    as3935Mask(0x02, 0xF0, 0);
    as3935Read(0x03);

    uint32_t noise = 0, dist = 0, strikes = 0;
    irqCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onLightningIRQ, RISING);

    uint32_t start = millis(), lastReport = start;
    while (millis() - start < seconds * 1000UL) {
        if (irqCount) {
            irqCount = 0;
            delay(3);
            uint8_t src = as3935Read(0x03) & 0x0F;
            if (src == 0x01) noise++;
            else if (src == 0x04) dist++;
            else if (src == 0x08) strikes++;
        }
        if (millis() - lastReport >= 15000) {
            lastReport = millis();
            CONSOLE.print(F("  t="));
            CONSOLE.print((millis() - start) / 1000);
            CONSOLE.print(F("s  noise="));   CONSOLE.print(noise);
            CONSOLE.print(F(" disturber=")); CONSOLE.println(dist);
        }
    }
    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));

    CONSOLE.println();
    CONSOLE.print(F("  TOTAL: noise="));  CONSOLE.print(noise);
    CONSOLE.print(F(" disturber="));      CONSOLE.print(dist);
    CONSOLE.print(F(" strikes="));        CONSOLE.println(strikes);

    if (noise + dist + strikes == 0) {
        CONSOLE.println(F("  DEAF. Nothing at maximum sensitivity indoors is"));
        CONSOLE.println(F("  not plausible. The analog front end is not"));
        CONSOLE.println(F("  hearing the antenna. Suspect the antenna solder"));
        CONSOLE.println(F("  joints or a damaged AS3935 — SPI works fine, so"));
        CONSOLE.println(F("  this is the RF side, not the digital side."));
    } else {
        CONSOLE.println(F("  Front end is alive. The quiet result at normal"));
        CONSOLE.println(F("  tuning is real, not a dead receiver."));
    }

    // Restore configured tuning.
    as3935Mask(0x00, 0xC1, (AS3935_OUTDOOR_MODE ? AS3935_OUTDOOR_GAIN
                                                : AS3935_INDOOR_GAIN) << 1);
    as3935Mask(0x01, 0x8F, AS3935_NOISE_FLOOR << 4);
    as3935Mask(0x01, 0xF0, AS3935_WATCHDOG_THRESH & 0x0F);
    as3935Mask(0x02, 0xF0, AS3935_SPIKE_REJECT & 0x0F);
    as3935Read(0x03);
}

// ============================================================
// Test 7 — PPS on PA0
//
// PA0 is ALSO the onboard KEY button on the WeAct Black Pill — the button
// shorts it to GND. Do not press KEY during this test or the edge count is
// meaningless. See the warning in config.h.
// ============================================================

static void testPPS() {
    banner("GPS PPS (PA0)");
    CONSOLE.println(F("  NOTE: PA0 is also the onboard KEY button — do not"));
    CONSOLE.println(F("        press it during this test."));

    pinMode(PIN_GPS_PPS, INPUT);
    ppsCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), onPPS, RISING);
    CONSOLE.println(F("  counting rising edges for 4 s"));
    delay(4000);
    detachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS));

    uint32_t n = ppsCount;
    CONSOLE.print(F("  edges: "));
    CONSOLE.println(n);

    char d[48];
    if (n == 0) {
        record("GPS PPS", WARN, "no pulses - normal until GPS has a fix");
    } else if (n >= 3 && n <= 5) {
        snprintf(d, sizeof(d), "%lu pulses in 4s - 1 Hz confirmed", n);
        record("GPS PPS", PASS, d);
    } else {
        snprintf(d, sizeof(d), "%lu edges in 4s - not 1 Hz, check bounce", n);
        record("GPS PPS", WARN, d);
    }
}

// ============================================================
// Test 8 — uplink UART on USART2
// ============================================================
static void testUplink() {
    banner("Uplink UART (USART2, TX PA2 / RX PA3)");

    SerialUplink.begin(UPLINK_BAUD);
    delay(20);
    while (SerialUplink.available()) SerialUplink.read();

    const char *probe = "WXNODE-UPLINK-TEST\r\n";
    SerialUplink.print(probe);
    SerialUplink.flush();

    // Only meaningful if PA2 and PA3 are jumpered together.
    uint32_t start = millis();
    uint8_t  got = 0;
    while (millis() - start < 200 && got < 18) {
        if (SerialUplink.available()) { SerialUplink.read(); got++; }
    }

    CONSOLE.print(F("  sent test string at "));
    CONSOLE.print(UPLINK_BAUD);
    CONSOLE.println(F(" baud"));
    CONSOLE.print(F("  loopback bytes returned: "));
    CONSOLE.println(got);

    if (got >= 18) {
        record("Uplink UART", PASS, "loopback jumper confirmed TX+RX");
    } else {
        record("Uplink UART", SKIP, "TX sent; jumper PA2-PA3 to verify");
    }
}

// ============================================================
// AS3935 antenna calibration — LCO frequency sweep
//
// Register 0x08 bit 7 puts the LCO on the IRQ pin, divided by 16. The
// antenna must land within 3.5% of 500 kHz, so we look for the tuning
// cap value that puts the divided output nearest 31250 Hz.
// ============================================================
static volatile uint32_t lcoEdges = 0;
static void onLCO() { lcoEdges++; }

static uint32_t measureLCO(uint8_t tuningCap) {
    as3935Mask(0x08, 0xF0, tuningCap & 0x0F);
    delay(3);
    as3935Mask(0x08, 0x7F, 0x80);           // DISP_LCO on
    delay(20);                              // let it settle

    lcoEdges = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onLCO, RISING);
    delay(100);
    detachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ));

    as3935Mask(0x08, 0x7F, 0x00);           // DISP_LCO off
    delay(3);

    return lcoEdges * 10 * AS3935_LCO_DIVIDER;   // 100 ms window -> Hz
}

static void sweepLCO() {
    banner("AS3935 antenna sweep (LCO)");
    CONSOLE.println(F("  target 500000 Hz +/- 3.5% (482500 - 517500)"));
    CONSOLE.println(F("  cap  pF    measured Hz    error"));

    uint8_t  bestCap = 0;
    uint32_t bestErr = 0xFFFFFFFF;
    uint32_t bestHz  = 0;

    for (uint8_t cap = 0; cap < 16; cap++) {
        uint32_t hz  = measureLCO(cap);
        uint32_t err = hz > AS3935_LCO_TARGET_HZ ? hz - AS3935_LCO_TARGET_HZ
                                                 : AS3935_LCO_TARGET_HZ - hz;
        CONSOLE.print(F("   "));
        if (cap < 10) CONSOLE.print(' ');
        CONSOLE.print(cap);
        CONSOLE.print(F("   "));
        CONSOLE.print(cap * 8);
        if (cap * 8 < 100) CONSOLE.print(' ');
        CONSOLE.print(F("     "));
        CONSOLE.print(hz);
        CONSOLE.print(F("       "));
        CONSOLE.print(hz == 0 ? 100.0f : (err * 100.0f) / AS3935_LCO_TARGET_HZ, 2);
        CONSOLE.print(F(" %"));
        if (err < bestErr && hz > 0) { bestErr = err; bestCap = cap; bestHz = hz; }
        CONSOLE.println();
    }

    CONSOLE.println();
    if (bestHz == 0) {
        CONSOLE.println(F("  no LCO output at all — IRQ pin not connected,"));
        CONSOLE.println(F("  or the antenna is not oscillating."));
        return;
    }

    float errPct = (bestErr * 100.0f) / AS3935_LCO_TARGET_HZ;
    CONSOLE.print(F("  best: cap="));
    CONSOLE.print(bestCap);
    CONSOLE.print(F(" ("));
    CONSOLE.print(bestCap * 8);
    CONSOLE.print(F(" pF)  "));
    CONSOLE.print(bestHz);
    CONSOLE.print(F(" Hz  err "));
    CONSOLE.print(errPct, 2);
    CONSOLE.println(F(" %"));

    if (errPct <= AS3935_LCO_TOLERANCE * 100.0f) {
        CONSOLE.print(F("  IN SPEC — set AS3935_TUNING_CAP to "));
        CONSOLE.print(bestCap);
        CONSOLE.println(F(" in config.h"));
    } else {
        CONSOLE.println(F("  OUT OF SPEC — antenna cannot be trimmed into range."));
        CONSOLE.println(F("  Suspect the antenna, or heavy EMI on the IRQ line."));
    }

    as3935Mask(0x08, 0xF0, AS3935_TUNING_CAP & 0x0F);
}

// ============================================================
// Jump to the STM32 system DFU bootloader on command.
//
// Saves a trip to the board for every reflash: no BOOT0 jumper, no reset
// button. The part re-enumerates as 0483:df11 and dfu-util can take it
// from there. Only in the test firmware — the flight build must never
// expose a way to brick itself remotely.
// ============================================================
static void jumpToBootloader() {
    CONSOLE.println(F("  entering DFU bootloader — port will disappear"));
    CONSOLE.println(F("  (if it does not come back, power cycle the board)"));
    CONSOLE.flush();
    delay(200);

    typedef void (*bootJump_t)(void);
    const uint32_t sysMemBase = 0x1FFF0000; // STM32F4 system memory

    // Force a USB disconnect before jumping. Without this the host keeps
    // the old CDC device open, ignores the bootloader's enumeration, and
    // the board simply vanishes from the bus — which is exactly what
    // happened the one time this was tried without it. Driving D+ (PA12)
    // low for ~80 ms makes the host tear the device down first.
    pinMode(PA12, OUTPUT);
    digitalWrite(PA12, LOW);
    delay(80);
    pinMode(PA12, INPUT);
    delay(20);

    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    __disable_irq();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    bootJump_t bootJump = (bootJump_t)(*((uint32_t *)(sysMemBase + 4)));
    __set_MSP(*(uint32_t *)sysMemBase);
    __enable_irq();
    bootJump();

    while (1) { }                           // unreachable
}

// ============================================================
// Summary
// ============================================================
static void printSummary() {
    CONSOLE.println();
    CONSOLE.println(F("============================================================"));
    CONSOLE.println(F(" SUMMARY"));
    CONSOLE.println(F("============================================================"));

    uint8_t fails = 0, warns = 0;
    for (uint8_t i = 0; i < rowCount; i++) {
        CONSOLE.print(' ');
        CONSOLE.print(resultText(rows[i].result));
        CONSOLE.print(F("  "));
        CONSOLE.print(rows[i].name);
        for (uint8_t s = strlen(rows[i].name); s < 16; s++) CONSOLE.print(' ');
        CONSOLE.println(rows[i].detail);
        if (rows[i].result == FAIL) fails++;
        if (rows[i].result == WARN) warns++;
    }

    CONSOLE.println(F("------------------------------------------------------------"));
    CONSOLE.print(F(" "));
    CONSOLE.print(rowCount);
    CONSOLE.print(F(" checks, "));
    CONSOLE.print(fails);
    CONSOLE.print(F(" failed, "));
    CONSOLE.print(warns);
    CONSOLE.println(F(" warnings"));
    CONSOLE.println();
    CONSOLE.println(F(" Commands:  r = rerun    i = I2C scan    l = AS3935 regs"));
    CONSOLE.println(F("            g = GPS raw  c = LCO sweep   t = DS3231 time"));
    CONSOLE.println(F("            w = seed RTC + clear OSF (backup cell test)"));
    CONSOLE.println(F("            s = set RTC from host: s2026-08-08 15:52:00"));
    CONSOLE.println(F("            p = force PPS 1Hz (no fix needed)  v = sky view"));
    CONSOLE.println(F("            n = AS3935 noise soak (120 s)"));
    CONSOLE.println(F("            m = AS3935 max-sensitivity soak (60 s)"));
    CONSOLE.println(F("            b = reboot into DFU bootloader (no BOOT0)"));
    CONSOLE.println(F("============================================================"));
}

static void runAll() {
    rowCount = 0;
    testBME680();
    testAS3935();
    testSharedBus();
    testIRQLine();
    testI2C();
    testGPS();
    testPPS();
    testUplink();
    printSummary();
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);        // active low, off

    CONSOLE.begin(115200);
    uint32_t t = millis();
    while (!CONSOLE && millis() - t < 10000) {
        digitalWrite(PIN_LED, (millis() / 100) % 2 ? HIGH : LOW);
    }
    digitalWrite(PIN_LED, HIGH);
    delay(200);

    CONSOLE.println();
    CONSOLE.println(F("============================================================"));
    CONSOLE.println(F(" Weather node board test — " NODE_ID));
    CONSOLE.println(F(" STM32F411CEU6 @ 100 MHz   built " __DATE__ " " __TIME__));
    CONSOLE.println(F("============================================================"));

    // CS lines idle high before the bus comes up, so neither device sees
    // the other's traffic.
    pinMode(PIN_BME680_CS, OUTPUT);
    pinMode(PIN_AS3935_CS, OUTPUT);
    digitalWrite(PIN_BME680_CS, HIGH);
    digitalWrite(PIN_AS3935_CS, HIGH);
    delay(10);
    SPI.begin();

    runAll();
}

void loop() {
    if (CONSOLE.available()) {
        char c = CONSOLE.read();
        switch (c) {
            case 'r': runAll(); break;
            case 'i': rowCount = 0; testI2C(); break;
            case 'g': rowCount = 0; testGPS(); break;
            case 't': {
                banner("DS3231 time");
                rowCount = 0;
                testI2C();
                break;
            }
            case 'c': sweepLCO(); break;
            case 'b': jumpToBootloader(); break;
            case 'w': seedRTC(); break;
            case 'p': testPPSForced(); break;
            case 'v': gpsSkyView(); break;
            case 'n': as3935Soak(120); break;
            case 'm': as3935SoakSensitive(60); break;
            case 's': banner("DS3231 set from host"); setRTCFromHost(); break;
            case 'l': {
                banner("AS3935 registers");
                for (uint8_t r = 0x00; r <= 0x08; r++) {
                    CONSOLE.print(F("  0x0")); CONSOLE.print(r, HEX);
                    CONSOLE.print(F(" = 0x")); printHex8(as3935Read(r));
                    CONSOLE.println();
                }
                CONSOLE.print(F("  0x3A = 0x")); printHex8(as3935Read(0x3A)); CONSOLE.println();
                CONSOLE.print(F("  0x3B = 0x")); printHex8(as3935Read(0x3B)); CONSOLE.println();
                break;
            }
            default: break;
        }
    }

    // Slow heartbeat so it is obvious the board has not hung.
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        digitalWrite(PIN_LED, LOW);
        delay(20);
        digitalWrite(PIN_LED, HIGH);
    }
}
