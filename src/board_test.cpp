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
    as3935Write(0x3D, 0x96);            // calibrate RC oscillators
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

    irqCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onLightningIRQ, RISING);
    delay(1000);
    uint32_t n = irqCount;

    CONSOLE.print(F("  interrupts in 1 s = "));
    CONSOLE.println(n);

    char d[48];
    if (idle == HIGH) {
        record("AS3935 IRQ", WARN, "line stuck high - read INT reg to clear");
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
// Test 6 — GPS on USART1
// ============================================================
static void testGPS() {
    banner("GPS GT-U7 (USART1, RX PA10 / TX PA9)");

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
        record("GPS", FAIL, "silent - check 5V rail and TX->PA10");
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

// ============================================================
// Test 7 — PPS on PA0
// ============================================================
static volatile uint32_t ppsCount = 0;
static void onPPS() { ppsCount++; }

static void testPPS() {
    banner("GPS PPS (PA0)");

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
