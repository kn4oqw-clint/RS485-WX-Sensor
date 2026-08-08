// ============================================================
// rtc.cpp — DS3231 driver
// ============================================================

#include <Wire.h>
#include "rtc.h"
#include "config.h"

static inline uint8_t bcd2dec(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static inline uint8_t dec2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static bool readRegs(uint8_t start, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(start);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)ADDR_DS3231, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool rtcBegin() {
    Wire.setSCL(PIN_I2C_SCL);
    Wire.setSDA(PIN_I2C_SDA);
    Wire.begin();
    Wire.setClock(I2C_HZ);

    Wire.beginTransmission(ADDR_DS3231);
    return Wire.endTransmission() == 0;
}

bool rtcRead(RtcTime &out) {
    uint8_t b[7];
    if (!readRegs(0x00, b, 7)) return false;

    out.second = bcd2dec(b[0] & 0x7F);
    out.minute = bcd2dec(b[1] & 0x7F);
    out.hour   = bcd2dec(b[2] & 0x3F);          // forced 24-hour on write
    out.day    = bcd2dec(b[4] & 0x3F);
    out.month  = bcd2dec(b[5] & 0x1F);          // bit7 is the century flag
    out.year   = (uint16_t)(2000 + bcd2dec(b[6]));

    // A DS3231 with a dead cell can return values that decode to
    // nonsense. Reject rather than propagate them into timestamps.
    if (out.month < 1 || out.month > 12) return false;
    if (out.day   < 1 || out.day   > 31) return false;
    if (out.hour  > 23 || out.minute > 59 || out.second > 59) return false;
    return true;
}

bool rtcSet(const RtcTime &t) {
    Wire.beginTransmission(ADDR_DS3231);
    Wire.write((uint8_t)0x00);
    Wire.write(dec2bcd(t.second));
    Wire.write(dec2bcd(t.minute));
    Wire.write(dec2bcd(t.hour));                // bit6 clear -> 24 hour
    Wire.write((uint8_t)1);                     // day of week, unused
    Wire.write(dec2bcd(t.day));
    Wire.write(dec2bcd(t.month));               // century bit clear
    Wire.write(dec2bcd((uint8_t)(t.year % 100)));
    if (Wire.endTransmission() != 0) return false;

    // Setting the time means we now believe it, so clear OSF.
    return rtcClearOscFlag();
}

bool rtcSetUnix(uint32_t unix) {
    RtcTime t;
    fromUnix(unix, t);
    return rtcSet(t);
}

bool rtcOscStopped() {
    uint8_t s;
    if (!readRegs(0x0F, &s, 1)) return true;    // unreadable -> assume bad
    return (s & 0x80) != 0;
}

bool rtcClearOscFlag() {
    uint8_t s;
    if (!readRegs(0x0F, &s, 1)) return false;
    return writeReg(0x0F, (uint8_t)(s & 0x7F));
}

bool rtcDieTemp(float &out) {
    uint8_t b[2];
    if (!readRegs(0x11, b, 2)) return false;
    out = (int8_t)b[0] + ((b[1] >> 6) * 0.25f);
    return true;
}

uint32_t rtcUnix() {
    RtcTime t;
    if (!rtcRead(t)) return 0;
    return toUnix(t);
}

// ---- Calendar conversion -----------------------------------
// Days-from-civil, per Howard Hinnant's algorithm. Valid well beyond any
// plausible lifetime of this node and branch-free apart from the leap
// adjustment. Local implementation because avr/newlib time.h behaviour
// varies across cores and this must be predictable.
static int32_t daysFromCivil(int32_t y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}

static void civilFromDays(int32_t z, int32_t &y, unsigned &m, unsigned &d) {
    z += 719468;
    const int32_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int32_t yr = (int32_t)yoe + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp + (mp < 10 ? 3 : -9);
    y = yr + (m <= 2);
}

uint32_t toUnix(const RtcTime &t) {
    int32_t days = daysFromCivil((int32_t)t.year, t.month, t.day);
    return (uint32_t)days * 86400UL + t.hour * 3600UL + t.minute * 60UL + t.second;
}

void fromUnix(uint32_t unix, RtcTime &out) {
    int32_t  days = (int32_t)(unix / 86400UL);
    uint32_t rem  = unix % 86400UL;

    int32_t  y;
    unsigned m, d;
    civilFromDays(days, y, m, d);

    out.year   = (uint16_t)y;
    out.month  = (uint8_t)m;
    out.day    = (uint8_t)d;
    out.hour   = (uint8_t)(rem / 3600);
    out.minute = (uint8_t)((rem % 3600) / 60);
    out.second = (uint8_t)(rem % 60);
}
