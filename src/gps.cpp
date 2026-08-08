// ============================================================
// gps.cpp — minimal NMEA + UBX, enough to discipline the RTC
// ============================================================

#include "gps.h"
#include "config.h"

static char     line[100];
static uint8_t  lineLen  = 0;
static bool     haveFix  = false;
static bool     timeOk   = false;
static uint8_t  sats     = 0;
static bool     asleep   = false;

static uint16_t gYear;
static uint8_t  gMon, gDay, gHour, gMin, gSec;

static volatile bool ppsFlag = false;
static void onPps() { ppsFlag = true; }

// ---- UBX ---------------------------------------------------
static void sendUBX(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t head[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t a = 0, b = 0;
    for (uint8_t i = 2; i < 6; i++) { a += head[i]; b += a; }
    for (uint16_t i = 0; i < len; i++) { a += payload[i]; b += a; }

    SerialGPS.write(head, 6);
    if (len) SerialGPS.write(payload, len);
    SerialGPS.write(a);
    SerialGPS.write(b);
    SerialGPS.flush();
}

void gpsConfigTimepulse() {
    uint8_t p[32] = {0};
    p[0]  = 0;                      // tpIdx
    p[8]  = 1;                      // freqPeriod      = 1 Hz when unlocked
    p[12] = 1;                      // freqPeriodLock  = 1 Hz when locked

    uint32_t plen = 100000;         // 100 ms high time, both states
    memcpy(&p[16], &plen, 4);
    memcpy(&p[20], &plen, 4);

    // active | lockGnssFreq | lockedOtherSet | isFreq | isLength |
    // alignToTow | polarity(rising)
    uint32_t flags = 0x7F;
    memcpy(&p[28], &flags, 4);

    sendUBX(0x06, 0x31, p, 32);
}

void gpsBegin() {
    // Pin the peripheral explicitly. PB6/PB7 are also valid USART1 pins
    // on this part and they are the I2C bus — never let pinmap ordering
    // decide which the GPS gets.
    SerialGPS.setRx(PA10);
    SerialGPS.setTx(PA9);
    SerialGPS.begin(GPS_BAUD);

    pinMode(PIN_GPS_PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), onPps, RISING);

    asleep = false;
    delay(100);
    gpsConfigTimepulse();
}

// ---- NMEA --------------------------------------------------
static uint8_t splitFields(char *s, char **f, uint8_t maxF) {
    uint8_t n = 0;
    f[n++] = s;
    for (char *p = s; *p && n < maxF; p++)
        if (*p == ',') { *p = '\0'; f[n++] = p + 1; }
    return n;
}

static void parseRMC(char **f, uint8_t n) {
    // $xxRMC,hhmmss.ss,status,lat,NS,lon,EW,spd,cog,ddmmyy,...
    if (n < 10) return;
    if (!f[1] || strlen(f[1]) < 6) return;
    if (!f[9] || strlen(f[9]) < 6) return;

    gHour = (f[1][0] - '0') * 10 + (f[1][1] - '0');
    gMin  = (f[1][2] - '0') * 10 + (f[1][3] - '0');
    gSec  = (f[1][4] - '0') * 10 + (f[1][5] - '0');

    gDay  = (f[9][0] - '0') * 10 + (f[9][1] - '0');
    gMon  = (f[9][2] - '0') * 10 + (f[9][3] - '0');
    gYear = (uint16_t)(2000 + (f[9][4] - '0') * 10 + (f[9][5] - '0'));

    haveFix = (f[2] && f[2][0] == 'A');

    // A u-blox reports valid UTC once it has decoded a satellite's time,
    // before it can solve position. That is enough to discipline a clock
    // to the second, which is all this node needs — so time validity is
    // tracked separately from fix.
    timeOk = (gMon >= 1 && gMon <= 12 && gDay >= 1 && gDay <= 31 &&
              gHour <= 23 && gMin <= 59 && gSec <= 59 && gYear >= 2024);
}

static void parseGGA(char **f, uint8_t n) {
    if (n < 8) return;
    if (f[6] && f[6][0] != ',' && f[6][0] != '\0' && f[6][0] != '0') haveFix = true;
    if (f[7]) sats = (uint8_t)atoi(f[7]);
}

void gpsPoll() {
    if (asleep) return;

    while (SerialGPS.available()) {
        char c = SerialGPS.read();
        if (c == '\n') {
            line[lineLen] = '\0';
            if (lineLen > 6 && line[0] == '$') {
                char *f[24];
                char  work[100];
                strncpy(work, line, sizeof(work) - 1);
                work[sizeof(work) - 1] = '\0';
                uint8_t n = splitFields(work, f, 24);
                if      (strstr(f[0], "RMC")) parseRMC(f, n);
                else if (strstr(f[0], "GGA")) parseGGA(f, n);
            }
            lineLen = 0;
        } else if (c != '\r' && lineLen < sizeof(line) - 1) {
            line[lineLen++] = c;
        }
    }
}

bool     gpsHasFix()      { return haveFix; }
bool     gpsTimeValid()   { return timeOk; }
uint8_t  gpsSatellites()  { return sats; }
bool     gpsIsAsleep()    { return asleep; }

uint32_t gpsUnix() {
    if (!timeOk) return 0;
    RtcTime t = { gYear, gMon, gDay, gHour, gMin, gSec };
    return toUnix(t);
}

bool gpsPpsTick() {
    if (!ppsFlag) return false;
    ppsFlag = false;
    return true;
}

void gpsSleep() {
    if (asleep) return;

    // UBX-RXM-PMREQ: duration 0 means sleep until woken by activity on
    // the port. backupMode bit set.
    uint8_t p[8] = {0};
    uint32_t flags = 0x02;
    memcpy(&p[4], &flags, 4);
    sendUBX(0x02, 0x41, p, 8);
    delay(50);

    // TX to high-Z BEFORE cutting power, or the module is back-powered
    // through its ESD diodes and never actually goes down.
    SerialGPS.end();
    pinMode(PA9,  INPUT);
    pinMode(PA10, INPUT);

#ifdef GPS_POWER_GATE_FITTED
    pinMode(PIN_GPS_PWR, OUTPUT);
    digitalWrite(PIN_GPS_PWR, LOW);
#endif

    asleep  = true;
    haveFix = false;
    timeOk  = false;
    sats    = 0;
}

void gpsWake() {
    if (!asleep) return;

#ifdef GPS_POWER_GATE_FITTED
    pinMode(PIN_GPS_PWR, OUTPUT);
    digitalWrite(PIN_GPS_PWR, HIGH);
    delay(200);                     // let the regulator settle
#endif

    SerialGPS.setRx(PA10);
    SerialGPS.setTx(PA9);
    SerialGPS.begin(GPS_BAUD);
    asleep  = false;
    lineLen = 0;

    // Any byte wakes a PMREQ-sleeping module.
    SerialGPS.write((uint8_t)0xFF);
    delay(200);

    // Timepulse config lives in RAM and is lost on every power cycle.
    gpsConfigTimepulse();
}
