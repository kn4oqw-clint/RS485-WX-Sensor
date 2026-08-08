// ============================================================
// frame.cpp — uplink frame serialisation
// ============================================================

#include "frame.h"
#include "crc16.h"

// Little-endian writers. Explicit rather than memcpy of a struct: the
// wire layout must not depend on this compiler's padding or alignment
// choices, because the decoder is a different program on a different
// architecture.
static inline void put8(uint8_t *&p, uint8_t v)  { *p++ = v; }
static inline void put16(uint8_t *&p, uint16_t v){ *p++ = v & 0xFF; *p++ = v >> 8; }
static inline void put32(uint8_t *&p, uint32_t v){
    *p++ = v & 0xFF; *p++ = (v >> 8) & 0xFF; *p++ = (v >> 16) & 0xFF; *p++ = (v >> 24) & 0xFF;
}

static size_t finish(uint8_t *buf, uint8_t type, uint8_t payloadLen) {
    buf[0] = FRAME_SYNC0;
    buf[1] = FRAME_SYNC1;
    buf[2] = FRAME_VERSION;
    buf[3] = type;
    buf[4] = payloadLen;

    // CRC covers version..payload, i.e. everything but the sync bytes.
    uint16_t crc = crc16_ccitt(&buf[2], (size_t)payloadLen + 3);
    buf[FRAME_HEADER_LEN + payloadLen]     = crc & 0xFF;
    buf[FRAME_HEADER_LEN + payloadLen + 1] = crc >> 8;

    return FRAME_HEADER_LEN + payloadLen + FRAME_CRC_LEN;
}

size_t frameBuildWeather(uint8_t *buf, size_t bufLen, const WeatherPayload &p) {
    if (bufLen < FRAME_HEADER_LEN + WX_PAYLOAD_LEN + FRAME_CRC_LEN) return 0;

    uint8_t *w = buf + FRAME_HEADER_LEN;

    put32(w, p.unixTime);
    put8(w,  p.node);
    put8(w,  p.flags);
    put8(w,  p.samples);
    put8(w,  0);                    // reserved

    put16(w, (uint16_t)p.tempNow);
    put16(w, (uint16_t)p.tempMin);
    put16(w, (uint16_t)p.tempMax);
    put16(w, (uint16_t)p.tempTrend);

    put16(w, p.rhNow);
    put16(w, p.rhMin);
    put16(w, p.rhMax);
    put16(w, (uint16_t)p.rhTrend);

    put32(w, p.pressNow);
    put32(w, p.pressMin);
    put32(w, p.pressMax);
    put16(w, (uint16_t)p.pressTrend);

    put16(w, p.strikes);
    put8(w,  p.nearestKm);
    put8(w,  0);                    // reserved2
    put16(w, p.disturbers);
    put16(w, p.vddMv);
    put16(w, p.uptimeMin);

    // Catches a payload edit that forgets to update WX_PAYLOAD_LEN.
    size_t written = (size_t)(w - (buf + FRAME_HEADER_LEN));
    if (written != WX_PAYLOAD_LEN) return 0;

    return finish(buf, FRAME_TYPE_WEATHER, WX_PAYLOAD_LEN);
}

size_t frameBuildBoot(uint8_t *buf, size_t bufLen, uint8_t node,
                      uint32_t unixTime, uint8_t flags, uint16_t vddMv) {
    const uint8_t len = 8;
    if (bufLen < FRAME_HEADER_LEN + len + FRAME_CRC_LEN) return 0;

    uint8_t *w = buf + FRAME_HEADER_LEN;
    put32(w, unixTime);
    put8(w,  node);
    put8(w,  flags);
    put16(w, vddMv);

    return finish(buf, FRAME_TYPE_BOOT, len);
}

void frameDump(Print &out, const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (buf[i] < 0x10) out.print('0');
        out.print(buf[i], HEX);
        out.print((i % 16 == 15) ? '\n' : ' ');
    }
    if (len % 16) out.println();
}
