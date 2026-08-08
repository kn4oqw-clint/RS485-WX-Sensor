#pragma once
// ============================================================
// Uplink frame format — STM32 node -> RAK4631 hub, one way.
//
// There is no return path and no ACK: the hub cannot ask for a
// retransmit, so every frame must stand alone and be verifiable on its
// own. Hence a length prefix and a CRC over everything that matters.
//
//   off  size  field
//   ---  ----  -----------------------------------------------
//     0     2  sync, 0xAA 0x55
//     2     1  version (FRAME_VERSION)
//     3     1  type
//     4     1  payload length N (0..255)
//     5     N  payload
//   5+N     2  CRC-16/CCITT-FALSE over bytes [2 .. 4+N],
//              little-endian. Covers version, type, length and
//              payload — everything except the sync bytes.
//
// Framing notes:
//   - Length-prefixed rather than escaped/delimited. A byte-stuffing
//     scheme would let the receiver resync on a delimiter, but it makes
//     frame size data-dependent, which is worse on a slow link.
//   - The sync bytes can legitimately appear inside a payload. The
//     receiver must treat a sync match as a *candidate* only, and
//     confirm with the CRC before accepting. That is why the CRC covers
//     the length byte: a false sync gives a garbage length, which
//     produces a garbage CRC, and the receiver moves on.
//   - Little-endian throughout, matching both Cortex-M and nRF52.
//
// All physical quantities are fixed-point integers, not floats. Floats
// would need matching representations on both ends for no benefit at
// this precision, and the scaling is documented per field below.
// ============================================================

#include <Arduino.h>

#define FRAME_SYNC0        0xAA
#define FRAME_SYNC1        0x55
#define FRAME_VERSION      0x01

#define FRAME_TYPE_WEATHER 0x01
#define FRAME_TYPE_BOOT    0x02

#define FRAME_HEADER_LEN   5
#define FRAME_CRC_LEN      2
#define FRAME_MAX_PAYLOAD  200
#define FRAME_MAX_LEN      (FRAME_HEADER_LEN + FRAME_MAX_PAYLOAD + FRAME_CRC_LEN)

// ---- Status flags (payload byte 5) -------------------------
#define FLAG_RTC_VALID     0x01   // DS3231 OSF clear; timestamp trustworthy
#define FLAG_GPS_FIX       0x02   // receiver had a fix at report time
#define FLAG_GPS_SYNCED    0x04   // RTC disciplined from GPS this session
#define FLAG_BME_OK        0x08   // last BME680 read succeeded
#define FLAG_AS3935_OK     0x10   // lightning sensor responding
#define FLAG_IAQ_VALID     0x20   // gas channel usable (0 on this board)
#define FLAG_STORM_ACTIVE  0x40   // strike seen within LIGHTNING_HOLD_MS

// ============================================================
// Weather payload, 48 bytes. Offsets are explicit so the decoder on
// the hub side can be written straight from this table.
//
//   off  size  type    field              scaling
//   ---  ----  ------  -----------------  --------------------------
//     0     4  u32     unix_time          seconds, UTC
//     4     1  u8      node                node number
//     5     1  u8      flags              see FLAG_* above
//     6     1  u8      samples            readings in this window
//     7     1  u8      reserved           0
//     8     2  i16     temp_now           0.01 C
//    10     2  i16     temp_min           0.01 C
//    12     2  i16     temp_max           0.01 C
//    14     2  i16     temp_trend         0.01 C per hour
//    16     2  u16     rh_now             0.01 %
//    18     2  u16     rh_min             0.01 %
//    20     2  u16     rh_max             0.01 %
//    22     2  i16     rh_trend           0.01 % per hour
//    24     4  u32     press_now          0.01 hPa
//    28     4  u32     press_min          0.01 hPa
//    32     4  u32     press_max          0.01 hPa
//    36     2  i16     press_trend        0.01 hPa per hour  <-- the
//                                          meteorologically useful one
//    38     2  u16     strikes            count this window
//    40     1  u8      nearest_km         0 = none
//    41     1  u8      reserved2          0
//    42     2  u16     disturbers         count this window
//    44     2  u16     vdd_mv             supply millivolts
//    46     2  u16     uptime_min         minutes since boot
// ============================================================
#define WX_PAYLOAD_LEN 48

struct WeatherPayload {
    uint32_t unixTime;
    uint8_t  node;
    uint8_t  flags;
    uint8_t  samples;
    int16_t  tempNow, tempMin, tempMax, tempTrend;
    uint16_t rhNow, rhMin, rhMax;
    int16_t  rhTrend;
    uint32_t pressNow, pressMin, pressMax;
    int16_t  pressTrend;
    uint16_t strikes;
    uint8_t  nearestKm;
    uint16_t disturbers;
    uint16_t vddMv;
    uint16_t uptimeMin;
};

// Serialises into buf and returns total frame length, or 0 on overflow.
size_t frameBuildWeather(uint8_t *buf, size_t bufLen, const WeatherPayload &p);

// Boot frame: lets the hub distinguish a node restart from a comms gap.
size_t frameBuildBoot(uint8_t *buf, size_t bufLen, uint8_t node,
                      uint32_t unixTime, uint8_t flags, uint16_t vddMv);

// Hex dump to a Print, for eyeballing frames on the console before the
// hub exists.
void frameDump(Print &out, const uint8_t *buf, size_t len);
