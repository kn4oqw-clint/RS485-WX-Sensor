#pragma once
// ============================================================
// CRC-16/CCITT-FALSE — poly 0x1021, init 0xFFFF, no reflection,
// no final XOR.
//
// Chosen because it is the variant every embedded toolchain and
// online calculator agrees on, which matters when the receiving end
// is written separately (RAK4631 / MeshCore) and has to match this
// byte for byte. Check value for "123456789" is 0x29B1.
// ============================================================

#include <Arduino.h>

static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}
