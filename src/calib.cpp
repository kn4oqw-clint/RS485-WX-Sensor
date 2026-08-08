// ============================================================
// calib.cpp — emulated-EEPROM persistence
// ============================================================

#include <EEPROM.h>
#include <stddef.h>
#include "calib.h"
#include "crc16.h"
#include "config.h"

#define CALIB_ADDR 0

// offsetof, NOT sizeof-sizeof(crc). The struct has trailing padding
// (sizeof is 24, crc sits at offset 20), so the naive expression covers
// the crc field itself: the value computed at save time differs from the
// one computed at load time, and verification can never succeed.
static uint16_t calcCrc(const NodeCalib &c) {
    return crc16_ccitt((const uint8_t *)&c, offsetof(NodeCalib, crc));
}

bool calibLoad(NodeCalib &out) {
    // eeprom_buffered_read_byte() indexes a RAM shadow buffer, it does
    // not touch flash. Without this fill the buffer is zeroed .bss on a
    // fresh boot and every load returns zeros.
    eeprom_buffer_fill();

    uint8_t *p = (uint8_t *)&out;
    for (size_t i = 0; i < sizeof(NodeCalib); i++)
        p[i] = eeprom_buffered_read_byte(CALIB_ADDR + i);

    // A blank sector reads as 0xFF, which fails the magic check.
    if (out.magic != CALIB_MAGIC)     return false;
    if (out.version != CALIB_VERSION) return false;
    if (out.crc != calcCrc(out))      return false;

    // Range-check anything that will be written to a device register. A
    // corrupt value here would be programmed straight into the AS3935.
    if (out.tuningCap   > 15) return false;
    if (out.noiseFloor  >  7) return false;
    if (out.watchdog    > 15) return false;
    if (out.spikeReject > 15) return false;
    return true;
}

bool calibSave(NodeCalib &c) {
    c.magic   = CALIB_MAGIC;
    c.version = CALIB_VERSION;
    c.crc     = calcCrc(c);

    // Read-modify-write the whole emulated page, then flush ONCE. See
    // the storage warning in calib.h — a per-byte write would erase the
    // sector once per byte.
    eeprom_buffer_fill();
    const uint8_t *p = (const uint8_t *)&c;
    for (size_t i = 0; i < sizeof(NodeCalib); i++)
        eeprom_buffered_write_byte(CALIB_ADDR + i, p[i]);
    eeprom_buffer_flush();

    // Re-read through a fresh fill so this verifies what actually landed
    // in flash, not what we just put in the RAM buffer.
    NodeCalib check;
    return calibLoad(check) && check.crc == c.crc &&
           check.tuningCap == c.tuningCap && check.watchdog == c.watchdog;
}

void calibInvalidate() {
    eeprom_buffer_fill();
    for (size_t i = 0; i < sizeof(NodeCalib); i++)
        eeprom_buffered_write_byte(CALIB_ADDR + i, 0xFF);
    eeprom_buffer_flush();
}

bool calibShouldRefresh(const NodeCalib &c, int8_t nowTempC) {
    int delta = (int)nowTempC - (int)c.tempC;
    if (delta < 0) delta = -delta;
    return delta >= CALIB_TEMP_DELTA_C;
}
