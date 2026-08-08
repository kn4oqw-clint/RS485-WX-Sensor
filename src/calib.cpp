// ============================================================
// calib.cpp — emulated-EEPROM persistence
// ============================================================

#include <EEPROM.h>
#include "calib.h"
#include "crc16.h"
#include "config.h"

#define CALIB_ADDR 0

static uint16_t calcCrc(const NodeCalib &c) {
    // Everything except the trailing crc field.
    return crc16_ccitt((const uint8_t *)&c, sizeof(NodeCalib) - sizeof(uint16_t));
}

bool calibLoad(NodeCalib &out) {
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

    NodeCalib check;
    return calibLoad(check) && check.crc == c.crc;
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
