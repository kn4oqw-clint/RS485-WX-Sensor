// ============================================================
// AS3935 SPI Driver Implementation
// ============================================================

#include "as3935.h"

const SPISettings AS3935::_spiSettings(2000000, MSBFIRST, SPI_MODE1);

AS3935::AS3935(uint8_t csPin, SPIClass &spiBus)
    : _cs(csPin), _spi(spiBus) {}

bool AS3935::begin() {
    pinMode(_cs, OUTPUT);
    _deselect();

    // Give the chip time to settle after power-up
    delay(2);

    reset();
    delay(2);

    calibrate();

    // Verify chip is responding — REG 0x00 AFE gain default is 0x24
    uint8_t val = readRegister(AS3935_REG_AFE_GAIN);
    return (val != 0xFF && val != 0x00);
}

void AS3935::reset() {
    writeRegister(AS3935_REG_RESET, 0x96);
    delay(2);
}

void AS3935::calibrate() {
    // Trigger RC oscillator calibration
    writeRegister(AS3935_REG_CALIB, 0x96);
    delay(2);
}

void AS3935::setIndoor() {
    maskRegisterBits(AS3935_REG_AFE_GAIN, 0xC1, AS3935_INDOOR << 1);
}

void AS3935::setOutdoor() {
    maskRegisterBits(AS3935_REG_AFE_GAIN, 0xC1, AS3935_OUTDOOR << 1);
}

void AS3935::setNoiseFloor(uint8_t level) {
    // Bits [6:4] of REG 0x01
    maskRegisterBits(AS3935_REG_THRESHOLD, 0x8F, (level & 0x07) << 4);
}

void AS3935::setWatchdogThreshold(uint8_t t) {
    // Bits [3:0] of REG 0x01
    maskRegisterBits(AS3935_REG_THRESHOLD, 0xF0, t & 0x0F);
}

void AS3935::setSpikeRejection(uint8_t r) {
    // Bits [3:0] of REG 0x02
    maskRegisterBits(AS3935_REG_LIGHTNING, 0xF0, r & 0x0F);
}

void AS3935::setMinLightningEvents(uint8_t n) {
    // Bits [5:4] of REG 0x02
    // Valid encoded values: 0=1, 1=5, 2=9, 3=16
    uint8_t encoded;
    if      (n <= 1)  encoded = 0;
    else if (n <= 5)  encoded = 1;
    else if (n <= 9)  encoded = 2;
    else              encoded = 3;
    maskRegisterBits(AS3935_REG_LIGHTNING, 0xCF, encoded << 4);
}

uint8_t AS3935::getInterruptSource() {
    // Datasheet: wait 2ms after IRQ before reading INT register
    delay(2);
    return readRegister(AS3935_REG_INT_MASK_ANT) & 0x0F;
}

uint8_t AS3935::getDistanceEstimate() {
    return readRegister(AS3935_REG_DISTANCE) & 0x3F;
}

uint32_t AS3935::getLightningEnergy() {
    uint32_t lsb  = readRegister(AS3935_REG_ENERGY_LSB);
    uint32_t msb  = readRegister(AS3935_REG_ENERGY_MSB);
    uint32_t mmsb = readRegister(AS3935_REG_ENERGY_MMSB) & 0x1F;
    return (mmsb << 16) | (msb << 8) | lsb;
}

// ---- Low-level SPI -------------------------------------------------------------

uint8_t AS3935::readRegister(uint8_t reg) {
    // AS3935 SPI read: bit7=1 (read), bit6=0, bits[5:0]=address
    uint8_t result;
    _select();
    _spi.beginTransaction(_spiSettings);
    _spi.transfer((reg & 0x3F) | 0x40);  // read flag
    result = _spi.transfer(0x00);
    _spi.endTransaction();
    _deselect();
    return result;
}

void AS3935::writeRegister(uint8_t reg, uint8_t value) {
    // AS3935 SPI write: bit7=0, bit6=0, bits[5:0]=address
    _select();
    _spi.beginTransaction(_spiSettings);
    _spi.transfer(reg & 0x3F);
    _spi.transfer(value);
    _spi.endTransaction();
    _deselect();
}

void AS3935::maskRegisterBits(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t current = readRegister(reg);
    writeRegister(reg, (current & mask) | value);
}

void AS3935::_select() {
    digitalWrite(_cs, LOW);
    delayMicroseconds(1);
}

void AS3935::_deselect() {
    delayMicroseconds(1);
    digitalWrite(_cs, HIGH);
}
