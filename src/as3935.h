#pragma once
// ============================================================
// AS3935 SPI Driver
// Target: STM32F411CEU6 (Black Pill) via STM32duino
//
// Pinout:
//   SCK  -> PA5  (SPI1)
//   MISO -> PA6  (SPI1)
//   MOSI -> PA7  (SPI1)
//   CS   -> PB0  (PA3 confirmed bad joint)
//   IRQ  -> PA8  (EXTI8, rising edge)
//
// Static wiring on AMS eval board:
//   SI   -> GND  (selects SPI mode on THIS board — opposite of datasheet)
//   EN-V -> 3.3V
//   A0   -> GND
//   A1   -> GND
// ============================================================

#include <Arduino.h>
#include <SPI.h>

#define AS3935_REG_AFE_GAIN     0x00
#define AS3935_REG_THRESHOLD    0x01
#define AS3935_REG_LIGHTNING    0x02
#define AS3935_REG_INT_MASK_ANT 0x03
#define AS3935_REG_ENERGY_LSB   0x04
#define AS3935_REG_ENERGY_MSB   0x05
#define AS3935_REG_ENERGY_MMSB  0x06
#define AS3935_REG_DISTANCE     0x07
#define AS3935_REG_DISP_LCO     0x08
#define AS3935_REG_CALIB_TRCO   0x3A
#define AS3935_REG_CALIB_SRCO   0x3B
#define AS3935_REG_RESET        0x3C
#define AS3935_REG_CALIB        0x3D

#define AS3935_INT_NOISE        0x01
#define AS3935_INT_DISTURBER    0x04
#define AS3935_INT_LIGHTNING    0x08

#define AS3935_INDOOR           0x12
#define AS3935_OUTDOOR          0x0E

class AS3935 {
public:
    AS3935(uint8_t csPin, SPIClass &spiBus = SPI);

    bool     begin();
    void     reset();
    void     calibrate();

    void     setIndoor();
    void     setOutdoor();
    void     setNoiseFloor(uint8_t level);
    void     setWatchdogThreshold(uint8_t t);
    void     setSpikeRejection(uint8_t r);
    void     setMinLightningEvents(uint8_t n);

    uint8_t  getInterruptSource();
    uint8_t  getDistanceEstimate();
    uint32_t getLightningEnergy();

    uint8_t  readRegister(uint8_t reg);
    void     writeRegister(uint8_t reg, uint8_t value);
    void     maskRegisterBits(uint8_t reg, uint8_t mask, uint8_t value);

private:
    uint8_t   _cs;
    SPIClass &_spi;
    static const SPISettings _spiSettings;
    void _select();
    void _deselect();
};
