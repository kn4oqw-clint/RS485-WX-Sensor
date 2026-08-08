#pragma once
// ============================================================
// config.h — single place for pins, tuning values, intervals
//
// Board: STM32F411CEU6 (WeAct Black Pill V2.0)
// Node:  Stevenson enclosure weather node, solar powered
// ============================================================

#include <Arduino.h>

// ---- Node identity -----------------------------------------
#define NODE_ID "wx-01"

// ---- Debug console -----------------------------------------
// USBCON + USBD_USE_CDC means the core #defines Serial -> SerialUSB
// (see cores/arduino/WSerial.h). Spelled out here so it is unambiguous.
#define CONSOLE SerialUSB

// ---- SPI1 (shared bus: BME680 + AS3935) --------------------
// SCK PA5, MISO PA6, MOSI PA7 are the SPI1 defaults on this variant.
// The two devices need DIFFERENT SPI modes, so every access must be
// wrapped in beginTransaction()/endTransaction() with its own settings.
#define PIN_BME680_CS   PA4     // BME680, SPI MODE0
#define PIN_AS3935_CS   PB0     // AS3935, SPI MODE1
#define PIN_AS3935_IRQ  PA8     // EXTI8, rising edge

#define BME680_SPI_HZ   4000000
#define AS3935_SPI_HZ   2000000

// ---- I2C1 --------------------------------------------------
#define PIN_I2C_SCL     PB6
#define PIN_I2C_SDA     PB7
#define I2C_HZ          100000

#define ADDR_DS3231     0x68
#define ADDR_INA226     0x40    // not populated yet

// ---- GPS (GT-U7 / u-blox 7 clone) on USART1 ----------------
// Serial1 defaults to RX PA10 / TX PA9 on this variant, which matches
// the wiring, so no custom HardwareSerial instance is needed. Requires
// -D ENABLE_HWSERIAL1. Do NOT also declare HardwareSerial(PA10, PA9) —
// that would instantiate USART1 twice and the IRQ handlers will fight.
#define SerialGPS       Serial1
#define GPS_BAUD        9600
#define PIN_GPS_PPS     PA0     // EXTI0
#define PIN_GPS_PWR     PA1     // power gate MOSFET — NOT POPULATED YET

// ---- Uplink to hub (Cat6 run) on USART2 --------------------
// TX PA2 / RX PA3, plain 3.3V UART. Slow on purpose: it is a long
// unbalanced run. Raise only after it proves clean.
#define SerialUplink    Serial2
#define UPLINK_BAUD     9600

// ---- Status LED --------------------------------------------
#define PIN_LED         PC13    // active low

// ---- AS3935 tuning (set at the install site) ---------------
// These are starting points. Antenna resonance and noise floor must be
// verified on the roof, not on the bench — see the 'c' and 'n' commands
// in the board test.
#define AS3935_OUTDOOR_MODE     1   // 1 = outdoor AFE gain, 0 = indoor
#define AS3935_NOISE_FLOOR      2   // 0..7, raise if noise interrupts flood
#define AS3935_WATCHDOG_THRESH  2   // 0..15, raise if disturbers flood
#define AS3935_SPIKE_REJECT     2   // 0..15, raise to reject non-lightning
#define AS3935_MIN_STRIKES      1   // 1, 5, 9 or 16
#define AS3935_TUNING_CAP       0   // 0..15, set from the LCO sweep result
#define AS3935_MASK_DISTURBER   0   // 1 = stop reporting disturbers

// LCO target for antenna calibration: 500 kHz +/- 3.5%
#define AS3935_LCO_TARGET_HZ    500000
#define AS3935_LCO_TOLERANCE    0.035f
#define AS3935_LCO_DIVIDER      16      // DISP_LCO output is f_LCO / divider

// ---- Aggregation / scheduling ------------------------------
#define REPORT_INTERVAL_MS      (10UL * 60UL * 1000UL)  // 10 min uplink
#define SAMPLE_INTERVAL_MS      (30UL * 1000UL)         // 30 s into window
#define GPS_SYNC_INTERVAL_MS    (60UL * 60UL * 1000UL)  // hourly GPS resync
#define GPS_FIX_TIMEOUT_MS      (5UL * 60UL * 1000UL)   // give up on fix
#define LIGHTNING_HOLD_MS       (30UL * 60UL * 1000UL)  // storm "still active"

// Rolling window sized to hold one report interval of samples, plus slack.
#define SAMPLE_WINDOW_LEN       24

// ---- Bus safety --------------------------------------------
// No sensor read may stall the main loop; the node is on a roof.
#define I2C_TIMEOUT_MS          50
#define SPI_TIMEOUT_MS          50
#define IWDG_TIMEOUT_MS         8000
