// ============================================================
// bme.cpp — direct BME680 access via the Bosch bme68x driver
// ============================================================

#include <SPI.h>
#include "bme.h"
#include "config.h"
extern "C" {
#include "bme68x.h"
}

static struct bme68x_dev  dev;
static struct bme68x_conf conf;
static bool               ready = false;

// Every access wraps its own transaction. The AS3935 shares this bus and
// wants MODE1, so CPOL/CPHA must be reprogrammed per device — leaving
// settings applied across calls is what makes shared-bus code fail
// intermittently and in access-order-dependent ways.
static int8_t spiRead(uint8_t reg, uint8_t *data, uint32_t len, void *) {
    SPI.beginTransaction(SPISettings(BME680_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_BME680_CS, LOW);
    SPI.transfer(reg);
    for (uint32_t i = 0; i < len; i++) data[i] = SPI.transfer(0x00);
    digitalWrite(PIN_BME680_CS, HIGH);
    SPI.endTransaction();
    return 0;
}

static int8_t spiWrite(uint8_t reg, const uint8_t *data, uint32_t len, void *) {
    SPI.beginTransaction(SPISettings(BME680_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_BME680_CS, LOW);
    SPI.transfer(reg);
    for (uint32_t i = 0; i < len; i++) SPI.transfer(data[i]);
    digitalWrite(PIN_BME680_CS, HIGH);
    SPI.endTransaction();
    return 0;
}

static void spiDelayUs(uint32_t period, void *) { delayMicroseconds(period); }

bool bmeBegin() {
    ready = false;

    dev.intf     = BME68X_SPI_INTF;
    dev.read     = spiRead;
    dev.write    = spiWrite;
    dev.delay_us = spiDelayUs;
    dev.intf_ptr = NULL;
    dev.amb_temp = 20;

    if (bme68x_init(&dev) != BME68X_OK) return false;

    // Oversampling chosen for a fixed weather station, not a handheld:
    //   pressure 16x — the trend is the useful output and needs the
    //                  resolution; noise here is the forecast signal.
    //   temp      2x — plenty at 0.01 C reporting resolution.
    //   humidity  1x — slowest-moving quantity of the three.
    // IIR filter off: we average over a 10 minute window ourselves, and
    // an on-chip filter would add lag we cannot see or account for.
    conf.os_pres = BME68X_OS_16X;
    conf.os_temp = BME68X_OS_2X;
    conf.os_hum  = BME68X_OS_1X;
    conf.filter  = BME68X_FILTER_OFF;
    conf.odr     = BME68X_ODR_NONE;
    if (bme68x_set_conf(&conf, &dev) != BME68X_OK) return false;

    // Heater off — see the rationale in bme.h.
    struct bme68x_heatr_conf h;
    h.enable     = BME68X_DISABLE;
    h.heatr_temp = 0;
    h.heatr_dur  = 0;
    if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &h, &dev) != BME68X_OK) return false;

    ready = true;
    return true;
}

bool bmeRead(BmeReading &out) {
    out.valid = false;
    if (!ready) return false;

    if (bme68x_set_op_mode(BME68X_FORCED_MODE, &dev) != BME68X_OK) return false;

    // Milliseconds, not delayMicroseconds(): that function is unreliable
    // for the tens-of-milliseconds range this needs.
    uint32_t durUs = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &dev);
    delay(durUs / 1000 + 5);

    struct bme68x_data d;
    uint8_t n = 0;
    if (bme68x_get_data(BME68X_FORCED_MODE, &d, &n, &dev) != BME68X_OK || n == 0)
        return false;

    // Range-check before the value can reach the aggregation window. A
    // bad reading that gets averaged in is far harder to notice later
    // than one rejected here.
    if (d.temperature < -50.0f || d.temperature > 90.0f)   return false;
    if (d.humidity   <   0.0f  || d.humidity    > 100.0f)  return false;
    if (d.pressure   < 30000   || d.pressure    > 120000)  return false;

    out.temperature = d.temperature;
    out.humidity    = d.humidity;
    out.pressure    = d.pressure / 100.0f;
    out.valid       = true;

    // Feed ambient back so the driver's compensation tracks conditions.
    dev.amb_temp = (int8_t)d.temperature;
    return true;
}
