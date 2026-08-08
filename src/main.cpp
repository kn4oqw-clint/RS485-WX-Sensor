// ============================================================
// main.cpp — RS485 Weather Sensor Node
// STM32F411CEU6 Black Pill / STM32duino / PlatformIO
//
// Architecture:
//   Sensors  -> SPI1 (BME680/BSEC + AS3935)
//   Output   -> JSON over UART2 (PA2=TX, PA3=RX) @ 115200
//   Debug    -> USB CDC (SerialUSB)
//
// Pin Assignment:
//   PA2  -> UART2 TX  -> MAX485 DI -> RS485 -> Pi ttyAMA4
//   PA3  -> UART2 RX  (unused, available)
//   PA4  -> BME680 CS
//   PA5  -> SPI1 SCK
//   PA6  -> SPI1 MISO
//   PA7  -> SPI1 MOSI
//   PA8  -> AS3935 IRQ (EXTI, rising edge)
//   PB0  -> AS3935 CS
//   PC13 -> Onboard LED (active low)
//
// MAX485 DE/RE tied to 3.3V (always transmit)
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include "as3935.h"
#include "bsec.h"

// ---- Node identity -----------------------------------------
#define NODE_ID "wx-01"

// ---- Debug macros (USB CDC) --------------------------------
#define DEBUG 1
#if DEBUG
  #define DBG(...)   SerialUSB.print(__VA_ARGS__)
  #define DBGLN(...) SerialUSB.println(__VA_ARGS__)
  #define DBGF(...)  SerialUSB.printf(__VA_ARGS__)
#else
  #define DBG(...)
  #define DBGLN(...)
  #define DBGF(...)
#endif

// ---- Pin definitions ---------------------------------------
#define PIN_BME680_CS  PA4
#define PIN_AS3935_CS  PB0
#define PIN_AS3935_IRQ PA8

// ---- BSEC / BME680 -----------------------------------------
Bsec bme;
static bool bmeOk = false;

// ---- AS3935 ------------------------------------------------
AS3935 lightning(PIN_AS3935_CS, SPI);
volatile bool lightningIRQ = false;
static uint8_t  lastLightningDist   = 0;
static uint32_t lastLightningEnergy = 0;
static unsigned long lastLightningTime = 0;

void onLightningIRQ() { lightningIRQ = true; }

// ---- JSON output -------------------------------------------
static unsigned long lastJsonMs = 0;
#define JSON_INTERVAL_MS 1000

void sendJSON() {
    bool lightningPresent = (millis() - lastLightningTime < 30000) && (lastLightningDist > 0);

    // bme.temperature and bme.humidity are heat-compensated by BSEC
    // bme.staticIaq is correct for a fixed outdoor node (vs bme.iaq for handheld)
    Serial2.print(F("{"));
    Serial2.print(F("\"node\":\""));      Serial2.print(F(NODE_ID));   Serial2.print(F("\""));
    Serial2.print(F(",\"ts\":"));         Serial2.print(millis());
    Serial2.print(F(",\"t\":"));          Serial2.print(bme.temperature, 2);
    Serial2.print(F(",\"h\":"));          Serial2.print(bme.humidity, 2);
    Serial2.print(F(",\"p\":"));          Serial2.print(bme.pressure / 100.0f, 1);
    Serial2.print(F(",\"gas\":"));        Serial2.print((unsigned long)bme.gasResistance);
    Serial2.print(F(",\"iaq\":"));        Serial2.print(bme.staticIaq, 1);
    Serial2.print(F(",\"iaq_acc\":"));    Serial2.print((int)bme.staticIaqAccuracy);
    Serial2.print(F(",\"iaq_valid\":")); Serial2.print(bme.staticIaqAccuracy >= 1 ? 1 : 0);
    Serial2.print(F(",\"co2\":"));        Serial2.print(bme.co2Equivalent, 1);
    Serial2.print(F(",\"co2_acc\":"));    Serial2.print((int)bme.co2Accuracy);
    Serial2.print(F(",\"bvoc\":"));       Serial2.print(bme.breathVocEquivalent, 3);
    Serial2.print(F(",\"bvoc_acc\":"));   Serial2.print((int)bme.breathVocAccuracy);
    Serial2.print(F(",\"lx_dist\":"));    Serial2.print(lightningPresent ? lastLightningDist : 0);
    Serial2.print(F(",\"lx_energy\":")); Serial2.print(lightningPresent ? lastLightningEnergy : 0UL);
    Serial2.print(F(",\"uptime\":"));     Serial2.print(millis() / 1000);
    Serial2.println(F("}"));
}

// ---- Setup -------------------------------------------------
void setup() {
#if DEBUG
    SerialUSB.begin(115200);
    unsigned long t = millis();
    while (!SerialUSB && millis() - t < 3000);
    DBGLN(F("\n=== RS485 Weather Sensor Node — JSON/UART2 ==="));
    DBGLN(F("Node: " NODE_ID));
#endif

    // UART2 — JSON data link to Pi via MAX485
    Serial2.begin(115200);

    // CS pins high before SPI init
    pinMode(PIN_AS3935_CS, OUTPUT);
    pinMode(PIN_BME680_CS, OUTPUT);
    digitalWrite(PIN_AS3935_CS, HIGH);
    digitalWrite(PIN_BME680_CS, HIGH);
    delay(10);

    SPI.begin();

    // ---- AS3935 --------------------------------------------
    if (lightning.begin()) {
        lightning.setOutdoor();
        lightning.setNoiseFloor(1);
        lightning.setWatchdogThreshold(2);
        lightning.setSpikeRejection(2);
        lightning.setMinLightningEvents(1);
        DBGLN(F("AS3935 OK"));
    } else {
        DBGLN(F("AS3935 FAILED — check SPI/CS wiring"));
    }

    // Attach IRQ after begin() so chip is configured first
    pinMode(PIN_AS3935_IRQ, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onLightningIRQ, RISING);

    // ---- BME680 / BSEC -------------------------------------
    bme.begin(PIN_BME680_CS, SPI);
    if (bme.bsecStatus != BSEC_OK) {
        DBG(F("BSEC failed: "));
        DBGLN(bme.bsecStatus);
    } else {
        bsec_virtual_sensor_t sensorList[] = {
            BSEC_OUTPUT_RAW_TEMPERATURE,
            BSEC_OUTPUT_RAW_HUMIDITY,
            BSEC_OUTPUT_RAW_PRESSURE,
            BSEC_OUTPUT_RAW_GAS,
            BSEC_OUTPUT_IAQ,
            BSEC_OUTPUT_STATIC_IAQ,
            BSEC_OUTPUT_CO2_EQUIVALENT,
            BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        };
        bme.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
        bmeOk = true;
        DBGLN(F("BME680/BSEC OK"));
    }

    // ---- LED -----------------------------------------------
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH); // active low, off

    DBGLN(F("Setup complete — streaming JSON on UART2 @ 115200"));
}

// ---- Loop --------------------------------------------------
void loop() {

    // ---- AS3935 IRQ handling --------------------------------
    if (lightningIRQ) {
        lightningIRQ = false;
        delay(2); // AS3935 needs 2ms before register read
        uint8_t intSource = lightning.getInterruptSource();

        if (intSource == AS3935_INT_LIGHTNING) {
            lastLightningDist   = lightning.getDistanceEstimate();
            lastLightningEnergy = lightning.getLightningEnergy();
            lastLightningTime   = millis();
            DBGF("LIGHTNING! dist=%dkm energy=%lu\n", lastLightningDist, lastLightningEnergy);
            for (int i = 0; i < 5; i++) {
                digitalWrite(PC13, LOW);  delay(50);
                digitalWrite(PC13, HIGH); delay(50);
            }
        } else if (intSource == AS3935_INT_DISTURBER) {
            DBGLN(F("AS3935: disturber"));
        } else if (intSource == AS3935_INT_NOISE) {
            DBGLN(F("AS3935: noise"));
        }
    }

    // ---- BSEC run ------------------------------------------
    if (bmeOk && bme.run()) {
        DBGF("T=%.2fC  RH=%.2f%%  P=%.1fhPa  Gas=%lu  sIAQ=%.1f(acc=%d)  CO2=%.1f  bVOC=%.3f\n",
            bme.temperature,
            bme.humidity,
            bme.pressure / 100.0f,
            (unsigned long)bme.gasResistance,
            bme.staticIaq,
            (int)bme.staticIaqAccuracy,
            bme.co2Equivalent,
            bme.breathVocEquivalent
        );
    }

    // ---- JSON transmit every 1s ----------------------------
    if (millis() - lastJsonMs >= JSON_INTERVAL_MS) {
        lastJsonMs = millis();
        sendJSON();
    }

    // ---- LED heartbeat (500ms) -----------------------------
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    if (millis() - lastBlink > 500) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(PC13, ledState ? LOW : HIGH);
    }
}