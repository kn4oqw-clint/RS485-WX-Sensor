# RS485-WX-Sensor

A weather sensor node built on an STM32F411CEU6 "Black Pill". It reads a BME680
(temperature, humidity, pressure, air quality) and an AS3935 lightning detector,
then streams one JSON line per second over RS485 to a host — in my case a
Raspberry Pi.

## Hardware

| Part | Notes |
|---|---|
| STM32F411CEU6 Black Pill | Arduino framework via STM32duino |
| BME680 | SPI, driven through Bosch BSEC |
| AS3935 | SPI, on an AMS eval board |
| MAX485 | UART2 TX to RS485 |

### Wiring

| MCU pin | Connects to |
|---|---|
| PA2 | UART2 TX → MAX485 DI |
| PA3 | UART2 RX (unused) |
| PA4 | BME680 CS |
| PA5 | SPI1 SCK |
| PA6 | SPI1 MISO |
| PA7 | SPI1 MOSI |
| PA8 | AS3935 IRQ (rising edge) |
| PB0 | AS3935 CS |
| PC13 | Onboard LED (active low) |

MAX485 `DE`/`RE` are tied to 3.3V, so the node only ever transmits.

On the AS3935 eval board: `SI` → GND (this selects SPI on that board, which is
the opposite of what the bare datasheet says), `EN-V` → 3.3V, `A0` and `A1` → GND.

## Output

One JSON object per second on UART2 at 115200 8N1. Debug logging goes to USB CDC
separately, so the RS485 link stays clean.

```json
{"node":"wx-01","ts":123456,"t":21.44,"h":48.20,"p":1013.2,"gas":128340,"iaq":52.1,"iaq_acc":2,"iaq_valid":1,"co2":612.4,"co2_acc":2,"bvoc":0.634,"bvoc_acc":2,"lx_dist":0,"lx_energy":0,"uptime":123}
```

| Field | Meaning |
|---|---|
| `node` | Node ID string |
| `ts` | Milliseconds since boot |
| `t` | Temperature °C (heat-compensated) |
| `h` | Relative humidity % (heat-compensated) |
| `p` | Pressure hPa |
| `gas` | Raw gas resistance, ohms |
| `iaq` | Static IAQ index |
| `iaq_acc` | BSEC accuracy for IAQ, 0–3 |
| `iaq_valid` | 1 once `iaq_acc` reaches 1 |
| `co2` | CO2 equivalent, ppm |
| `co2_acc` | BSEC accuracy for CO2, 0–3 |
| `bvoc` | Breath VOC equivalent, ppm |
| `bvoc_acc` | BSEC accuracy for bVOC, 0–3 |
| `lx_dist` | Lightning distance in km, 0 if none in the last 30s |
| `lx_energy` | Lightning energy, 0 if none in the last 30s |
| `uptime` | Seconds since boot |

BSEC needs time to calibrate. `iaq_acc` starts at 0 and climbs to 3 over hours of
runtime — treat the air quality numbers as meaningless until `iaq_valid` is 1.

## Building

You need PlatformIO and an ST-Link.

BSEC is not in this repo because Bosch's license doesn't allow redistributing it.
Download it yourself first:

1. Get BSEC from https://www.bosch-sensortec.com/software-tools/software/bsec/
2. Create `lib/BSEC/` in this project and copy in:
   - `libalgobsec.a` from `algo/normal_version/bin/STM32/STM32F4/`
   - `src/bsec_datatypes.h` and `src/bsec_interface.h` from `algo/normal_version/inc/`
   - `src/bsec.cpp`, `src/bsec.h`, `src/bme68x.c`, `src/bme68x.h`, `src/bme68x_defs.h`
     from the Bosch BSEC Arduino library and BME68x sensor API

Then:

```
pio run              # build
pio run -t upload    # flash over ST-Link
pio device monitor   # USB CDC debug output
```

Note that `platformio.ini` unsets `-mfloat-abi=hard` and builds `softfp`, because
the prebuilt BSEC blob for STM32F4 is softfp. Mixing them won't link.

## Configuration

Change the node name with `NODE_ID` in [src/main.cpp](src/main.cpp). Set
`DEBUG` to 0 in the same file to drop the USB CDC logging.

AS3935 sensitivity is tuned in `setup()` — noise floor, watchdog threshold,
spike rejection, and minimum strike count. Defaults here are fairly sensitive
because the node lives outdoors and away from noise sources. If you get a lot of
`disturber` messages on the debug console, raise the watchdog threshold.

## License

MIT, see [LICENSE](LICENSE). This does not cover Bosch BSEC, which has its own
license and is not distributed here.
