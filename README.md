# ESPresso

Custom ESP32-C3 Supermini controller for the Gaggia Classic Pro. Handles boiler PID via SSR, shot timing, brew switch sense, and future pump PWM pressure profiling via MOC3021 + BTA41-600B TRIAC. Built with the Arduino framework.

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | ESP32-C3 Supermini (22×18mm, GPIOs 0–10, 20, 21) |
| Boiler control | Fotek 25DA SSR on GPIO10 via 330Ω |
| Thermocouple | MAX31855 on GPIOs 4 (SCK), 5 (MISO), 7 (CS) |
| Display | SSD1306 128×64 OLED on I2C, GPIOs 8 (SDA), 9 (SCL) |
| Pump control | MOC3021 + BTA41-600B TRIAC, GPIO2 PWM (pending MOC3041 + snubber) |
| Brew sense | Dry contact on second side of brew switch, GPIO1, 10kΩ pullup, active low |
| Pressure | XDB401 0–1.2MPa via 10kΩ/20kΩ divider → GPIO6 (not yet wired) |
| Mode toggle | Momentary to GND on GPIO20, firmware latched |
| Temp buttons | Resistor ladder on GPIO21 — up 10kΩ, down 47kΩ, 10kΩ pullup |
| Pot wiper | GPIO3, replacement pot coming (100nF cap to GND) |

# Electrical Schematic
<img width="3507" height="2480" alt="image" src="https://github.com/user-attachments/assets/6106e972-beaf-4b9a-ba1f-e3d61d97681c" />

## Firmware Features

- **Boiler PID** — Kp=38, Ki=4, Kd=250, 2s cycle time, SSR on GPIO10
- **Brew/steam modes** — toggle GPIO20; brew 185–220°F (default 205°F), steam 250–300°F (default 280°F)
- **Shot timer** — starts on brew switch close, stops on open, displayed live on OLED
- **Temp adjustment** — resistor ladder buttons ±1°F per press, long-press temp-up for settings stub
- **OLED** — mode, boiler state, current temp (large), setpoint, PID on-time, shot timer / error

## Pin Map

| GPIO | Function | Status |
|------|----------|--------|
| 0 | free | — |
| 1 | Brew switch sense | wired |
| 2 | MOC3021 PWM out | wired, pending MOC3041 + snubber |
| 3 | Pot wiper ADC | not yet wired |
| 4 | MAX31855 SCK | wired |
| 5 | MAX31855 MISO | wired |
| 6 | Pressure transducer ADC | not yet wired |
| 7 | MAX31855 CS | wired |
| 8 | OLED SDA | wired |
| 9 | OLED SCL | wired |
| 10 | SSR boiler out | wired |
| 20 | Mode toggle button | wired |
| 21 | Resistor ladder buttons | wired |

## Pending

- MOC3041 (zero crossing) to replace MOC3021 — required for AC pump PWM
- X2 snubber capacitor (100nF/275VAC) + 39Ω across TRIAC MT1-MT2
- XDB401 pressure transducer wiring and firmware
- Replacement pot for pressure profiling
- Housing design in FreeCAD

## Repository Contents
    /ESPresso    — Arduino sketch (ESP32-C3)
    /docs        — wiring notes and schematic

