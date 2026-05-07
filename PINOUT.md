# Aura-Synth — ESP32 (Feather V2) pinout

Firmware pin assignments for **Adafruit ESP32 Feather V2** (ESP32-WROOM). Silk labels are the usual Feather names; verify against [Adafruit’s pinout](https://learn.adafruit.com/adafruit-esp32-feather-v2/pinouts) for your PCB revision.

## Wiring vs firmware

**Existing Aura hardware** (DAC, ToFs, 74HC595, WS2812) uses fixed GPIOs in `i2s_output.h`, `sensor_task.cpp`, and `led_task.h`. Those definitions were **not** moved to make room for the front panel.

The **3×3 matrix + sliders** use **separate** GPIOs documented in `panel_input.c`. If an older test sketch used **12, 14, 27, 33** for the matrix, that conflicts with this project’s **595 / XSMT / LED** pins — wire the matrix to the **`panel_input.c`** map instead.

- **DEMO_MODE**: only the core columns below (no matrix init).
- **INTEGRATION_MODE**: also enables matrix rows/cols and ADC lines in the “Integration” column.

## GPIO reference (ESP32 — numeric order)

The ESP32 has no GPIO **24** or **28–31** (chip limitation).

| GPIO | Typical Feather V2 | Core firmware | Integration (matrix / ADC) | Notes |
|------|-------------------|---------------|----------------------------|--------|
| 0 | `IO0` / NeoPixel data | — | — | Onboard RGB; strapping |
| 1 | *(module TXD)* | — | — | Flash / UART |
| 2 | STEMMA / Neo power | — | — | Board control |
| 3 | *(module RXD)* | — | — | Flash / UART |
| 4 | **A5** | I2S DOUT → DAC DIN | — | |
| 5 | **SCK** | — | Matrix **Row2** (out) | SPI silk |
| 6 | — | — | — | SPI flash (internal) |
| 7 | **RX** | — | Matrix **Col0** (in, PU) | UART RX silk |
| 8 | **TX** | — | — | UART TX — **not used** by firmware (matrix col1 is **GPIO 21**) |
| 9–11 | — | — | — | SPI flash (internal) — **do not use** |
| 12 | **D12** | 595 **SHCP** | — | ToF XSHUT chain |
| 13 | **D13** | 595 **STPC** | — | ToF XSHUT chain |
| 14 | **D14** | 595 **DS** | — | ToF XSHUT chain |
| 15 | **D15** | — | Matrix **Row0** (out) | |
| 16–18 | *(often NC on Feather)* | — | — | Free if broken out |
| 19 | **MOSI** | — | Matrix **Col2** (in, PU) | SPI silk |
| 20 | **SCL** | I2C SCL (ToFs) | — | STEMMA QT |
| 21 | **MISO** | — | Matrix **Col1** (in, PU) | SPI MISO silk |
| 22 | **SDA** | I2C SDA (ToFs) | — | STEMMA QT |
| 23 | *(varies)* | — | — | Often not on header |
| 24 | — | — | — | *Not a GPIO on ESP32* |
| 25 | **A1** | I2S WS (LRCK) | — | |
| 26 | **A0** | I2S BCLK | — | |
| 27 | *(header)* | **XSMT** → PCM5102A (out, high) | — | Unmute DAC |
| 28–31 | — | — | — | *Not GPIOs on ESP32* |
| 32 | **D32** | — | Matrix **Row1** (out) | |
| 33 | **D33** | WS2812 data | — | RMT |
| 34 | **A2** | — | Slider 1 → `master_volume` | Input only; ADC1 CH6 |
| 35 | VBAT sense | — | — | Input only; divider to battery |
| 36 | **A4** | — | Pot → `master_delay_mix` | Input only; ADC1 CH0 |
| 37 | Corner | — | — | Input only on V2 |
| 38 | **USER** | — | — | Button to GND |
| 39 | **A3** | — | Slider 2 → `master_filter` | Input only; ADC1 CH3 |

### Integration-only GPIOs (summary)

| GPIO | Role |
|------|------|
| 5 | Matrix row 2 (output) |
| 7 | Matrix column 0 (input, pull-up), **RX** silk |
| 21 | Matrix column 1 (input, pull-up), **MISO** silk |
| 15 | Matrix row 0 (output) |
| 19 | Matrix column 2 (input, pull-up) |
| 32 | Matrix row 1 (output) |
| 34 | Slider 1 (ADC) |
| 36 | Pot (ADC) |
| 39 | Slider 2 (ADC) |

## Matrix topology (INTEGRATION_MODE)

Rows are **outputs** (idle high; scan pulls one row low). Columns are **inputs** with **internal pull-ups**; a press connects that column to the active row (column reads low).

|  | Col0 (GPIO 7, RX) | Col1 (GPIO 21, MISO) | Col2 (GPIO 19, MOSI) |
|--|---------------|---------------|----------------|
| **Row0 (15)** | Pad0 | Pad1 | Pad2 |
| **Row1 (32)** | Pad3 | Pad mode toggle | Record |
| **Row2 (5)** | Clear | Play / pause | Track cycle |

Each key is **row-to-column** only (no shared ground bus to the buttons).

## Quick “used GPIO” sets

**Always (both modes):** 4, 12, 13, 14, 20, 22, 25, 26, 27, 33  

**INTEGRATION_MODE add:** 5, 7, 15, 19, 21, 32, 34, 36, 39  

**Still free in this firmware (typical Feather):** 0, 2, 8 (TX — unused here), 16–18, 23, 35 (battery), 37, 38 — subject to board strapping and your own constraints.
