## 1. Hardware Required
- ESP32 dev board (with ADC1 pins 32/34/35 exposed)
- BME280 (I²C, address 0x76) — temperature/humidity/pressure
- BH1750 (I²C, address 0x23) — light
- MQ-2 gas sensor (analog out)
- Soil moisture sensor (analog out)
- Rain sensor (analog out)
- Relay module (active level selectable)
- 3.3 V / 5 V supply as required by sensors

## 2. Wiring
| Signal | ESP32 GPIO | Notes |
|---|---|---|
| SDA | 21 | BME280 + BH1750 |
| SCL | 22 | BME280 + BH1750 |
| MQ-2 AOUT | 32 | ADC1_CH4 |
| Soil AOUT | 34 | ADC1_CH6, input-only |
| Rain AOUT | 35 | ADC1_CH7, input-only |
| Relay IN | 25 | Digital out |
| BOOT button | 0 | Built-in; hold 5 s = factory reset |

- Power sensors from 3.3 V unless they require 5 V (MQ-2 heater typically 5 V; its AOUT must not exceed 3.3 V — use a divider if needed).
- Share GND between ESP32 and all sensors.
- Pull-ups on SDA/SCL are usually on the breakout boards; otherwise add 4.7 kΩ to 3.3 V.

## 3. Build & Flash
1. Install Arduino IDE + ESP32 board support (Espressif).
2. Install libraries:
   - **Adafruit BME280 Library** + **Adafruit Unified Sensor**
   - **BH1750** (by Christopher Laws)
3. Open `v3.txt` (rename to `.ino` matching folder name).
4. Select board: *ESP32 Dev Module*, correct COM port.
5. Upload. Open Serial Monitor at 115200 baud.

## 4. First Boot / Wi-Fi Setup
1. After boot the ESP32 creates an access point:
   - SSID: `ESP32_Multi_Sensors`
   - Password: `12345678`
2. Connect your phone/PC to that AP. A captive-portal page should open; if not, browse to `http://192.168.4.1`.
3. Change SSID/password on the **WiFi AP** tab (8–63 char password). Device reboots automatically.

## 5. Using the Web UI
- **Dashboard** — live temp, humidity, pressure, light, MQ-2 %, soil %, rain %, relay state. Buttons: Relay **On / Off / Auto**.
- **Settings** — calibration & relay rules:
  - Soil: Dry ADC / Wet ADC (use observed raw values).
  - MQ-2: Clean ADC / Dirty ADC.
  - Rain: Dry ADC / Wet ADC.
  - Offsets for BME280 (temp/hum ×10), pressure (Pa), lux (×10).
  - Relay: mode (Off/Auto/Manual ON/Manual OFF), rule (soil above/below, MQ-2 above, rain above %), threshold, min ON time, invert.
  - Sampling interval (500–3 600 000 ms).
- **System** — firmware/build, uptime, boot/save counts, heap, reset reason, boot-button hold progress, reboot.

## 6. Calibration Procedure
1. Note raw ADC values shown on the dashboard for each sensor in known conditions:
   - Soil in dry air → set **Dry ADC**; in water → **Wet ADC**.
   - MQ-2 in clean air → **Clean ADC**; near a smoke source → **Dirty ADC**.
   - Rain sensor dry → **Dry ADC**; with water droplets → **Wet ADC**.
2. Save; the percentage bars update on the next sample.
3. For temperature/humidity/pressure/lux, enter small offsets (e.g. `temp_off = 15` means +1.5 °C) if readings differ from a reference.

## 7. Relay Modes
- **Off** — relay always de-energised.
- **Manual ON / OFF** — fixed state.
- **Auto** — follows the selected rule; `Min ON time` keeps it on at least that long; `Invert` flips the output logic for active-low relay boards.

## 8. Reset / Recovery
- **Factory reset**: hold BOOT (GPIO0) for 5 s, or use the button on the WiFi tab. Erases NVS and reboots to defaults.
- **Reboot**: System tab.

## 9. Notes
- Settings are saved to NVS after a 5 s cooldown (and every 15 min max delay) to reduce flash wear.
- I²C bus auto-recovers after repeated sensor failures.
- If a sensor is not detected, the dashboard shows `FAIL` for BME280/BH1750; check wiring and address (BME280 sometimes 0x77 — change `BME280_I2C_ADDR` if needed).
- Only 4 clients can join the AP at once; IP is `192.168.4.1`.
