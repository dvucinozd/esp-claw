# JC-ESP32P4-M3-DEV Board Notes

This directory contains the ESP Board Manager board definition for `jc_esp32p4_m3_dev`.

## Covered Hardware

- MCU: `ESP32-P4`
- Wireless companion: `ESP32-C6` (used through ESP-Hosted path in app)
- Audio codec: `ES8311` over I2C + I2S (`audio_dac`, `audio_adc`)
- Display: `JD9365` MIPI-DSI (`display_lcd`)
- Touch: `GT911` over I2C (`lcd_touch`)
- Camera: `OV5647` over MIPI-CSI (`camera`)
- Storage: SD card via SDMMC (`fs_sdcard`)
- Backlight: LEDC (`lcd_brightness`)
- Extra peripheral mapping: UART RS485 example (`uart_rs485`)

## Bring-Up Notes

- `fs_spiffs` is intentionally not enabled in this board profile.
- `display_lcd` and `lcd_touch` are configured with `init_skip: true` and require board-specific init path in `setup_device.c`.
- Camera control bus uses I2C (`GPIO7/GPIO8`), while CSI lanes/clock use dedicated MIPI-CSI pins.
- `ov5647_low_light.json` is kept as an optional tuning profile for low-light experiments.

## Pin Summary

- I2C: SDA `7`, SCL `8`
- I2S: MCLK `13`, BCLK `12`, WS `10`, DOUT `9`, DIN `48`
- PA control: `11`
- SDMMC slot0 mux: D0 `39`, D1 `40`, D2 `41`, D3 `42`, CMD `44`, CLK `43`
- RS485 example: UART1 TX `26`, RX `27`
- Camera SCCB: SDA `7`, SCL `8`
- Camera RESET/PWDN: `-1` / `-1` (not connected)

## Validation Command

From `application/edge_agent`, validate/generate board code with:

```bash
idf.py bmgr -c ./boards -b jc_esp32p4_m3_dev
```
