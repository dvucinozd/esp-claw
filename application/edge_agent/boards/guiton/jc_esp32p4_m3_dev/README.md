# JC-ESP32P4-M3-DEV Board Notes

This board definition is based on vendor files from:

- `C:\Users\klikn\Desktop\JC-ESP32P4-M3-DEV\1-Demo`
- `C:\Users\klikn\Desktop\JC-ESP32P4-M3-DEV\2-Specification`
- `C:\Users\klikn\Desktop\JC-ESP32P4-M3-DEV\6-User_Manual`

## Mapped Components

- MCU: `ESP32-P4` (board info)
- Wireless companion: `ESP32-C6` (not yet modeled in board_manager)
- Audio codec: `ES8311` over I2C + I2S
- Microphone/speaker path: enabled via `audio_adc` and `audio_dac`
- Display: `JD9365` MIPI-DSI panel (`800x1280`)
- Touch: `GT911` over I2C
- Camera: MIPI-CSI (`camera` device)
- SD card: SDMMC slot0 (`fs_sdcard`)
- SPI flash FS: `fs_spiffs`
- Backlight: LEDC fallback (`GPIO23`)
- RS485: UART1 (`TX=26`, `RX=27`) peripheral added

## Important Validation Notes

- Vendor demos show two brightness control variants:
  - PWM/LEDC on `GPIO23`
  - I2C brightness controller (`0x45`, register `0x96`)
- If LCD init succeeds but panel remains dark, implement I2C backlight control path.
- Vendor Arduino audio demo references ES8311 address `0x18`, while ESP board-manager examples often use `0x30`.
  If audio init fails, test the alternate address.

## Pin References Used

- I2C: SDA `7`, SCL `8`
- I2S: MCLK `13`, BCLK `12`, WS `10`, DOUT `9`, DIN `48`
- PA control: `11`
- SDMMC (slot0 mux): D0 `39`, D1 `40`, D2 `41`, D3 `42`, CMD `44`, CLK `43`
- RS485 (example): UART1 TX `26`, RX `27`
