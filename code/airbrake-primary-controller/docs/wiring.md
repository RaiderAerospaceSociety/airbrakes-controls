# Wiring And Pins

Pins
- See `include/pins.h` for the canonical pin map. Values can be overridden via PlatformIO `build_flags`.

I2C
- SDA: `PIN_SDA1` (default 33), SCL: `PIN_SCL1` (default 38) on Feather S3
- Bus initialized once in `bus_setup()`; optional scan prints discovered addresses at boot when DEBUG is enabled.

SPI
- SCK/MISO/MOSI: `PIN_SCK1`/`PIN_MISO1`/`PIN_MOSI1`
- Chip selects: `PIN_CS_BMP1`, `PIN_CS_SD1`

Device addresses
- USFSMAX (via MAX32660): per vendor defaults
- MPU6050: 0x68 (fallback 0x69)
- BMP390: per breakout wiring (SPI by default in this project)

Tips
- Keep I2C wires short and twisted when possible; use 4.7k–10k pull-ups if needed.
- For high-speed or noisy environments, lower I2C clock or add filtering at the device layer.

