# Sitron Labs DACx0501 Arduino Library

[![Version](https://img.shields.io/badge/version-0.1.0-blue.svg)](https://github.com/sitronlabs/SitronLabs_TexasInstruments_DACX0501_Arduino_Library)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE.txt)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-orange.svg)](https://www.arduino.cc/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-blue.svg)](https://platformio.org/)

A professional, easy-to-use Arduino library for the Texas Instruments DACx0501 series of digital-to-analog converters. This library provides comprehensive support for SPI and I2C communication interfaces with multiple resolution options and extensive configuration capabilities.

## Features

- **Multiple DAC Variants**: Support for DAC60501 (12-bit), DAC70501 (14-bit), and DAC80501 (16-bit)
- **Dual Communication**: Both SPI and I2C interfaces supported
- **Flexible Reference Options**: Internal 2.5V reference or external reference voltage
- **Configurable Gain**: 1x or 2x gain modes for different output voltage ranges
- **Power Management**: Power-down modes for low power applications
- **Multiple Output Methods**: Voltage, ratio, and raw code output options
- **Comprehensive Examples**: Four detailed example sketches included
- **Professional Documentation**: Extensive inline documentation and comments
- **Error Handling**: Robust error checking and reporting
- **Maker-Friendly**: Designed with ease of use and learning in mind

## Supported Devices

| Device | Resolution | Steps | Max Output (2x gain) |
|--------|------------|-------|---------------------|
| DAC60501 | 12-bit | 4,096 | 5.0V |
| DAC70501 | 14-bit | 16,384 | 5.0V |
| DAC80501 | 16-bit | 65,536 | 5.0V |

## Installation

### Arduino IDE Library Manager
1. Open Arduino IDE
2. Go to **Tools** → **Manage Libraries**
3. Search for "SitronLabs DACx0501"
4. Click **Install**

### Manual Installation
1. Download the latest release from [GitHub](https://github.com/sitronlabs/SitronLabs_TexasInstruments_DACX0501_Arduino_Library/releases)
2. Extract the ZIP file
3. Copy the `SitronLabs_TexasInstruments_DACX0501_Arduino_Library` folder to your Arduino `libraries` directory
4. Restart Arduino IDE

## Quick Start

### Basic I2C Example
```cpp
#include "dac80501.h"  // or dac60501.h, dac70501.h

dac80501 dac;

void setup() {
    Wire.begin();
    dac.begin(Wire, 0x48);  // I2C address 0x48
    dac.reference_internal_set();
    dac.output_voltage_set(2.5);  // Set output to 2.5V
}

void loop() {
    // Your code here
}
```

### Basic SPI Example
```cpp
#include "dac80501.h"  // or dac60501.h, dac70501.h

dac80501 dac;

void setup() {
    SPI.begin();
    dac.begin(SPI, 1000000, 10);  // SPI, 1MHz, CS pin 10
    dac.reference_internal_set();
    dac.output_voltage_set(2.5);  // Set output to 2.5V
}

void loop() {
    // Your code here
}
```

## Hardware Connections

### I2C Connections
| DAC Pin | Arduino Pin | Description |
|---------|-------------|-------------|
| VDD | 3.3V or 5V | Power supply |
| GND | GND | Ground |
| SDA | SDA | I2C data line |
| SCL | SCL | I2C clock line |
| A00 | GND, VDD, SDA or SCL | Address pin |
| VOUT | - | DAC output |

### SPI Connections
| DAC Pin | Arduino Pin | Description |
|---------|-------------|-------------|
| VDD | 3.3V or 5V | Power supply |
| GND | GND | Ground |
| SCLK | SCK | SPI clock |
| DIN | MOSI | SPI data input |
| CS | Digital pin 10 | Chip select |
| VOUT | - | DAC output |

### I2C Address Configuration
| A0 | I2C Address |
|----|-------------|
| GND | 0x48 |
| VDD | 0x49 |
| SDA | 0x4A |
| SCL | 0x4B |

## API Reference

### Constructor
```cpp
dac80501 dac;  // 16-bit DAC
dac70501 dac;  // 14-bit DAC
dac60501 dac;  // 12-bit DAC
```

### Initialization
```cpp
// I2C initialization
int begin(TwoWire& i2c_library, uint8_t i2c_address);

// SPI initialization
int begin(SPIClass& spi_library, int spi_speed, int pin_cs);
```

### Device Detection
```cpp
bool detect();  // Returns true if device is responding
```

### Reference Configuration
```cpp
int reference_internal_set();                    // Use internal 2.5V reference
int reference_external_set(float voltage);       // Use external reference
int reference_divider_set(dacx0501_reference_divider mode);  // Set reference divider
```

### Gain Configuration
```cpp
int gain_set(dacx0501_gain_mode mode);  // Set gain (1x or 2x)
```

### Output Methods
```cpp
int output_voltage_set(float voltage);     // Set output voltage directly
int output_ratio_set(float ratio);         // Set output as ratio (0.0 to 1.0)
int output_code_set(uint16_t code);        // Set output using raw DAC code
```

### Power Management
```cpp
int powerdown_mode_set(dacx0501_powerdown_mode mode);  // Set power-down mode
```

### Utility Methods
```cpp
uint8_t resolution_get();  // Get DAC resolution in bits
```

## Error Handling

All methods return an integer error code:
- `0`: Success
- Negative values: Error codes (e.g., `-EINVAL` for invalid parameter)

Common error codes:
- `-EINVAL`: Invalid parameter
- `-EIO`: Communication error
- `-ETIMEDOUT`: Device not responding

## Performance Considerations

- **SPI**: Faster communication, better for high-speed applications
- **I2C**: Simpler wiring, better for multi-device systems
- **Raw Code Method**: Fastest output method for waveform generation
- **Voltage Method**: Most intuitive for general use
- **Ratio Method**: Best for percentage-based control

## Troubleshooting

### Device Not Detected
1. Check power supply connections
2. Verify I2C/SPI wiring
3. Check I2C address configuration
4. Ensure proper pull-up resistors (I2C)

### Incorrect Output Voltage
1. Ensure [enough analaog headroom](https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/994874/faq-dac80501-why-is-the-output-not-updating)
2. Verify reference voltage setting
3. Check gain configuration
4. Ensure external reference is connected (if used)
5. Check for voltage drops in wiring

### Communication Errors
1. Verify SPI/I2C connections
2. Check clock speed settings
3. Ensure proper CS pin configuration (SPI)
4. Check for electrical noise

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

## License

This library is released under the MIT License. See [LICENSE.txt](LICENSE.txt) for details.

## Support

For support, please:
1. Check the [Issues](https://github.com/sitronlabs/SitronLabs_TexasInstruments_DACX0501_Arduino_Library/issues) page
2. Join our [Discord community](https://discord.gg/b6VzayWAMZ) for live support and discussion

---

**Sitron Labs** - Professional Arduino Libraries for Makers and Engineers

[Website](https://sitronlabs.com) | [Store](https://www.sitronlabs.com/store) | [GitHub](https://github.com/sitronlabs)
