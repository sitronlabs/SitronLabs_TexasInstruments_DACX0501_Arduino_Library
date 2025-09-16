# SitronLabs Texas Instruments DACx0501 Arduino Library

A comprehensive Arduino library for the Texas Instruments DACx0501 series of digital-to-analog converters. This library supports both SPI and I2C communication interfaces and provides an easy-to-use API for controlling these high-precision DACs.

## Features

- **Dual Interface Support**: Both SPI and I2C communication protocols
- **Multiple DAC Models**: Support for DAC60501 (12-bit), DAC70501 (14-bit), and DAC80501 (16-bit)
- **Single-Channel Output**: High-precision single-channel DAC output
- **Flexible Reference**: Internal 2.5V reference or external reference support
- **Configurable Gain**: 1x and 2x gain options
- **Power Management**: Multiple power-down modes for energy efficiency
- **Broadcast Mode**: Synchronize multiple DAC devices
- **Modern Arduino API**: Easy-to-use methods with comprehensive error handling
- **Cross-Platform**: Compatible with Arduino, ESP32, ESP8266, and other Arduino-compatible boards

## Supported Devices

| Model | Resolution | Interface | Description |
|-------|------------|-----------|-------------|
| DAC60501 | 12-bit | SPI/I2C | 12-bit precision DAC |
| DAC70501 | 14-bit | SPI/I2C | 14-bit precision DAC |
| DAC80501 | 16-bit | SPI/I2C | 16-bit precision DAC |

## Installation

### Arduino Library Manager (Recommended)
1. Open Arduino IDE
2. Go to **Tools** → **Manage Libraries...**
3. Search for "SitronLabs Texas Instruments DACx0501"
4. Click **Install**

### Manual Installation
1. Download the library as a ZIP file
2. In Arduino IDE, go to **Sketch** → **Include Library** → **Add .ZIP Library...**
3. Select the downloaded ZIP file

## Hardware Connections

### SPI Connection
```
Arduino    DACx0501
--------   --------
MOSI   →   DIN
MISO   →   DOUT
SCK    →   SCLK
CS     →   CS
VCC    →   VDD
GND    →   GND
```

### I2C Connection
```
Arduino    DACx0501
--------   --------
SDA    →   SDA
SCL    →   SCL
VCC    →   VDD
GND    →   GND
```

## Quick Start

### Basic SPI Usage
```cpp
#include <dac80501.h>

// Create DAC instance (16-bit resolution)
dac80501 dac;

void setup() {
    // Initialize SPI interface
    dac.begin(SPI, 1000000, 10);  // SPI, 1MHz, CS pin 10
    
    // Set reference voltage
    dac.reference_voltage_set(3.3);
    
    // Set output voltage
    dac.output_voltage_set(1.65);  // 1.65V output
}

void loop() {
    // Your code here
}
```

### Basic I2C Usage
```cpp
#include <dac80501.h>

// Create DAC instance with custom I2C address
dac80501 dac(0x49);  // I2C address 0x49

void setup() {
    // Initialize I2C interface
    dac.begin(100000);  // 100kHz I2C speed
    
    // Set reference voltage
    dac.reference_voltage_set(5.0);
    
    // Set output ratio
    dac.output_ratio_set(1, 0.5);  // 50% of reference voltage
}

void loop() {
    // Your code here
}
```

## API Reference

### Constructor
```cpp
DACx0501(uint8_t bits, uint8_t i2c_address = 0x48)
```
- `bits`: DAC resolution (12, 14, or 16)
- `i2c_address`: I2C address (default: 0x48)

### Initialization
```cpp
// SPI initialization
int begin(SPIClass& spi_library, int spi_speed, int pin_cs)

// I2C initialization
int begin(int i2c_speed = 100000)
```

### Configuration
```cpp
// Set reference mode (internal or external)
int reference_mode_set(reference_mode mode)

// Set reference voltage (only for external reference)
int reference_voltage_set(float voltage)

// Set gain mode (1x or 2x)
int gain_set(bool gain_2x)

// Set power-down mode
int power_down_mode_set(power_down_mode mode)

// Enable/disable broadcast mode
int broadcast_mode_set(bool enable)
```

### Output Control
```cpp
// Set output ratio (0.0 to 1.0)
int output_ratio_set(float ratio)

// Set output voltage
int output_voltage_set(float voltage)

// Set raw DAC code
int output_code_set(uint32_t code)
```

### Utility Methods
```cpp
// Get device status
int status_get(uint8_t& status)

// Trigger DAC update
int update_trigger()

// Synchronize output
int sync_output()

// Convert voltage to DAC code
uint32_t voltage_to_code(float voltage)

// Convert DAC code to voltage
float code_to_voltage(uint32_t code)
```

### Reference Modes
```cpp
enum reference_mode {
    REF_INTERNAL,  // Use internal 2.5V reference
    REF_EXTERNAL   // Use external reference
};
```

### Power-Down Modes
```cpp
enum power_down_mode {
    POWER_DOWN_NORMAL,  // Normal operation
    POWER_DOWN_1K,      // 1 kΩ to GND
    POWER_DOWN_100K,    // 100 kΩ to GND
    POWER_DOWN_HIGHZ    // High impedance
};
```

## Examples

### Single-Channel Output
```cpp
#include <dac80501.h>

dac80501 dac;

void setup() {
    dac.begin(SPI, 1000000, 10);
    dac.reference_voltage_set(3.3);
    
    // Set output voltage
    dac.output_voltage_set(1.65);    // 1.65V output
}

void loop() {
    // Synchronize output
    dac.sync_output();
    delay(1000);
}
```

### Waveform Generation
```cpp
#include <dac80501.h>

dac80501 dac;
float angle = 0.0;

void setup() {
    dac.begin(SPI, 1000000, 10);
    dac.reference_voltage_set(3.3);
}

void loop() {
    // Generate sine wave
    float voltage = 1.65 + 1.65 * sin(angle);
    dac.output_voltage_set(voltage);
    
    angle += 0.1;
    if (angle >= 2 * PI) angle = 0;
    
    delay(10);
}
```

### Reference Configuration
```cpp
#include <dac80501.h>

dac80501 dac;

void setup() {
    dac.begin(SPI, 1000000, 10);
    
    // Use internal 2.5V reference (default)
    dac.reference_mode_set(REF_INTERNAL);
    // Internal reference is fixed at 2.5V
    
    // Or switch to external reference
    dac.reference_mode_set(REF_EXTERNAL);
    dac.reference_voltage_set(5.0);  // Set external reference to 5V
    
    // Check current reference mode
    if (dac.reference_mode_get() == REF_INTERNAL) {
        Serial.println("Using internal 2.5V reference");
    } else {
        Serial.print("Using external reference: ");
        Serial.print(dac.reference_voltage_get());
        Serial.println("V");
    }
}
```

### I2C with Multiple Devices
```cpp
#include <dac80501.h>

// Create multiple DAC instances with different addresses
dac80501 dac1(0x48);  // Default address
dac80501 dac2(0x49);  // Custom address

void setup() {
    // Initialize both devices
    dac1.begin(100000);
    dac2.begin(100000);
    
    // Set reference voltages
    dac1.reference_voltage_set(3.3);
    dac2.reference_voltage_set(5.0);
    
    // Enable broadcast mode on both
    dac1.broadcast_mode_set(true);
    dac2.broadcast_mode_set(true);
}

void loop() {
    // Set outputs on both devices
    dac1.output_voltage_set(0, 1.65);
    dac2.output_voltage_set(0, 2.5);
    
    // Synchronize all devices
    dac1.sync_output();
    delay(1000);
}
```

## Error Handling

The library returns error codes for all operations:
- `0`: Success
- `-EINVAL`: Invalid parameter
- `-ENODEV`: Device not initialized or not found
- `-EIO`: Communication error

```cpp
int result = dac.output_voltage_set(0, 2.0);
if (result != 0) {
    Serial.print("Error setting voltage: ");
    Serial.println(result);
}
```

## Advanced Features

### Reference Configuration
The DACx0501 series features a flexible reference system:
- **Internal Reference**: Built-in 2.5V precision reference (default)
- **External Reference**: Accepts external reference voltage from 1.25V to 5.5V

```cpp
// Use internal 2.5V reference (default behavior)
dac.reference_mode_set(REF_INTERNAL);

// Switch to external reference
dac.reference_mode_set(REF_EXTERNAL);
dac.reference_voltage_set(3.3);  // Set external reference voltage

// Check current reference configuration
if (dac.reference_mode_get() == REF_INTERNAL) {
    Serial.println("Internal 2.5V reference active");
} else {
    Serial.print("External reference: ");
    Serial.print(dac.reference_voltage_get());
    Serial.println("V");
}
```

**Note**: When using internal reference, the reference voltage is fixed at 2.5V and cannot be changed. When using external reference, you must set the reference voltage to match your external reference source.

### Gain Configuration
The DACx0501 supports configurable gain modes:
- **1x Gain**: Output range 0V to VREF
- **2x Gain**: Output range 0V to 2×VREF

```cpp
// Enable 2x gain
dac.gain_set(true);

// Now output can go up to 2 × reference voltage
dac.output_voltage_set(6.6);  // Valid with 3.3V reference and 2x gain
```

### Power Management
```cpp
// Set power-down mode for energy efficiency
dac.power_down_mode_set(POWER_DOWN_100K);

// Return to normal operation
dac.power_down_mode_set(POWER_DOWN_NORMAL);
```

### Broadcast Mode
Broadcast mode allows synchronizing multiple DAC devices:
```cpp
// Enable broadcast mode
dac.broadcast_mode_set(true);

// Set outputs on multiple devices
dac.output_voltage_set(0, 1.0);
dac.output_voltage_set(1, 2.0);

// Synchronize all devices
    dac.sync_output();
```

## Troubleshooting

### Common Issues

1. **Device not responding**
   - Check wiring connections
   - Verify I2C address (use I2C scanner sketch)
   - Ensure power supply is adequate

2. **SPI communication errors**
   - Verify CS pin configuration
   - Check SPI speed (max 50MHz)
   - Ensure proper SPI mode (MODE0)

3. **I2C communication errors**
   - Check I2C speed (max 400kHz)
   - Verify pull-up resistors
   - Check for address conflicts

### Debug Tips

```cpp
// Check device status
uint8_t status;
if (dac.status_get(status) == 0) {
    Serial.print("Device status: 0x");
    Serial.println(status, HEX);
}

// Verify interface type
if (dac.i2c_is()) {
    Serial.println("Using I2C interface");
} else {
    Serial.println("Using SPI interface");
}
```

## Contributing

We welcome contributions! Please feel free to submit issues, feature requests, or pull requests.

## License

This library is licensed under the terms specified in the LICENSE.txt file.

## Support

For support and questions:
- Create an issue on GitHub
- Contact support@sitronlabs.com

## Version History

- **v1.1.0**: Added reference management functionality
  - Internal vs external reference selection
  - Automatic reference voltage management
  - Enhanced configuration options
  - Improved documentation

- **v1.0.0**: Initial release with SPI and I2C support
  - Support for DAC60501 (12-bit), DAC70501 (14-bit), and DAC80501 (16-bit)
  - Dual interface support (SPI/I2C)
  - Single-channel output control
  - Power management features
  - Comprehensive error handling
