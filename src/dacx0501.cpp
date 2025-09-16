/**
 * @file dacx0501.cpp
 * @brief Implementation of the DACx0501 Arduino Library
 * @author Sitron Labs
 * @version 0.1.0
 * @date 2024
 *
 * This file contains the implementation of the DACx0501 base class, providing
 * all the functionality for communicating with Texas Instruments DACx0501
 * series digital-to-analog converters via SPI or I2C interfaces.
 *
 * @copyright Copyright (c) 2024 Sitron Labs. All rights reserved.
 * @license This library is released under the MIT License.
 */

/* Self header */
#include "dacx0501.h"

/**
 * @brief Constructor for DACx0501 base class
 *
 * Initializes the DAC object with the specified resolution and sets up
 * default configuration values. The actual communication interface
 * (SPI or I2C) must be initialized separately using the begin() methods.
 *
 * @param[in] bits DAC resolution in bits (12, 14, or 16)
 *                 - 12 bits: DAC60501 (4096 steps)
 *                 - 14 bits: DAC70501 (16384 steps)
 *                 - 16 bits: DAC80501 (65536 steps)
 *
 * @note Default configuration:
 *       - Internal 2.5V reference
 *       - 2x gain mode
 *       - No reference divider
 *       - No interface initialized
 */
dacx0501::dacx0501(const uint8_t bits)
    : m_bits(bits),                               // Set DAC resolution
      m_interface(INTERFACE_NONE),                // No interface initialized yet
      m_spi_library(nullptr),                     // SPI library not set
      m_pin_cs(-1),                               // Invalid CS pin
      m_i2c_library(nullptr),                     // I2C library not set
      m_i2c_address(0),                           // Invalid I2C address
      m_reference_voltage(2.5f),                  // Default internal reference
      m_gain(DACX0501_GAIN_MODE_2X),              // Default 2x gain
      m_reference_divider(DACX0501_DIVIDER_1X) {  // No reference divider
}

/**
 * @brief Initialize the DAC using SPI communication
 *
 * Sets up SPI communication with the DAC and performs initial configuration.
 * This method configures the SPI settings, performs a soft reset, and waits
 * for the device to be ready before returning.
 *
 * @param[in] spi_library SPI library instance (usually SPI)
 * @param[in] spi_speed SPI clock speed in Hz (1 Hz to 50 MHz)
 * @param[in] pin_cs Chip select pin number (must be valid digital pin)
 * @return 0 on success, negative error code on failure
 *
 * @note This method performs the following operations:
 *       1. Validates input parameters
 *       2. Configures SPI settings (MSB first, Mode 0)
 *       3. Sets up chip select pin
 *       4. Performs soft reset (trigger register = 0b1010)
 *       5. Waits for device to respond (up to 500ms timeout)
 *       6. Sets default configuration values
 *
 * @warning Make sure to call pinMode(pin_cs, OUTPUT) before calling this method
 */
int dacx0501::begin(SPIClass& spi_library, const int spi_speed, const int pin_cs) {
    int res;

    /* Validate SPI speed parameter */
    if ((spi_speed <= 0) || (spi_speed > 50000000)) {
        return -EINVAL;  // Invalid SPI speed (must be 1 Hz to 50 MHz)
    }

    /* Validate chip select pin parameter */
    if (pin_cs < 0) {
        return -EINVAL;  // Invalid CS pin (must be non-negative)
    }

    /* Configure SPI interface settings */
    m_spi_library = &spi_library;
    m_spi_settings = SPISettings(spi_speed, MSBFIRST, SPI_MODE0);  // MSB first, Mode 0
    m_pin_cs = pin_cs;
    m_interface = INTERFACE_SPI;

    /* Configure CS pin as output and set high (inactive) */
    pinMode(m_pin_cs, OUTPUT);
    digitalWrite(m_pin_cs, HIGH);

    /* Perform soft reset to ensure clean state */
    uint16_t reg_trigger = 0b1010;  // Soft reset command
    res = register_write(DACX0501_REGISTER_TRIGGER, reg_trigger);
    if (res != 0) {
        return res;  // Failed to perform soft reset
    }

    /* Wait for device to be ready after soft reset (with timeout) */
    uint32_t time = millis();
    while (true) {
        if (detect()) {
            break;  // Device is responding
        } else if ((millis() - time) > 500) {
            return -ETIMEDOUT;  // Device not responding after 500ms
        } else {
            delay(10);  // Wait 10ms before trying again
        }
    }

    /* Set default configuration values */
    m_gain = DACX0501_GAIN_MODE_2X;             // 2x gain for 0-5V output range
    m_reference_divider = DACX0501_DIVIDER_1X;  // No reference divider
    m_reference_voltage = 2.5f;                 // Internal 2.5V reference

    /* Return success */
    return 0;
}

/**
 * @brief Initialize the DAC using I2C communication
 *
 * Sets up I2C communication with the DAC and performs initial configuration.
 * This method validates the I2C address, performs a soft reset, and waits
 * for the device to be ready before returning.
 *
 * @param[in] i2c_library I2C library instance (usually Wire)
 * @param[in] i2c_address I2C device address (0x48 to 0x4B)
 * @return 0 on success, negative error code on failure
 *
 * @note This method performs the following operations:
 *       1. Validates I2C address (must be 0x48-0x4B)
 *       2. Configures I2C interface
 *       3. Performs soft reset (trigger register = 0b1010)
 *       4. Waits for device to respond (up to 500ms timeout)
 *       5. Sets default configuration values
 *
 * @note I2C Address Configuration:
 *       - 0x48: A0=GND
 *       - 0x49: A0=VDD
 *       - 0x4A: A0=SDA
 *       - 0x4B: A0=SCL
 *
 * @warning Make sure to call Wire.begin() before calling this method
 */
int dacx0501::begin(TwoWire& i2c_library, const uint8_t i2c_address) {
    int res;

    /* Validate I2C address parameter */
    if ((i2c_address < 0x48) || (i2c_address > 0x4B)) {
        return -EINVAL;  // Invalid I2C address (must be 0x48-0x4B)
    }

    /* Configure I2C interface settings */
    m_i2c_library = &i2c_library;
    m_i2c_address = i2c_address;
    m_interface = INTERFACE_I2C;

    /* Perform soft reset to ensure clean state */
    uint16_t reg_trigger = 0b1010;  // Soft reset command
    res = register_write(DACX0501_REGISTER_TRIGGER, reg_trigger);
    if (res != 0) {
        return res;  // Failed to perform soft reset
    }

    /* Wait for device to be ready after soft reset (with timeout) */
    uint32_t time = millis();
    while (true) {
        if (detect()) {
            break;  // Device is responding
        } else if ((millis() - time) > 500) {
            return -ETIMEDOUT;  // Device not responding after 500ms
        } else {
            delay(10);  // Wait 10ms before trying again
        }
    }

    /* Set default configuration values */
    m_gain = DACX0501_GAIN_MODE_2X;             // 2x gain for 0-5V output range
    m_reference_divider = DACX0501_DIVIDER_1X;  // No reference divider
    m_reference_voltage = 2.5f;                 // Internal 2.5V reference

    /* Return success */
    return 0;
}

/**
 * @brief Detect if the DAC device is present and responding
 *
 * This method attempts to communicate with the DAC by reading the device ID
 * register and checking for the expected signature. This is useful for
 * verifying that the device is properly connected and responding.
 *
 * @return true if device is detected and responding, false otherwise
 *
 * @note The device signature is 0x0115 (masked with 0x0F7F to ignore
 *       reserved bits). This signature is common to all DACx0501 variants.
 */
bool dacx0501::detect(void) {
    int res;

    /* Read device ID register to get device signature */
    uint16_t reg_devid;
    res = register_read(DACX0501_REGISTER_DEVID, reg_devid);
    if (res < 0) {
        return false;  // Failed to read device ID register
    }

    /* Check if device signature matches expected value */
    uint16_t signature = reg_devid & 0x0F7F;  // Mask reserved bits
    if (signature == 0x0115) {
        return true;  // Device detected successfully
    } else {
        return false;  // Device signature doesn't match
    }
}

/**
 * Configure the DAC to use the internal reference voltage
 * @return 0 on success, negative error code on failure
 */
int dacx0501::reference_internal_set(void) {
    int res;

    /* Clear REF_PWDWN bit in the config register */
    uint16_t reg_config = 0;
    res = register_read(DACX0501_REGISTER_CONFIG, reg_config);
    if (res != 0) {
        return -EIO;
    }
    reg_config &= ~(1 << 8);
    res = register_write(DACX0501_REGISTER_CONFIG, reg_config);
    if (res != 0) {
        return -EIO;
    }

    /* Save reference voltage */
    m_reference_voltage = 2.5f;

    /* Return success */
    return 0;
}

/**
 * Configure the DAC to use an external reference voltage
 * @return 0 on success, negative error code on failure
 */
int dacx0501::reference_external_set(const float voltage) {
    int res;

    /* Ensure voltage is valid */
    if (voltage <= 0) {
        return -EINVAL;
    }

    /* Set REF_PWDWN bit in the config register */
    uint16_t reg_config = 0;
    res = register_read(DACX0501_REGISTER_CONFIG, reg_config);
    if (res != 0) {
        return -EIO;
    }
    reg_config |= (1 << 8);
    res = register_write(DACX0501_REGISTER_CONFIG, reg_config);
    if (res != 0) {
        return -EIO;
    }

    /* Save reference voltage */
    m_reference_voltage = voltage;

    /* Return success */
    return 0;
}

/**
 * Set reference divider mode
 * @param[in] mode Reference divider mode (DACX0501_DIVIDER_1X or DACX0501_DIVIDER_2X)
 * @return 0 on success, negative error code on failure
 */
int dacx0501::reference_divider_set(const enum dacx0501_reference_divider mode) {
    int res;

    /* Update REF-DIV bit in the gain register */
    uint16_t reg_gain = 0;
    res = register_read(DACX0501_REGISTER_GAIN, reg_gain);
    if (res != 0) {
        return -EIO;
    }
    reg_gain &= ~(1 << 8);
    reg_gain |= (mode << 8);
    res = register_write(DACX0501_REGISTER_GAIN, reg_gain);
    if (res != 0) {
        return -EIO;
    }

    /* Save reference divider mode */
    m_reference_divider = mode;

    /* Return success */
    return 0;
}

/**
 * Set gain mode
 * @param[in] mode Gain mode (DACX0501_GAIN_MODE_1X or DACX0501_GAIN_MODE_2X)
 * @return 0 on success, negative error code on failure
 */
int dacx0501::gain_set(const enum dacx0501_gain_mode mode) {
    int res;

    /* Update BUFF-GAIN bit in the gain register */
    uint16_t reg_gain = 0;
    res = register_read(DACX0501_REGISTER_GAIN, reg_gain);
    if (res != 0) {
        return -EIO;
    }
    reg_gain &= ~(1 << 0);
    reg_gain |= (mode << 0);
    res = register_write(DACX0501_REGISTER_GAIN, reg_gain);
    if (res != 0) {
        return -EIO;
    }

    /* Save gain mode */
    m_gain = mode;

    /* Return success */
    return 0;
}

/**
 * Set power down mode
 * @param[in] mode Power down mode
 * @return 0 on success, negative error code on failure
 */
int dacx0501::powerdown_mode_set(const enum dacx0501_powerdown_mode mode) {
    int res;

    /* Update DAC_PWDWN bit in the config register */
    uint16_t reg_config = 0;
    res = register_read(DACX0501_REGISTER_CONFIG, reg_config);
    if (res != 0) {
        return -EIO;
    }
    reg_config &= ~(1 << 0);
    reg_config |= (mode << 0);
    res = register_write(DACX0501_REGISTER_CONFIG, reg_config);
    if (res != 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * Set output ratio
 * @param[in] ratio Output ratio (0.0 to 1.0)
 * @return 0 on success, negative error code on failure
 */
int dacx0501::output_ratio_set(const float ratio) {

    /* Ensure ratio is in range */
    if ((ratio < 0.0f) || (ratio > 1.0f)) {
        return -EINVAL;
    }

    /* Convert ratio to DAC code */
    uint32_t max_code = (1 << m_bits) - 1;
    uint32_t code = ratio * max_code;

    /* Write DAC code to the DATA register */
    return output_code_set(code);
}

/**
 * Set output voltage
 * @param[in] voltage Output voltage in volts
 * @return 0 on success, negative error code on failure
 */
int dacx0501::output_voltage_set(const float voltage) {

    /* Calculate maximum voltage based on gain setting */
    float max_voltage = m_reference_voltage;
    if (m_reference_divider == DACX0501_DIVIDER_2X) {
        max_voltage /= 2.0f;
    }
    if (m_gain == DACX0501_GAIN_MODE_2X) {
        max_voltage *= 2.0f;
    }

    /* Ensure voltage is in range */
    if ((voltage < 0.0f) || (voltage > max_voltage)) {
        return -EINVAL;
    }

    /* Convert voltage to code */
    uint32_t max_code = (1 << m_bits) - 1;
    uint32_t code = (uint32_t)((voltage / max_voltage) * max_code);

    /* Write DAC code to the DATA register */
    return output_code_set(code);
}

/**
 * Set output code
 * @param[in] code DAC code value
 * @return 0 on success, negative error code on failure
 */
int dacx0501::output_code_set(const uint16_t code) {
    int res;

    /* Ensure code is in range */
    uint32_t max_code = (1 << m_bits) - 1;
    if (code > max_code) {
        return -EINVAL;
    }

    /* Left shift code to align with 16-bit register format */
    /* The DATA register is 16 bits, but different DACs use different bit positions:
     * - DAC60501 (12-bit): shift left by 4 bits (bits 15:4)
     * - DAC70501 (14-bit): shift left by 2 bits (bits 15:2)
     * - DAC80501 (16-bit): no shift needed (bits 15:0)
     */
    uint16_t shifted_code = code << (16 - m_bits);

    /* Write DAC code to the DATA register */
    res = register_write(DACX0501_REGISTER_DATA, shifted_code);
    if (res != 0) {
        return -EIO;
    }

    /* Return success */
    return 0;
}

/**
 * Write register (handles both SPI and I2C)
 * @param[in] reg Register address
 * @param[in] data Data to write
 * @return 0 on success, negative error code on failure
 */
int dacx0501::register_write(const uint8_t reg, const uint16_t data) {
    switch (m_interface) {

        case INTERFACE_SPI: {
            /* SPI write */
            digitalWrite(m_pin_cs, LOW);
            m_spi_library->beginTransaction(m_spi_settings);

            /* Send register address */
            m_spi_library->transfer(reg);

            /* Send data based on resolution */
            m_spi_library->transfer16((uint16_t)(data & 0xFFFF));

            m_spi_library->endTransaction();
            digitalWrite(m_pin_cs, HIGH);
            break;
        }

        case INTERFACE_I2C: {

            /* Send register address */
            m_i2c_library->beginTransmission(m_i2c_address);
            m_i2c_library->write(reg);

            /* Send register contents */
            m_i2c_library->write((uint8_t)((data >> 8) & 0xFF));
            m_i2c_library->write((uint8_t)(data & 0xFF));
            if (m_i2c_library->endTransmission(true) != 0) {
                return -EIO;
            }
            break;
        }

        default: {
            return -EINVAL;
        }
    }

    /* Return success */
    return 0;
}

/**
 * Read register (handles both SPI and I2C)
 * @param[in] reg Register address
 * @param[out] data Data read from register
 * @return 0 on success, negative error code on failure
 */
int dacx0501::register_read(const uint8_t reg, uint16_t& data) {
    switch (m_interface) {

        case INTERFACE_SPI: {
            /* SPI read */
            digitalWrite(m_pin_cs, LOW);
            m_spi_library->beginTransaction(m_spi_settings);

            /* Send register address */
            m_spi_library->transfer(reg);

            /* Read data based on resolution */
            data = m_spi_library->transfer16(0x0000);

            m_spi_library->endTransaction();
            digitalWrite(m_pin_cs, HIGH);
            break;
        }

        case INTERFACE_I2C: {

            /* Send register address */
            m_i2c_library->beginTransmission(m_i2c_address);
            m_i2c_library->write(reg);
            if (m_i2c_library->endTransmission(false) != 0) {
                return -EIO;
            }

            /* Read register contents */
            m_i2c_library->requestFrom(m_i2c_address, 2, true);
            if (m_i2c_library->available() != 2) {
                return -EIO;
            }

            /* Read data */
            data = 0;
            data |= m_i2c_library->read();
            data <<= 8;
            data |= m_i2c_library->read();
            break;
        }

        default: {
            return -EINVAL;
        }
    }

    /* Return success */
    return 0;
}
