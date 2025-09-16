/**
 * @file dacx0501.h
 * @brief Sitron Labs Arduino Library for Texas Instruments DACx0501 Series
 * @author Sitron Labs
 * @version 0.1.0
 * @date 2024
 *
 * This library provides an easy-to-use interface for the Texas Instruments
 * DACx0501 series of digital-to-analog converters. It supports both SPI and
 * I2C communication interfaces and includes features like configurable gain,
 * power management, and multiple resolution options.
 *
 * Supported Devices:
 * - DAC60501: 12-bit resolution
 * - DAC70501: 14-bit resolution
 * - DAC80501: 16-bit resolution
 *
 * @copyright Copyright (c) 2024 Sitron Labs. All rights reserved.
 * @license This library is released under the MIT License.
 */

#ifndef DACX0501_H
#define DACX0501_H

/* Arduino libraries */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

/* C/C++ libraries */
#include <errno.h>
#include <stdint.h>

/**
 * @brief DACx0501 Register addresses
 *
 * These are the internal register addresses used to communicate with the DACx0501
 * series devices. Each register controls different aspects of the DAC operation.
 */
enum dacx0501_register {
    DACX0501_REGISTER_NOOP = 0x00,     ///< No operation register (used for testing)
    DACX0501_REGISTER_DEVID = 0x01,    ///< Device ID register (contains device signature)
    DACX0501_REGISTER_SYNC = 0x02,     ///< Synchronization register (for multi-device sync)
    DACX0501_REGISTER_CONFIG = 0x03,   ///< Configuration register (power modes, reference)
    DACX0501_REGISTER_GAIN = 0x04,     ///< Gain register (output gain and reference divider)
    DACX0501_REGISTER_TRIGGER = 0x05,  ///< Trigger register (soft reset, LDAC control)
    DACX0501_REGISTER_STATUS = 0x07,   ///< Status register (device status flags)
    DACX0501_REGISTER_DATA = 0x08      ///< Data register (DAC output value)
};

/**
 * @brief Power-down modes for the DAC output
 *
 * These modes control the behavior of the DAC output when the device is in
 * power-down mode. This helps reduce power consumption when the DAC is not needed.
 */
enum dacx0501_powerdown_mode {
    DACX0501_POWERDOWN_MODE_NORMAL = 0,  ///< Normal operation (DAC active)
    DACX0501_POWERDOWN_MODE_1K = 1,      ///< Power-down with 1 kΩ pull-down to GND
};

/**
 * @brief Output gain modes
 *
 * The gain setting affects the maximum output voltage range. With 2x gain,
 * the output can reach up to 2x the reference voltage (when using internal reference).
 */
enum dacx0501_gain_mode {
    DACX0501_GAIN_MODE_1X = 0,  ///< 1x gain (output = 0V to Vref)
    DACX0501_GAIN_MODE_2X = 1   ///< 2x gain (output = 0V to 2×Vref)
};

/**
 * @brief Reference voltage divider modes
 *
 * The reference divider allows you to effectively halve the reference voltage,
 * which can be useful for lower voltage applications or when using higher
 * external reference voltages.
 */
enum dacx0501_reference_divider {
    DACX0501_DIVIDER_1X = 0,  ///< Reference divider disabled (use full reference)
    DACX0501_DIVIDER_2X = 1   ///< Reference divider enabled (divide reference by 2)
};

/**
 * @brief DACx0501 Base Class
 *
 * This is the main class for interfacing with Texas Instruments DACx0501 series
 * digital-to-analog converters. It provides a unified interface for all three
 * variants (DAC60501, DAC70501, DAC80501) with support for both SPI and I2C
 * communication protocols.
 *
 * Key Features:
 * - Single-channel DAC output
 * - SPI and I2C communication support
 * - Configurable gain (1x or 2x)
 * - Internal or external reference voltage
 * - Reference voltage divider option
 * - Power-down modes for low power operation
 * - Multiple output methods (voltage, ratio, raw code)
 *
 * @note This is a base class. Use the specific device classes (dac60501,
 *       dac70501, dac80501) for your application.
 *
 * @example Basic usage with SPI:
 * @code
 * #include "dac80501.h"
 *
 * dac80501 dac;
 *
 * void setup() {
 *   dac.begin(SPI, 1000000, 10);  // SPI, 1MHz, CS pin 10
 *   dac.reference_internal_set();
 *   dac.output_voltage_set(1.5);  // Set output to 1.5V
 * }
 * @endcode
 *
 * @example Basic usage with I2C:
 * @code
 * #include "dac60501.h"
 *
 * dac60501 dac;
 *
 * void setup() {
 *   dac.begin(Wire, 0x48);        // I2C, address 0x48
 *   dac.reference_internal_set();
 *   dac.output_ratio_set(0.5);    // Set output to 50% of max
 * }
 * @endcode
 */
class dacx0501 {
   public:
    /**
     * @brief Constructor
     * @param[in] bits DAC resolution in bits (12, 14, or 16)
     * @note This constructor is called by the specific device classes
     */
    dacx0501(const uint8_t bits);

    /* ===== Setup Methods ===== */

    /**
     * @brief Initialize the DAC using SPI communication
     * @param[in] spi_library SPI library instance (usually SPI)
     * @param[in] spi_speed SPI clock speed in Hz (max 50 MHz)
     * @param[in] pin_cs Chip select pin number
     * @return 0 on success, negative error code on failure
     * @note Performs a soft reset and sets default configuration
     */
    int begin(SPIClass& spi_library, const int spi_speed, const int pin_cs);

    /**
     * @brief Initialize the DAC using I2C communication
     * @param[in] i2c_library I2C library instance (usually Wire)
     * @param[in] i2c_address I2C address (0x48 to 0x4B)
     * @return 0 on success, negative error code on failure
     * @note Performs a soft reset and sets default configuration
     */
    int begin(TwoWire& i2c_library, const uint8_t i2c_address);

    /**
     * @brief Detect if the DAC is present and responding
     * @return true if device detected, false otherwise
     * @note Reads the device ID register to verify communication
     */
    bool detect(void);

    /* ===== Configuration Methods ===== */

    /**
     * @brief Configure DAC to use internal 2.5V reference
     * @return 0 on success, negative error code on failure
     * @note This is the default reference mode
     */
    int reference_internal_set(void);

    /**
     * @brief Configure DAC to use external reference voltage
     * @param[in] voltage External reference voltage in volts
     * @return 0 on success, negative error code on failure
     * @note External reference must be connected to VREF pin
     */
    int reference_external_set(const float voltage);

    /**
     * @brief Set reference voltage divider mode
     * @param[in] mode Divider mode (DACX0501_DIVIDER_1X or DACX0501_DIVIDER_2X)
     * @return 0 on success, negative error code on failure
     * @note 2X divider halves the effective reference voltage
     */
    int reference_divider_set(const enum dacx0501_reference_divider mode);

    /**
     * @brief Set output gain mode
     * @param[in] mode Gain mode (DACX0501_GAIN_MODE_1X or DACX0501_GAIN_MODE_2X)
     * @return 0 on success, negative error code on failure
     * @note 2X gain doubles the maximum output voltage
     */
    int gain_set(const enum dacx0501_gain_mode mode);

    /**
     * @brief Set power-down mode
     * @param[in] mode Power-down mode
     * @return 0 on success, negative error code on failure
     * @note Power-down reduces current consumption when DAC is not needed
     */
    int powerdown_mode_set(const enum dacx0501_powerdown_mode mode);

    /* ===== Output Methods ===== */

    /**
     * @brief Set output as a ratio of maximum voltage
     * @param[in] ratio Output ratio (0.0 to 1.0)
     * @return 0 on success, negative error code on failure
     * @note 0.0 = 0V, 1.0 = maximum voltage (Vref × gain)
     */
    int output_ratio_set(const float ratio);

    /**
     * @brief Set output voltage directly
     * @param[in] voltage Desired output voltage in volts
     * @return 0 on success, negative error code on failure
     * @note Voltage is automatically limited to valid range
     */
    int output_voltage_set(const float voltage);

    /**
     * @brief Set output using raw DAC code
     * @param[in] code Raw DAC code value (0 to 2^bits-1)
     * @return 0 on success, negative error code on failure
     * @note This is the most direct method for advanced users
     */
    int output_code_set(const uint16_t code);

    /* ===== Utility Methods ===== */

    /**
     * @brief Get DAC resolution in bits
     * @return Resolution in bits (12, 14, or 16)
     */
    uint8_t resolution_get() const { return m_bits; }

    /* ===== Low-Level Register Methods ===== */

    /**
     * @brief Write data to a DAC register
     * @param[in] reg Register address
     * @param[in] data Data to write
     * @return 0 on success, negative error code on failure
     * @note For advanced users who need direct register access
     */
    int register_write(const uint8_t reg, const uint16_t data);

    /**
     * @brief Read data from a DAC register
     * @param[in] reg Register address
     * @param[out] data Data read from register
     * @return 0 on success, negative error code on failure
     * @note For advanced users who need direct register access
     */
    int register_read(const uint8_t reg, uint16_t& data);

   protected:
    /* ===== Member Variables ===== */

    uint8_t m_bits;  ///< DAC resolution in bits (12, 14, or 16)

    /**
     * @brief Communication interface type
     */
    enum interface_type {
        INTERFACE_NONE,  ///< No interface initialized
        INTERFACE_SPI,   ///< SPI interface active
        INTERFACE_I2C,   ///< I2C interface active
    } m_interface;

    /* SPI-related members */
    SPIClass* m_spi_library;     ///< Pointer to SPI library instance
    SPISettings m_spi_settings;  ///< SPI communication settings
    int m_pin_cs;                ///< Chip select pin number

    /* I2C-related members */
    TwoWire* m_i2c_library;  ///< Pointer to I2C library instance
    uint8_t m_i2c_address;   ///< I2C device address

    /* Configuration state */
    float m_reference_voltage;                            ///< Current reference voltage setting
    enum dacx0501_gain_mode m_gain;                       ///< Current gain mode setting
    enum dacx0501_reference_divider m_reference_divider;  ///< Current reference divider setting
};

#endif
