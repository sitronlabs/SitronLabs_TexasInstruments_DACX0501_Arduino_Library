/**
 * @file dac60501.h
 * @brief Sitron Labs Arduino Library for Texas Instruments DAC60501
 * @author Sitron Labs
 * @version 0.1.0
 * @date 2024
 *
 * This header defines the DAC60501 class, which provides an easy-to-use
 * interface for the Texas Instruments DAC60501 12-bit digital-to-analog
 * converter. This class inherits from the dacx0501 base class and sets
 * the appropriate resolution for the 12-bit DAC.
 *
 * @copyright Copyright (c) 2024 Sitron Labs. All rights reserved.
 * @license This library is released under the MIT License.
 */

#ifndef DAC60501_H
#define DAC60501_H

/* Library headers */
#include "dacx0501.h"

/**
 * @brief DAC60501 Class - 12-bit Digital-to-Analog Converter
 *
 * This class provides a specialized interface for the Texas Instruments
 * DAC60501, a 12-bit single-channel DAC with SPI and I2C interfaces.
 *
 * Key Features:
 * - 12-bit resolution (4096 steps)
 * - Single-channel output
 * - SPI and I2C communication
 * - Internal 2.5V reference or external reference
 * - Configurable gain (1x or 2x)
 * - Power-down modes
 *
 * @note This class inherits all functionality from dacx0501 base class
 *       and automatically sets the correct 12-bit resolution.
 *
 * @example Basic usage:
 * @code
 * #include "dac60501.h"
 *
 * dac60501 dac;
 *
 * void setup() {
 *   dac.begin(SPI, 1000000, 10);  // SPI, 1MHz, CS pin 10
 *   dac.reference_internal_set();
 *   dac.output_voltage_set(1.25); // Set output to 1.25V
 * }
 * @endcode
 */
class dac60501 : public dacx0501 {
   public:
    /**
     * @brief Constructor for DAC60501
     *
     * Initializes the DAC60501 with 12-bit resolution. All other
     * configuration is handled by the base class constructor.
     *
     * @note The DAC must be initialized with begin() before use.
     */
    dac60501() : dacx0501(12) {}
};

#endif
