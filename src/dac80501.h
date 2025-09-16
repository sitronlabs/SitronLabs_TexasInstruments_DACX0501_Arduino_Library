/**
 * @file dac80501.h
 * @brief Sitron Labs Arduino Library for Texas Instruments DAC80501
 * @author Sitron Labs
 * @version 0.1.0
 * @date 2024
 *
 * This header defines the DAC80501 class, which provides an easy-to-use
 * interface for the Texas Instruments DAC80501 16-bit digital-to-analog
 * converter. This class inherits from the dacx0501 base class and sets
 * the appropriate resolution for the 16-bit DAC.
 *
 * @copyright Copyright (c) 2024 Sitron Labs. All rights reserved.
 * @license This library is released under the MIT License.
 */

#ifndef DAC80501_H
#define DAC80501_H

/* Library headers */
#include "dacx0501.h"

/**
 * @brief DAC80501 Class - 16-bit Digital-to-Analog Converter
 *
 * This class provides a specialized interface for the Texas Instruments
 * DAC80501, a 16-bit single-channel DAC with SPI and I2C interfaces.
 *
 * Key Features:
 * - 16-bit resolution (65536 steps)
 * - Single-channel output
 * - SPI and I2C communication
 * - Internal 2.5V reference or external reference
 * - Configurable gain (1x or 2x)
 * - Power-down modes
 *
 * @note This class inherits all functionality from dacx0501 base class
 *       and automatically sets the correct 16-bit resolution.
 *
 * @example Basic usage:
 * @code
 * #include "dac80501.h"
 *
 * dac80501 dac;
 *
 * void setup() {
 *   dac.begin(SPI, 1000000, 10);  // SPI, 1MHz, CS pin 10
 *   dac.reference_internal_set();
 *   dac.output_code_set(32768);   // Set output to mid-scale (50%)
 * }
 * @endcode
 */
class dac80501 : public dacx0501 {
   public:
    /**
     * @brief Constructor for DAC80501
     *
     * Initializes the DAC80501 with 16-bit resolution. All other
     * configuration is handled by the base class constructor.
     *
     * @note The DAC must be initialized with begin() before use.
     */
    dac80501() : dacx0501(16) {}
};

#endif
