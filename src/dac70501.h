/**
 * @file dac70501.h
 * @brief Sitron Labs Arduino Library for Texas Instruments DAC70501
 * @author Sitron Labs
 * @version 0.1.0
 * @date 2024
 *
 * This header defines the DAC70501 class, which provides an easy-to-use
 * interface for the Texas Instruments DAC70501 14-bit digital-to-analog
 * converter. This class inherits from the dacx0501 base class and sets
 * the appropriate resolution for the 14-bit DAC.
 *
 * @copyright Copyright (c) 2024 Sitron Labs. All rights reserved.
 * @license This library is released under the MIT License.
 */

#ifndef DAC70501_H
#define DAC70501_H

/* Library headers */
#include "dacx0501.h"

/**
 * @brief DAC70501 Class - 14-bit Digital-to-Analog Converter
 *
 * This class provides a specialized interface for the Texas Instruments
 * DAC70501, a 14-bit single-channel DAC with SPI and I2C interfaces.
 *
 * Key Features:
 * - 14-bit resolution (16384 steps)
 * - Single-channel output
 * - SPI and I2C communication
 * - Internal 2.5V reference or external reference
 * - Configurable gain (1x or 2x)
 * - Power-down modes
 *
 * @note This class inherits all functionality from dacx0501 base class
 *       and automatically sets the correct 14-bit resolution.
 *
 * @example Basic usage:
 * @code
 * #include "dac70501.h"
 *
 * dac70501 dac;
 *
 * void setup() {
 *   dac.begin(Wire, 0x48);        // I2C, address 0x48
 *   dac.reference_internal_set();
 *   dac.output_ratio_set(0.75);   // Set output to 75% of max
 * }
 * @endcode
 */
class dac70501 : public dacx0501 {
   public:
    /**
     * @brief Constructor for DAC70501
     *
     * Initializes the DAC70501 with 14-bit resolution. All other
     * configuration is handled by the base class constructor.
     *
     * @note The DAC must be initialized with begin() before use.
     */
    dac70501() : dacx0501(14) {}
};

#endif
