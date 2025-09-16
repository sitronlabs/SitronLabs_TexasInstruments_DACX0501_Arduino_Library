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
 */
enum dacx0501_register {
    DACX0501_REGISTER_NOOP = 0x00,
    DACX0501_REGISTER_DEVID = 0x01,
    DACX0501_REGISTER_SYNC = 0x02,
    DACX0501_REGISTER_CONFIG = 0x03,
    DACX0501_REGISTER_GAIN = 0x04,
    DACX0501_REGISTER_TRIGGER = 0x05,
    DACX0501_REGISTER_STATUS = 0x07,
    DACX0501_REGISTER_DATA = 0x08
};

/**
 * @brief Power-down modes
 */
enum dacx0501_powerdown_mode {
    DACX0501_POWERDOWN_MODE_NORMAL = 0,  // Normal operation
    DACX0501_POWERDOWN_MODE_1K = 1,      // 1 kΩ to GND
};

/**
 * @brief Gain values
 */
enum dacx0501_gain_mode {
    DACX0501_GAIN_MODE_1X = 0,  // 1x gain
    DACX0501_GAIN_MODE_2X = 1   // 2x gain
};

/**
 * @brief Reference divider values
 */
enum dacx0501_reference_divider {
    DACX0501_DIVIDER_1X = 0,  // Reference divider disabled
    DACX0501_DIVIDER_2X = 1   // Reference divider enabled (divides by 2)
};

/**
 * @brief DACx0501 Base Class
 * Single-channel DAC supporting both SPI and I2C interfaces
 */
class dacx0501 {
   public:
    /* Constructor */
    dacx0501(const uint8_t bits);

    /* Setup methods */
    int begin(SPIClass& spi_library, const int spi_speed, const int pin_cs);
    int begin(TwoWire& i2c_library, const uint8_t i2c_address);
    bool detect(void);

    /* Configuration */
    int reference_internal_set(void);
    int reference_external_set(const float voltage);
    int reference_divider_set(const enum dacx0501_reference_divider mode);
    int gain_set(const enum dacx0501_gain_mode mode);
    int powerdown_mode_set(const enum dacx0501_powerdown_mode mode);

    /* Output methods - single channel only */
    int output_ratio_set(const float ratio);
    int output_voltage_set(const float voltage);
    int output_code_set(const uint16_t code);

    /* Utility methods */
    uint8_t resolution_get() const { return m_bits; }

    /* Register methods */
    int register_write(const uint8_t reg, const uint16_t data);
    int register_read(const uint8_t reg, uint16_t& data);

   protected:
    /* Member variables */
    uint8_t m_bits;
    enum interface_type {
        INTERFACE_NONE,
        INTERFACE_SPI,
        INTERFACE_I2C,
    } m_interface;
    SPIClass* m_spi_library;
    SPISettings m_spi_settings;
    int m_pin_cs;
    TwoWire* m_i2c_library;
    uint8_t m_i2c_address;
    float m_reference_voltage;
    enum dacx0501_gain_mode m_gain;
    enum dacx0501_reference_divider m_reference_divider;
};

#endif
