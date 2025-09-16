/* Self header */
#include "dacx0501.h"

/**
 * Constructor
 * @param[in] bits Resolution in bits (12, 14, or 16)
 */
dacx0501::dacx0501(const uint8_t bits)
    : m_bits(bits),
      m_interface(INTERFACE_NONE),
      m_spi_library(nullptr),
      m_pin_cs(-1),
      m_i2c_library(nullptr),
      m_i2c_address(0),
      m_reference_voltage(2.5f),
      m_gain(DACX0501_GAIN_MODE_2X),
      m_reference_divider(DACX0501_DIVIDER_1X) {
}

/**
 * Initialize SPI interface
 * @param[in] spi_library SPI library instance
 * @param[in] spi_speed SPI speed in Hz
 * @param[in] pin_cs Chip select pin
 * @return 0 on success, negative error code on failure
 */
int dacx0501::begin(SPIClass& spi_library, const int spi_speed, const int pin_cs) {
    int res;

    /* Ensure SPI speed is valid */
    if ((spi_speed <= 0) || (spi_speed > 50000000)) {
        return -EINVAL;
    }

    /* Ensure chip select pin is valid */
    if (pin_cs < 0) {
        return -EINVAL;
    }

    /* Setup SPI */
    m_spi_library = &spi_library;
    m_spi_settings = SPISettings(spi_speed, MSBFIRST, SPI_MODE0);
    m_pin_cs = pin_cs;
    m_interface = INTERFACE_SPI;

    /* Perform a soft reset */
    uint16_t reg_trigger = 0b1010;
    res = register_write(DACX0501_REGISTER_TRIGGER, reg_trigger);
    if (res != 0) {
        return res;
    }

    /* Wait for device to be ready after soft reset */
    uint32_t time = millis();
    while (true) {
        if (detect()) {
            break;
        } else if ((millis() - time) > 500) {
            return -ETIMEDOUT;
        } else {
            delay(10);
        }
    }

    /* Set default configuration */
    m_gain = DACX0501_GAIN_MODE_2X;
    m_reference_divider = DACX0501_DIVIDER_1X;
    m_reference_voltage = 2.5f;

    /* Return success */
    return 0;
}

/**
 * Initialize I2C interface
 * @param[in] i2c_library I2C library instance
 * @param[in] i2c_address I2C address
 * @return 0 on success, negative error code on failure
 */
int dacx0501::begin(TwoWire& i2c_library, const uint8_t i2c_address) {
    int res;

    /* Ensure i2c address is valid */
    if ((i2c_address < 0x48) || (i2c_address > 0x4B)) {
        return -EINVAL;
    }

    /* Setup I2C */
    m_i2c_library = &i2c_library;
    m_i2c_address = i2c_address;
    m_interface = INTERFACE_I2C;

    /* Perform a soft reset */
    uint16_t reg_trigger = 0b1010;
    res = register_write(DACX0501_REGISTER_TRIGGER, reg_trigger);
    if (res != 0) {
        return res;
    }

    /* Wait for device to be ready after soft reset */
    uint32_t time = millis();
    while (true) {
        if (detect()) {
            break;
        } else if ((millis() - time) > 500) {
            return -ETIMEDOUT;
        } else {
            delay(10);
        }
    }

    /* Set default configuration */
    m_gain = DACX0501_GAIN_MODE_2X;
    m_reference_divider = DACX0501_DIVIDER_1X;
    m_reference_voltage = 2.5f;

    /* Return success */
    return 0;
}

/**
 * Tries to detect the device.
 * @return true if the device has been detected, or false otherwise.
 */
bool dacx0501::detect(void) {
    int res;

    /* Read device id register */
    uint16_t reg_devid;
    res = register_read(DACX0501_REGISTER_DEVID, reg_devid);
    if (res < 0) {
        return false;
    }

    /* Return whether or not the expected value matches */
    uint16_t signature = reg_devid & 0x0F7F;
    if (signature == 0x0115) {
        return true;
    } else {
        return false;
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
