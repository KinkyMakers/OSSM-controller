/**
 * @file       ard_i2c.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      C wrapper for I2C Communications with the Arduino
 *
 * @details
 *
 * See group @ref ArdI2C
 *
 */

#ifndef ARD_I2C_H
#define ARD_I2C_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus

#ifndef restrict
#define restrict __restrict
#endif

#ifndef ARD_I2C_SDA_PIN
#define ARD_I2C_SDA_PIN 21
#endif

#ifndef ARD_I2C_SCL_PIN
#define ARD_I2C_SCL_PIN 22
#endif

extern "C"
{
#endif

    /**
     * @defgroup   ArdI2C I2C Communications.
     * @brief      C wrapper for I2C Communications.
     *
     * @{
     *
     */

    // TODO: i2c slave

    // /** @brief function pointer for handling i2C requests from a master device */
    // typedef void (*ArdI2CReqHandler)(void);

    // /** @brief function pointer for receiving data from an i2C master device */
    // typedef void (*ArdI2CRecHandler)(int);

    // /**
    //  * @brief      Setup as a slave device for I2C communications
    //  *
    //  * @param[in]  slave_address    The slave device address
    //  * @param[in]  request_handler  The request handler
    //  * @param[in]  receive_handler  The receive handler
    //  */
    // void ard_i2c_slave_setup(const uint8_t slave_address, ArdI2CReqHandler request_handler,
    //                          ArdI2CRecHandler receive_handler);

    // /**
    //  * @brief      Write data as a slave to master device
    //  *
    //  * @param      data  Array of data (bytes)
    //  * @param[in]  size  Size of data array (number of bytes)
    //  *
    //  * @return     Length of data sent to master
    //  */
    // size_t ard_i2c_slave_write(uint8_t *restrict data, const size_t size);

    /**
     * @brief Start i2c master
     *
     */
    void ard_i2c_master_begin();

    /**
     * @brief
     *
     * @param device_address
     * @param write_register
     * @param register_value
     * @return uint8_t
     */
    uint8_t ard_i2c_master_write_register(const uint8_t device_address,
                                          const uint8_t write_register,
                                          const uint8_t register_value);

    /**
     * @brief
     *
     * @param device_address
     * @param start_register
     * @return uint8_t
     */
    uint8_t ard_i2c_master_read_begin(const uint8_t device_address, const uint8_t start_register);

    /**
     * @brief
     *
     * @param device_address
     * @param start_register
     * @return uint8_t
     */
    uint8_t ard_i2c_master_read_byte(const uint8_t device_address, const uint8_t start_register);

    /**
     * @brief Read data from i2c device
     *
     * If size greater than @c I2C_BUFFER_LENGTH bytes will be read
     *
     * @param device_address device address
     * @param out output buffer
     * @param size number of bytes to read
     * @return size_t number of bytes read
     */
    size_t ard_i2c_master_read(const uint8_t device_address, uint8_t *out, const size_t size);

    /**
     * @brief
     *
     * @return uint8_t
     */
    uint8_t ard_i2c_master_end_transmission(void);

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
