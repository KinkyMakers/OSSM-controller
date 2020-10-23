
#include "ard_i2c.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef I2C_BUFFER_LENGTH
#ifdef BUFFER_LENGTH
#define I2C_BUFFER_LENGTH BUFFER_LENGTH
#else
#define I2C_BUFFER_LENGTH 64
#endif
#endif

// TODO: i2c slave
//
// void ard_i2c_slave_setup(const uint8_t slave_address, ArdI2CReqHandler request_handler,
//                          ArdI2CRecHandler receive_handler)
// {
//     Wire.onRequest(request_handler);
//     Wire.onReceive(receive_handler);
//     Wire.begin(slave_address);
// }

// // master write to slave
// size_t ard_i2c_slave_write(uint8_t *restrict data, const size_t size)
// {
//     if (size <= I2C_BUFFER_LENGTH)
//     {
//         return Wire.write(data, (uint8_t)size);
//     }
//     else
//     {
//         // need multiple calls to send all the data
//         size_t remaining = size;
//         size_t retval;
//         while (remaining > I2C_BUFFER_LENGTH)
//         {
//             retval = Wire.write(&data[size - remaining], I2C_BUFFER_LENGTH);
//             if (retval != I2C_BUFFER_LENGTH)
//             {
//                 return size - remaining + retval;
//             };
//             remaining = remaining - I2C_BUFFER_LENGTH;
//         }
//         // last call
//         retval = Wire.write(&data[size - remaining], (uint8_t)remaining);
//         if (retval != remaining)
//         {
//             return size - remaining + retval;
//         }
//         else
//         {
//             return size;
//         }
//     }
// }

void ard_i2c_master_begin()
{
#ifndef __AVR__
    Wire.begin(ARD_I2C_SDA_PIN, ARD_I2C_SCL_PIN);
#else
    Wire.begin();
#endif
    // fast mode 400kHz
    Wire.setClock(400000U);
}

uint8_t ard_i2c_master_write_register(const uint8_t device_address, const uint8_t write_register,
                                      const uint8_t register_value)
{
    // Writes value at i2c register
    Wire.beginTransmission(device_address);
    Wire.write(write_register);
    Wire.write(register_value);
    return Wire.endTransmission(true);
}

uint8_t ard_i2c_master_read_begin(const uint8_t device_address, const uint8_t start_register)
{
    // read bytes (size bytes) into out
    Wire.beginTransmission(device_address);
    Wire.write(start_register);
    return Wire.endTransmission(false);
}

uint8_t ard_i2c_master_read_byte(const uint8_t device_address, const uint8_t start_register)
{
    // read bytes (size bytes) into out
    Wire.beginTransmission(device_address);
    Wire.write(start_register);
    if (Wire.endTransmission(false) == 0)
    {
        Wire.requestFrom(device_address, (uint8_t)1, (uint8_t) true);
        return Wire.read();
        // if (Wire.available()) {
        //     return Wire.read();
        // }
    }
    return 0;
}

size_t ard_i2c_master_read(const uint8_t device_address, uint8_t *out, const size_t size)
{
    // read bytes (size bytes) into out
    if (size <= I2C_BUFFER_LENGTH)
    {
        // only one requestFrom call to get all the data
        Wire.requestFrom(device_address, (uint8_t)size, (uint8_t) true);
        for (size_t i = 0; i < size; i++)
        {
            out[i] = Wire.read();
        }
    }
    else
    {
        // need multiple requestFrom calls to get all the data
        size_t remaining = size;
        while (remaining > I2C_BUFFER_LENGTH)
        {
            Wire.requestFrom(device_address, (uint8_t)I2C_BUFFER_LENGTH, (uint8_t) false);
            for (size_t i = 0; i < I2C_BUFFER_LENGTH; i++)
            {
                out[size - remaining + i] = Wire.read();
            }
            remaining = remaining - I2C_BUFFER_LENGTH;
        }
        // last call
        if (remaining > 0)
        {
            Wire.requestFrom(device_address, (uint8_t)remaining, (uint8_t) true);
            for (size_t i = 0; i < remaining; i++)
            {
                out[size - remaining + i] = Wire.read();
            }
        }
    }
    return size;
}

uint8_t ard_i2c_master_end_transmission(void) { return Wire.endTransmission(true); }
