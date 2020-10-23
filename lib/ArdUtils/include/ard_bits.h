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

#ifndef ARD_BITS_H
#define ARD_BITS_H

// bits

#define BIT_0 0x01
#define BIT_1 0x02
#define BIT_2 0x04
#define BIT_3 0x08
#define BIT_4 0x10
#define BIT_5 0x20
#define BIT_6 0x40
#define BIT_7 0x80

#define BIT_MASK_00000001 0x01
#define BIT_MASK_00000011 0x03
#define BIT_MASK_00000111 0x07
#define BIT_MASK_00001111 0x0F
#define BIT_MASK_00011111 0x1F
#define BIT_MASK_00111111 0x3F
#define BIT_MASK_01111111 0x7F
#define BIT_MASK_11111111 0xFF

#endif
