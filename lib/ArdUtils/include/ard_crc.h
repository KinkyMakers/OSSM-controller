/**
 * @file       ard_crc.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      16-bit Cyclic Redundancy Check (crc16-ccitt)
 *
 * @details
 *
 * See group @ref ArdCrc
 *
 */

#ifndef ARD_CRC_H
#define ARD_CRC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus

#ifndef restrict
#define restrict __restrict
#endif

extern "C"
{
#endif

/*
 * http://automationwiki.com/index.php?title=CRC-16-CCITT
 * KERMIT (CRC-16/CCITT)
 * width=16 poly=0x1021 init=0x0000 refin=true refout=true xorout=0x0000
 * check=0x2189 name="KERMIT" CRC-CCITT    0x1021    
 * x^16 + x^12 + x^5 + 1
 * 
 * https://reveng.sourceforge.io/crc-catalogue/16.htm#crc.cat-bits.16
 * CRC-16/KERMIT
 * width=16 poly=0x1021 init=0x0000 refin=true refout=true xorout=0x0000 check=0x2189 residue=0x0000 name="CRC-16/KERMIT"
 * 
 * https://pycrc.org/models.html#kermit
 * 
 */

/**
 * @defgroup   ArdCrc Cyclic Redundancy Check
 * @brief      C wrapper for I2C Communications.
 * 
 * Code was generated using pycrc:
 * 
 * ```python
 * python -m pycrc --model kermit --algorithm table-driven --generate h -o ard_crc_kermit.h
 * python -m pycrc --model kermit --algorithm table-driven --generate c -o ard_crc_kermit.c
 * ```
 *
 * @{
 *
 */

    /**
     * @brief Check if CRC is verified
     *
     * @param data Buffer
     * @param len Lnegth of data buffer
     * @return @c true if CRC is verified
     * @return @c false if CRC is not verified
     */
    bool ard_crc16_kermit_test(const uint8_t *restrict data, const size_t len);

    /**
     * @brief Calculate and append CRC to buffer
     *
     * There MUST be at least 2 contiguous free bytes allocated at the end of the
     * array
     *
     * @param data Buffer
     * @param len Length of data buffer, not including trailing 2 bytes available
     * for CRC
     * @return uint16_t CRC value
     */
    uint16_t ard_crc16_kermit_append(uint8_t *restrict data, const size_t len);

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
