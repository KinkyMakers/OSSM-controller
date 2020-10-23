#include "ard_crc.h"

#include <stdbool.h>
#include <string.h>

#include "ard_crc_kermit.h"

bool ard_crc16_kermit_test(const uint8_t *restrict data, const size_t len)
{
    crc_t crc;
    crc = crc_init();
    crc = crc_update(crc, data, len + sizeof(crc_t));
    crc = crc_finalize(crc);

    return (crc == 0x00);
}

uint16_t ard_crc16_kermit_append(uint8_t *restrict data, const size_t len)
{
    crc_t crc;
    crc = crc_init();
    crc = crc_update(crc, data, len);
    crc = crc_finalize(crc);

    // append CRC
    memcpy(&data[len], &crc, sizeof(crc_t));

    return crc;
}
