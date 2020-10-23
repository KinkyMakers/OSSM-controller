#include "ard_serial.h"

#include <Arduino.h>
#include <stdint.h>

#include "ard_crc.h"

/**
 * @brief      read all requested bytes into buffer
 * @details    There is no check if the requested number of bytes
 *             is greater than the serial buffer and will always fail in this case.
 *
 * @param      buf   byte array
 * @param[in]  len   length of byte array
 *
 * @return     Returns number of bytes read into buffer
 *
 */
static uint16_t ard_serial_read_noblock(uint8_t *buf, const uint16_t len);

/**
 * @brief      read until a specified byte (delimiter)
 * @details    The serial buffer is consumed and read until the desired byte is
 *             consumed, there are no more bytes available to read, or the
 *             maximum number of bytes read is reached
 *
 * @param[in]  delimiter  desired byte to read until
 * @param[in]  max_tries  maximum number of bytes to read
 * @param[out] bytes_read number of bytes read
 *
 * @return     @c true if delimiter found, @ false otherwise
 */
static int ard_serial_read_noblock_until(const uint8_t delimiter, const uint16_t max_tries);

/**
 * @brief      write entire byte array to serial buffer
 * @details    This will only attempt to write if all @c len bytes are available to
 *             write to the serial buffer.
 *
 * @param[in]  buf   byte array to send
 * @param[in]  len   length of byte array
 *
 * @return     returns number of bytes sent or -1 if not enough bytes available to start write
 */
static size_t ard_serial_write_noblock(const uint8_t *buf, uint16_t len);

static void ard_packet_write_data(ArdSerialPacket *serial, uint16_t *data_index,
                                  uint16_t *bytes_remaining);

static int ard_packet_read_data(ArdSerialPacket *serial, uint16_t *data_read_index,
                                uint16_t *bytes_remaining);

/**
 * @brief      allocates memory for serial packet ArdSerialPacket
 * @details    The serial packet byte array packing is as follows
 *
 *             | bytes                   | description                     |
 *             |-------------------------|---------------------------------|
 *             |  0                      | delimiter                       |
 *             |  1 to 2                 | uint16_t number of bytes (size) |
 *             |  3 to (3+size-1)        | data            |
 *             |  (3+size) to (3+size+1) | appended crc                    |
 *
 * @param      serial     serial packet to alocate memory to
 * @param[in]  size       size of data to allocate
 * @param[in]  delimiter  delimiter for readign and writing serlial packets
 *
 * @return     0 on success, -1 on failure to allocate memory
 */
int ard_packet_alloc(ArdSerialPacket *serial, const uint16_t max_size, const uint8_t delimiter,
                     const bool use_crc)
{
    // packet format:
    //  0: delimiter
    //  1 to 2: uint16_t number of bytes (size)
    //  3 to (3+size-1): data
    //  (3+size) to (3+size+1): appended crc
    if (max_size > (UINT16_MAX - 5)) return -1;
    serial->packet = (uint8_t *)calloc(max_size + 5, sizeof(uint8_t));
    if (serial->packet == NULL)
    {
        serial->size = 0;
        serial->max_size = 0;
        serial->data = NULL;
        return -2;
    }

    // alias
    serial->data = &serial->packet[3];
    // initialize
    serial->delimiter = delimiter;
    serial->max_size = max_size;
    serial->crc = use_crc;

    // write info
    serial->index = 0;
    serial->remaining_bytes = 0;
    serial->state.read = EARD_PACKET_READ_DELIMITER;
    serial->state.write = EARD_PACKET_WRITE_IDLE;

    // write packet header
    serial->packet[0] = delimiter;
    // memcpy(&serial->packet[1], &max_size, 2);

    return 0;
}

void ard_packet_free(ArdSerialPacket *serial)
{
    free(serial->packet);
    serial->packet = NULL;
    serial->size = 0;
    serial->max_size = 0;
    serial->data = NULL;
}

uint16_t ard_packet_copy_from_buffer(ArdSerialPacket *serial, const uint8_t *restrict buf,
                                 const uint16_t size)
{
    // copy to data in serial packet
    if (size > serial->max_size) {
        serial->size = 0;
    } else {
        memcpy(serial->data, buf, size);
        serial->size = size;
    }
    return serial->size;
}

uint16_t ard_packet_copy_to_buffer(const ArdSerialPacket *serial, uint8_t *restrict buf,
                                 const uint16_t size)
{
    // copy from serial packet to buffer
    uint16_t copy_size = 0;
    if (size <= serial->size) {
        memcpy(buf, serial->data, serial->size);
        copy_size = serial->size;
    }
    return copy_size;
}

eArdPacketWriteStatus ard_packet_write(ArdSerialPacket *serial)
{
    eArdPacketWriteStatus status = EARD_PACKET_WRITE_NONE;

    // state machine
    if (serial->state.write == EARD_PACKET_WRITE_IDLE)
    {
        if (serial->size == 0)
        {
            // no data to write
            status = EARD_PACKET_WRITE_DATA_EMPTY;
        }
        else
        {
            // set size (little to big endian)
            // serial->packet[1] = (uint8_t)(serial->size >> 8);
            // serial->packet[2] = (uint8_t)(serial->size);
            // little endian copy
            memcpy(&serial->packet[1], &serial->size, sizeof(uint16_t));

            // build packet
            serial->remaining_bytes = serial->size + 3;
            if (serial->crc)
            {
                serial->remaining_bytes += 2;
                ard_crc16_kermit_append(serial->data, serial->size);
            }

            // write index
            serial->index = 0;

            // prepared state
            serial->state.write = EARD_PACKET_WRITE_PREPARED;
            status = EARD_PACKET_WRITE_IN_PROGRESS;
        }
    }
    else if (serial->state.write == EARD_PACKET_WRITE_PREPARED ||
             serial->state.write == EARD_PACKET_WRITE_DATA)
    {
        // write to serial
        ard_packet_write_data(serial, &serial->index, &serial->remaining_bytes);
        if (serial->remaining_bytes > 0)
        {
            // not done
            serial->state.write = EARD_PACKET_WRITE_DATA;
            status = EARD_PACKET_WRITE_IN_PROGRESS;
        }
        else
        {
            // done - success
            serial->state.write = EARD_PACKET_WRITE_DONE;
            status = EARD_PACKET_WRITE_SUCCESS;
        }
    }
    else if (serial->state.write == EARD_PACKET_WRITE_DONE)
    {
        serial->state.write = EARD_PACKET_WRITE_IDLE;
        serial->index = 0;
        serial->size = 0;
        status = EARD_PACKET_WRITE_NONE;
    }
    return status;
}

eArdPacketReadStatus ard_packet_read(ArdSerialPacket *serial)
{
    eArdPacketReadStatus status = EARD_PACKET_READ_NONE;

    // state machine
    if (serial->state.read == EARD_PACKET_READ_DELIMITER)
    {
        int ret = ard_serial_read_noblock_until(serial->delimiter, serial->max_size + 5);
        if (ret > 0)
        {
            // found delimiter
            serial->state.read = EARD_PACKET_READ_SIZE;
            status = EARD_PACKET_READ_IN_PROGRESS;
        } else if (ret == 0) {
            // no bytes available
            status = EARD_PACKET_READ_NONE;
        } else {
            // not delimiter
            status = EARD_PACKET_READ_NO_DELIMITER;
        }
    }
    else if (serial->state.read == EARD_PACKET_READ_SIZE)
    {
        // length of buffer to read
        uint16_t bytes_read = ard_serial_read_noblock(&serial->packet[1], 2);
        if (bytes_read != 2)
        {
            // failed to read
            serial->state.read = EARD_PACKET_READ_DELIMITER;
            status = EARD_PACKET_READ_SIZE_FAILED;
        }
        else
        {
            // big-endian to little endian
            // serial->size = (serial->packet[1] << 8) | (serial->packet[2] >> 8);
            // little endian copy
            memcpy(&serial->size, &serial->packet[1], sizeof(uint16_t));
            if (serial->size > serial->max_size)
            {
                // desired size too large
                serial->size = 0;
                serial->state.read = EARD_PACKET_READ_DELIMITER;
                status = EARD_PACKET_READ_SIZE_FAILED;
            }
            else
            {
                // size ok
                serial->state.read = EARD_PACKET_READ_DATA;
                serial->index = 0;
                serial->remaining_bytes = serial->size;
                if (serial->crc) serial->remaining_bytes += 2;
                status = EARD_PACKET_READ_IN_PROGRESS;
            }
        }
    }
    else if (serial->state.read == EARD_PACKET_READ_DATA)
    {
        ard_packet_read_data(serial, &serial->index, &serial->remaining_bytes);
        if (serial->remaining_bytes > 0)
        {
            // not done
        }
        else if (serial->crc)
        {
            // done read, check crc
            if (ard_crc16_kermit_test(serial->data, serial->size))
            {
                // passed crc
                status = EARD_PACKET_READ_SUCCESS;
                serial->state.read = EARD_PACKET_READ_PREPARED;
            }
            else
            {
                // failed crc
                status = EARD_PACKET_READ_CRC_FAILED;
                serial->size = 0;
                serial->state.read = EARD_PACKET_READ_DELIMITER;
            }
        }
        else
        {
            //  done read, no crc
            status = EARD_PACKET_READ_SUCCESS;
            serial->state.read = EARD_PACKET_READ_PREPARED;
        }
    }
    else if (serial->state.read == EARD_PACKET_READ_PREPARED)
    {
        // done - restart read
        serial->size = 0;
        serial->state.read = EARD_PACKET_READ_DELIMITER;
        status = EARD_PACKET_READ_NONE;
    }

    return status;
}

// arduino wrappers
void ard_serial_begin(const unsigned long baud_rate)
{
    Serial.begin(baud_rate);
    while (!Serial) continue;
}

void ard_serial_write_flush(void) { Serial.flush(); }

void ard_serial_read_flush(void)
{
    const int max_count = Serial.available();
    int count = 0;
    while (Serial.read() != -1 && count < max_count)
    {
        count++;
    }
}

static size_t ard_serial_write_noblock(const uint8_t *restrict buf, uint16_t len)
{
    const uint16_t bytes_available = Serial.availableForWrite();
    const uint16_t bytes_to_write = (len < bytes_available ? len : bytes_available);
    return Serial.write(buf, bytes_to_write);
}

static uint16_t ard_serial_read_noblock(uint8_t *restrict buf, const uint16_t len)
{
    const uint16_t bytes_available = Serial.available();
    const uint16_t bytes_to_read = (len < bytes_available ? len : bytes_available);
    for (uint16_t i = 0; i < bytes_to_read; i++)
    {
        buf[i] = (uint8_t)Serial.read();
    }
    return bytes_to_read;
}

static int ard_serial_read_noblock_until(const uint8_t delimiter, const uint16_t max_tries)
{
    const uint16_t bytes_available = Serial.available();
    const uint16_t bytes_to_read = (max_tries < bytes_available ? max_tries : bytes_available);
    for (uint16_t i = 0; i < bytes_to_read; i++)
    {
        int c = Serial.read();
        if (c == -1)
        {
            return -1;
        }
        else if ((uint8_t)c == delimiter)
        {
            return 1;
        }
    }
    return 0;
}

static void ard_packet_write_data(ArdSerialPacket *serial, uint16_t *data_index,
                                  uint16_t *bytes_remaining)
{
    const uint16_t write_len =
        ard_serial_write_noblock(&serial->packet[(*data_index)], (*bytes_remaining));
    (*data_index) += write_len;
    (*bytes_remaining) -= write_len;
}

static int ard_packet_read_data(ArdSerialPacket *serial, uint16_t *data_read_index,
                                uint16_t *bytes_remaining)
{
    const uint16_t read_len =
        ard_serial_read_noblock(&serial->data[(*data_read_index)], (*bytes_remaining));
    (*data_read_index) += read_len;
    return (*bytes_remaining) - read_len;
}
