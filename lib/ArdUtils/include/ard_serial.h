
/**
 * @file       ard_serial.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      Serial packet protocol 
 *
 * @details
 *
 * See group @ref ArdPacket
 *
 */

#ifndef ARD_SERIAL_H
#define ARD_SERIAL_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define ARD_SERIAL_MAX_WRITE_COUNT 20

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @defgroup   ArdPacket Serial packet protocol 
     * @brief      Read and write serial packets with header and CRC
     *
     * @{
     *
     */

    /**
     * @brief Read packet from serial state machine
     * 
     */
    typedef enum eArdPacketReadState {
        EARD_PACKET_READ_DELIMITER,
        EARD_PACKET_READ_SIZE,
        EARD_PACKET_READ_DATA,
        EARD_PACKET_READ_PREPARED
    } eArdPacketReadState;

    /**
     * @brief Status of read operation from serial
     * 
     */
    typedef enum eArdPacketReadStatus {
        EARD_PACKET_READ_NONE,
        EARD_PACKET_READ_NO_DELIMITER,
        EARD_PACKET_READ_SIZE_FAILED,
        EARD_PACKET_READ_IN_PROGRESS,
        EARD_PACKET_READ_CRC_FAILED,
        EARD_PACKET_READ_SUCCESS
    } eArdPacketReadStatus;

    /**
     * @brief Write packe to serial state machine
     * 
     */
    typedef enum eArdPacketWriteState {
        EARD_PACKET_WRITE_IDLE,
        EARD_PACKET_WRITE_PREPARED,
        EARD_PACKET_WRITE_DATA,
        EARD_PACKET_WRITE_DONE
    } eArdPacketWriteState;

    /**
     * @brief Status of write operation to serial
     * 
     */
    typedef enum eArdPacketWriteStatus {
        EARD_PACKET_WRITE_NONE,
        EARD_PACKET_WRITE_DATA_EMPTY,
        EARD_PACKET_WRITE_IN_PROGRESS,
        EARD_PACKET_WRITE_SUCCESS
    } eArdPacketWriteStatus;

    /**
     * Serial communications buffer container.
     */
    typedef struct ArdSerialPacket
    {
        /**
         * @brief Full data packet including header and crc
         * 
         */
        uint8_t *packet;
        /**
         * @brief Data buffer payload in packet
         * 
         */
        uint8_t *data;
        /**
         * @brief Size of data payload
         * 
         */
        uint16_t size;
        /**
         * @brief Maximum size of data payload
         * 
         */
        uint16_t max_size;
        /**
         * @brief Packet delimiter
         * 
         */
        uint8_t delimiter;
        /**
         * @brief Option to use CRC
         * 
         */
        bool crc;

        /**
         * @brief Internal read/write index
         * 
         */
        uint16_t index;
        /**
         * @brief Internal remaining bytes to read/write
         * 
         */
        uint16_t remaining_bytes;

        union uArdPackectState {
            /**
             * @brief Serial read state
             * 
             */
            eArdPacketReadState read;

            /**
             * @brief Serial write state
             * 
             */
            eArdPacketWriteState write;
        } state;

    } ArdSerialPacket;

    /**
     * @brief      setup Arduino serial communications
     * @details    A wrapper for Arduino C++ functions Serial.begin(baud_rate)
     *
     * @param[in]  baud_rate  baud rate for serial communications
     */
    void ard_serial_begin(const unsigned long baud_rate);

    /**
     * @brief      flush serial write buffer
     */
    void ard_serial_write_flush(void);

    /**
     * @brief      flush serial read buffer
     */
    void ard_serial_read_flush(void);

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
    int ard_packet_alloc(ArdSerialPacket *serial, const uint16_t max_size, const uint8_t delimiter, const bool use_crc);

    /**
     * @brief Free mameory allocated to serial packet
     *
     * @param serial serial packet to free
     */
    void ard_packet_free(ArdSerialPacket *serial);

    /**
     * @brief Copy buffer to packet data
     * 
     * @param serial Serial packet object
     * @param buf buffer to copy to serial object
     * @return number of bytes copied
     */
    uint16_t ard_packet_copy_from_buffer(ArdSerialPacket *serial, const uint8_t *buf, const uint16_t size);

    /**
     * @brief Copy from serial packet to buffer
     * 
     * @param serial Serial packet object
     * @param buf buffer to receive data from 
     * @param size size of buf
     * @return number of bytes copied
     */
    uint16_t ard_packet_copy_to_buffer(const ArdSerialPacket *serial, uint8_t *buf, const uint16_t size);

    /**
     * @brief      { function_description }
     *
     * @param      serial  The serial
     * @param[in]  size    The size
     *
     * @return     if >= 0 it is number of bytes remaining to write, error code otherwise
     */
    eArdPacketWriteStatus ard_packet_write(ArdSerialPacket *serial);

    /**
     * @brief      { function_description }
     *
     * @param      serial     The serial
     * @param[in]  fixed_len  The fixed length
     * @param[out] data_read_index current index of serial->data buffer for next read (if any read
     * bytes remaining)
     *
     * @return     if >= 0 it is number of bytes remaining to read, error code otherwise
     */
    eArdPacketReadStatus ard_packet_read(ArdSerialPacket *serial);

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
