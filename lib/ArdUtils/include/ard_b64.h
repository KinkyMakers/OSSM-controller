/**
 * @file       ard_b64.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      Base64 encoding and decoding
 *
 * @details
 *
 * See group @ref ArdB64
 *
 */

/*
 * Adapted from http://base64.sourceforge.net/b64.c
 * Copyright (c) Trantor Standard Systems Inc., 2001
 * and
 * https://github.com/littlstar/b64.c
 * Copyright (c) 2014 Joseph Werle
 */

/**
 * @defgroup   ArdB64 Base64 encoding
 * @brief      Base64 encoding and decoding, typically for serial and networking communications
 *
 * @{
 *
 */

#ifndef ARD_B64_H
#define ARD_B64_H

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

/**
 * @brief Buffer size macros
 */
#define ARD_B64_BUFFER_ENC_LEN(LENDEC) (((LENDEC + 2) / 3) * 4)
#define ARD_B64_BUFFER_DEC_LEN(LENENC) ((LENENC / 4) * 3)

    typedef enum eArdBase64Status
    {
        ARD_B64_NO_ERROR = 0,
        ARD_B64_ENCODED_SIZE_TOO_SMALL = 1,
        ARD_B64_DECODED_ARRAY_SIZE_TOO_SMALL = 2,
        ARD_B64_BAD_ENCODING = 3
    } eArdBase64Status;

    /**
     * @brief Encode to base64 from bytes
     *
     * @param dec
     * @param len_dec
     * @param enc
     * @param max_len_enc
     * @param len_enc
     * @return int
     */
    eArdBase64Status ard_b64_encode(const uint8_t *restrict dec, const size_t len_dec,
                                    char *restrict enc, const size_t max_len_enc,
                                    size_t *len_enc);

    /**
     * @brief Decode from base64 to bytes
     *
     * @param enc
     * @param len_enc
     * @param dec
     * @param max_len_dec
     * @param len_dec
     * @return int
     */
    eArdBase64Status ard_b64_decode(const char *restrict enc, const size_t len_enc,
                                    uint8_t *restrict dec, const size_t max_len_dec,
                                    size_t *len_dec);

    /* array <-> base64 conversions */
    eArdBase64Status ard_b64_convert_to_array(const char *restrict encoded_input,
                                              const size_t encoded_input_num_bytes,
                                              uint8_t *restrict array_output,
                                              const size_t array_output_type_bytes,
                                              const size_t array_output_max_size,
                                              size_t *array_output_size);

    eArdBase64Status ard_b64_convert_from_array(const uint8_t *restrict input_array,
                                                const size_t input_array_type_bytes,
                                                const size_t input_array_size,
                                                char *restrict encoded_output,
                                                const size_t encoded_output_max_bytes,
                                                size_t *encoded_output_bytes);

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// C++ interface

template <typename T>
eArdBase64Status ArdBase64ConvertFromArray(const T *input_array, const size_t input_array_size,
                                           char *encoded_output,
                                           const size_t encoded_output_max_bytes,
                                           size_t *encoded_output_bytes)
{
    return ard_b64_convert_from_array((const uint8_t *)input_array, sizeof(T), input_array_size,
                                      encoded_output, encoded_output_max_bytes,
                                      encoded_output_bytes);
};

template <typename T>
eArdBase64Status ArdBase64ConvertToArray(const char *encoded_input,
                                         const size_t encoded_input_num_bytes, T *array_output,
                                         const size_t array_output_max_size,
                                         size_t *array_output_size)
{
    return ard_b64_convert_to_array(encoded_input, encoded_input_num_bytes, (uint8_t *)array_output,
                                    sizeof(T), array_output_max_size, array_output_size);
}

#endif

#endif
