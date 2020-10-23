/**
 * @file       ard_crc.c
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      16-bit Cyclic Redundancy Check (crc16-ccitt)
 *
 * @details
 *
 * Cyclic redundancy check. See group @ref ArdCrc
 *
 */

#include "ard_b64.h"

/* encode 3 8-bit binary bytes as 4 '6-bit' characters */
static int ard_b64_encodeblock(const uint8_t in[3], char out[4], const size_t len_in)
{
    /* Translation Table as described in RFC1113 */
    static const char ard_cb64[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    if (len_in == 0) {
        out[0] = '=';
        out[1] = '=';
        out[2] = '=';
        out[3] = '=';
        return 0;
    }
    // retval is number of encoded bytes
    int retval = (len_in > 2 ? 4 : len_in + 1);
    // first character
    out[0] = ard_cb64[in[0] >> 2];
    if (retval == 4)
    {
        out[1] = ard_cb64[((in[0] & 0x03) << 4) | ((in[1] & 0xf0) >> 4)];
        out[2] = ard_cb64[((in[1] & 0x0f) << 2) | ((in[2] & 0xc0) >> 6)];
        out[3] = ard_cb64[in[2] & 0x3f];
    }
    else if (retval == 3)
    {
        out[1] = ard_cb64[((in[0] & 0x03) << 4) | ((in[1] & 0xf0) >> 4)];
        out[2] = ard_cb64[((in[1] & 0x0f) << 2)];
        out[3] = '=';
    }
    else
    {
        out[1] = ard_cb64[((in[0] & 0x03) << 4)];
        out[2] = '=';
        out[3] = '=';
    }

    return retval;
}

static int ard_b64_decodeblock(const char in[4], uint8_t out[3], const size_t len_out)
{
    /* decode 4 '6-bit' characters into 3 8-bit binary bytes */
    static const char ard_cd64[] =
        "|$$$}rstuvwxyz{$$$$$$$>?@ABCDEFGHIJKLMNOPQRSTUVW$$$$$$XYZ[\\]^_`abcdefghijklmnopq";

    // retval is number of decoded bytes if greater than 0
    int retval = (len_out > 2 ? 3 : len_out);
    char v[4] = {0};
    for (uint8_t i = 0; i < 4; i++)
    {
        if ('=' == in[i])
        {
            if (i == 3) {
                retval = 2;
            } else if (i == 2) {
                retval = 1;
            } else {
                retval = 0;
            }
            break;
        }

        if (in[i] < 43 || in[i] > 122)
        {
            retval = -1;
            break;
        }

        v[i] = ard_cd64[in[i] - 43];

        if (v[i] != '$')
        {
            v[i] = (v[i] - 62);
        }
        else
        {
            break;
        }
    }

    if (retval > 0)
    {
        out[0] = (uint8_t)(v[0] << 2 | v[1] >> 4);

        if (retval > 1)
        {
            out[1] = (uint8_t)(v[1] << 4 | v[2] >> 2);

            if (retval > 2)
            {
                out[2] = (uint8_t)(((v[2] << 6) & 0xc0) | v[3]);
            }
        }
    }

    return retval;
}

eArdBase64Status ard_b64_encode(const uint8_t *restrict dec, const size_t len_dec,
                                char *restrict enc, const size_t max_len_enc, size_t *len_enc)
{
    int ret = 0;

    (*len_enc) = 0;
    if (max_len_enc >= ARD_B64_BUFFER_ENC_LEN(len_dec))
    {
        /* full encode */
        for (size_t i = 0; i < len_dec; i += 3)
        {
            ret = ard_b64_encodeblock(dec + i, enc + (*len_enc), len_dec - i);
            if (ret == 0)
                break;
            (*len_enc) += ret;
        }
    }
    else
    {
        // Could not encode entire source. Maximum size of base64 array is not big enough.
        return ARD_B64_ENCODED_SIZE_TOO_SMALL;
    }

    return ARD_B64_NO_ERROR;
}

/* decode example */
eArdBase64Status ard_b64_decode(const char *restrict enc, const size_t len_enc,
                                uint8_t *restrict dec, const size_t max_len_dec, size_t *len_dec)
{
    size_t i = 0;
    int ret = 0;

    (*len_dec) = 0;
    if (max_len_dec >= ARD_B64_BUFFER_DEC_LEN(len_enc))
    {
        /* full decode */
        for (i = (*len_dec) = 0; i < len_enc; i += 4)
        {
            ret = ard_b64_decodeblock(enc + i, dec + (*len_dec), max_len_dec - (*len_dec));
            if (ret == -1)
            {
                // Improper encoding detected. Decoding aborted.
                return ARD_B64_BAD_ENCODING;
            } else if (ret == 0) {
                // no more processing
                break;
            }
            // increment
            (*len_dec) += ret;
        }
    }
    else
    {
        // Could not decode entire source. Maximum size of decoded data not big enough.
        return ARD_B64_DECODED_ARRAY_SIZE_TOO_SMALL;
    }

    return ARD_B64_NO_ERROR;
}

/* array <-> base64 conversions */
eArdBase64Status ard_b64_convert_to_array(const char *restrict encoded_input,
                                          const size_t encoded_input_num_bytes,
                                          uint8_t *restrict array_output,
                                          const size_t array_output_type_bytes,
                                          const size_t array_output_max_size,
                                          size_t *array_output_size)
{
    // maximum decoded length in bytes
    size_t max_output_bytes = array_output_max_size * array_output_type_bytes;
    // actual length of decoded array in bytes
    size_t output_num_bytes = 0;
    eArdBase64Status ret = ard_b64_decode(encoded_input, encoded_input_num_bytes, array_output,
                                          max_output_bytes, &output_num_bytes);
    // array size
    (*array_output_size) = output_num_bytes / array_output_type_bytes;
    return ret;
}

eArdBase64Status ard_b64_convert_from_array(const uint8_t *restrict input_array,
                                            const size_t input_array_type_bytes,
                                            const size_t input_array_size,
                                            char *restrict encoded_output,
                                            const size_t encoded_output_max_bytes,
                                            size_t *encoded_output_bytes)
{
    size_t input_array_btes = input_array_size * input_array_type_bytes;
    return ard_b64_encode(input_array, input_array_btes, encoded_output, encoded_output_max_bytes,
                          encoded_output_bytes);
}
