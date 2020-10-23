
/**
 * @file       ard_encoder.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      Counter encoding and decoding
 *
 * @details
 *
 * See group @ref ArdEncoder
 *
 */

#ifndef ARD_ENCODER_H
#define ARD_ENCODER_H

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
     * @defgroup   ArdEncoder Incremental encoder
     * @brief      Counter encoding and decoding
     *
     * @{
     *
     */

    /**
     * @brief Encoder state
     *
     */
    typedef struct ArdEncoder
    {
        /**
         * @brief Current encoder count
         *
         */
        double value;
        /**
         * @brief Current encoder count
         *
         */
        int32_t count;
        /**
         * @brief Scaled units per revolution
         *
         */
        double value_per_rev;
        /**
         * @brief Encoder counts per revolution
         *
         */
        uint32_t counts_per_rev;
        /**
         * @brief incremental encoder limit based on counter bit depth
         *
         */
        int32_t counter_limit;
    } ArdEncoder;

    /**
     * @brief Set encoder parameters
     *
     * @param enc Encoder
     * @param counter_resolution Bit dpeth of counter (max 32)
     * @param counts_per_rev Number of counts per recvolution
     * @param value_per_rev scaled units per revolution
     * @return int 0 is success, error otherwise
     */
    int ard_encoder_set(ArdEncoder *enc, const uint8_t counter_resolution_bits,
                               const uint32_t counts_per_rev, const double value_per_rev);

    /**
     * @brief Reset encoder to value and counts
     *
     * @param enc encoder
     * @param reset_count reset counter counts
     * @param reset_value reset scaled value
     */
    void ard_encoder_reset(ArdEncoder *enc, const int32_t reset_count, const double reset_value);

    /**
     * @brief Reset encoder value. Counts set to scaled value
     *
     * @param enc encoder
     * @param reset_value reset scaled value
     */
    void ard_encoder_reset_absolute(ArdEncoder *enc, const double reset_value);

    /**
     * @brief Get value in scaled units from counter update
     *
     * @param enc encoder
     * @param new_count new input from counter
     * @return double scaled value
     */
    double ard_encoder_decode(ArdEncoder *enc, const int32_t new_count);

    /**
     * @brief encode value to number of counts
     *
     * @param enc encoder
     * @param value value to encode
     * @return int32_t change in counts from last encode update
     */
    int32_t ard_encoder_encode(ArdEncoder *enc, const double value);

    // /**
    //  * @brief Use interrupts to count a single quadrature encoder with pins a and b
    //  *
    //  * This is only enabled if 8-bit avr or esp32 boards are being used.
    //  *
    //  * @param pin_a Quadrature input pin A
    //  * @param pin_b Quadrature input pin B
    //  * @return 0 on success, error otherwise
    //  */
    // int ard_encoder_quad_setup(const uint8_t pin_a, const uint8_t pin_b);

    // /**
    //  * @brief Get latest count from quadrature encoder
    //  *
    //  * The result can be used to update encoder value with
    //  * @c ard_encoder_decode
    //  *
    //  * @return int32_t Count
    //  */
    // int32_t ard_encoder_quad_get_count();

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
