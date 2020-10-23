/**
 * @file       ard_timing.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      Timing Module
 *
 * @details
 *
 * See group @ref ArdTiming
 *
 */

#ifndef ARD_TIMING_H
#define ARD_TIMING_H

#include <stdbool.h>
#include <stdint.h>


/**
 * @defgroup   ArdCrc Cyclic Redundancy Check
 * @brief      C wrapper for I2C Communications.
 *
 * @{
 *
 */

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Step size at 10Hz
     */
    #define ARD_TIMING_10HZ_STEP_SIZE 100000u
    /**
     * @brief Step size at 50Hz
     */
    #define ARD_TIMING_50HZ_STEP_SIZE 20000u
    /**
     * @brief Step size at 100Hz
     */
    #define ARD_TIMING_100HZ_STEP_SIZE 10000u
    /**
     * @brief Step size at 200Hz
     */
    #define ARD_TIMING_200HZ_STEP_SIZE 5000u

    /**
     * @brief Timing data
     *
     */
    typedef struct ArdTiming
    {
        uint32_t prev;
        uint32_t now;
        uint32_t step_size;
    } ArdTiming;

    /**
     * @brief Timeout data
     *
     */
    typedef struct ArdTimeout
    {
        bool is_set;
        uint32_t init;
        uint32_t wait_micros;
    } ArdTimeout;

    /**
     * @brief Reset timer data
     *
     * @param step_size Step size in microseconds
     * @param now Current time in microseconds
     * @param[out] timing Timing data
     */
    void ard_timing_reset(const uint32_t step_size, const uint32_t now, ArdTiming *timing);

    /**
     * @brief Step timer
     *
     * @param timing Timing data
     * @param now Current time in microseconds
     * @return uint32_t Steps elapsed since last time_step
     */
    uint32_t ard_timing_step(ArdTiming *timing, const uint32_t now);

    /**
     * @brief Timeout utility
     *
     * @param now
     * @param wait_micros
     * @param[out] timeout Timeout data
     */
    void ard_timeout_set(const uint32_t now, const uint32_t wait_micros, ArdTimeout *timeout);

    /**
     * @brief Check if timesout expired
     *
     * @param timeout Timeout data
     * @param now Current time in microseconds
     * @return true if timeout expired
     * @return false if timeout not yet expired
     */
    bool ard_timeout_check(ArdTimeout *timeout, const uint32_t now);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
