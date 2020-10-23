
/**
 * @file       ard_stepper.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      Stepper motor control
 *
 * @details
 *
 * See group @ref ArdStepper
 *
 */

#ifndef ARD_STEPPER_H
#define ARD_STEPPER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ard_encoder.h"

#define ARD_STEPPER_PULSE_ENABLED (defined(__AVR__) || defined(ESP_PLATFORM))

#ifdef __cplusplus

#ifndef restrict
#define restrict __restrict
#endif

extern "C"
{
#endif

    /**
     * @defgroup   ArdStepper Stepper Motor Control
     * @brief      Stepper motor control
     *
     * @{
     *
     */

#ifndef ARD_STEPPER_MIN_PULSE_PERIOD_MICROSECONDS
#define ARD_STEPPER_MIN_PULSE_PERIOD_MICROSECONDS 50
#endif

#ifndef ARD_STEPPER_PULSE_DURATION_MICROSECONDS
#define ARD_STEPPER_PULSE_DURATION_MICROSECONDS 20
#endif

#define ARD_STEPPER_POSITIVE_DIRECTION LOW
#define ARD_STEPPER_NEGATIVE_DIRECTION HIGH

    typedef struct ArdStepperPins
    {
        /**
         * @brief Pulse output pin
         *
         */
        uint8_t pulse;

        /**
         * @brief pulse direction pin
         *
         */
        uint8_t direction;

        /**
         * @brief enabled pin
         *
         */
        uint8_t enable;

        /**
         * @brief direction polarity
         *
         */
        uint8_t direction_polarity;

    } ArdStepperPins;

    /**
     * @brief Stepper pulse output state
     *
     */
    typedef struct ArdStepperState
    {
        /**
         * @brief Period between pulses in microseconds
         *
         */
        uint32_t pulse_step_size;
        /**
         * @brief Remaining steps in command output
         *
         */
        uint32_t steps_remaining;
        /**
         * @brief Direction of pulse steps
         *
         */
        uint8_t direction;
        /**
         * @brief Enabled stepper
         * 
         */
        bool enabled;
    } ArdStepperState;

    /**
     * @brief Stepper motor controller
     *
     */
    typedef struct ArdStepper
    {
        /**
         * @brief Circular buffer of commands for stepper
         *
         */
        int32_t *buffer;
        /**
         * @brief Set to 1 if buffer is full
         *
         */
        size_t buffer_tail;
        /**
         * @brief Current index of command arrays
         *
         */
        size_t buffer_index;
        /**
         * @brief Number of commands in step_increment_commands buffer
         *
         */
        size_t buffer_size;
        /**
         * @brief Command step size in microseconds
         *
         */
        uint32_t command_step_size;
        /**
         * @brief Delay between producing and consuming commands
         *
         */
        size_t command_delay_steps;
        /**
         * @brief @c `true` if drive is enabled
         *
         */
        bool enabled;
        /**
         * @brief Stepper motor pulses
         *
         */
        ArdEncoder encoder;

    } ArdStepper;

    /**
     * @brief Stepper motor controller parameters
     *
     */
    typedef struct ArdStepperParameters
    {
        /**
         * @brief Number of commands in step_increment_commands buffer
         *
         */
        size_t buffer_size;

        /**
         * @brief Stepper motor pulses per revolution
         *
         */
        uint32_t pulses_per_rev;

        /**
         * @brief Motor physical units per revolution (eg. degrees, mm, etc.)
         *
         */
        double units_per_rev;

        /**
         * @brief Maximum command velocity in physical units
         *
         */
        double max_unit_velocity;

        /**
         * @brief Minimum step size in microsepconds. High frequency sampling
         *
         */
        uint32_t min_pulse_step_size;

        /**
         * @brief Command loop step size
         *
         */
        uint32_t command_step_size;

        /**
         * @brief Delay between producing and consuming commands
         *
         */
        size_t command_delay_steps;
        /**
         * @brief Stepper pin assignments
         *
         */
        ArdStepperPins pins;

    } ArdStepperParameters;

    /**
     * @brief Allocate memory and initialize stepper controller
     *
     * @param controller Stepper controller
     * @param max_command_size maximum command array size
     * @param buffer_num_commands Number of command arrays in circular buffer
     * @param command_step_size Step size of controller
     * @return int
     */
    int ard_stepper_alloc(ArdStepper *stepper, const ArdStepperParameters *parameters);

    /**
     * @brief Controller delay
     *
     * @return uint32_t microseconds of controller delay
     */
    uint32_t ard_stepper_get_delay_microseconds(ArdStepper *stepper);

    /**
     * @brief
     *
     * @param controller
     */
    void ard_stepper_free(ArdStepper *stepper);

    /**
     * @brief Enable stepper
     *
     * @param stepper
     */
    void ard_stepper_enable(ArdStepper *stepper);

    /**
     * @brief Disable stepper
     *
     * @param stepper
     */
    void ard_stepper_disable(ArdStepper *stepper);

    /**
     * @brief
     *
     * @param controller
     * @return int
     */
    void ard_stepper_reset(ArdStepper *stepper, const double position, const bool enable);

    /**
     * @brief Append desired position to command buffer
     *
     * @param stepper controller
     * @param position desired position
     * @return int
     */
    int ard_stepper_produce_command(ArdStepper *stepper,
                                    const double *restrict position, const size_t size);

    /**
     * @brief run stepper
     *
     * @param controller Stepper controller
     * @param now microseconds
     * @return int whether pulse was sent
     */
    void ard_stepper_consume_run(ArdStepper *stepper, const uint32_t now);

    void ard_stepper_pulse_train_reset(const bool enable);

    void ard_stepper_pulse_train_setup(const ArdStepperPins *pins,
                                       const uint32_t pulse_timer_step_size_us);

    /**
     * @brief
     *
     * @param controller
     * @param steps
     * @param direction
     * @param step_period_microseconds
     * @return int
     */

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
