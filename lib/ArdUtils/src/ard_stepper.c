#include "ard_stepper.h"

#include <Arduino.h>

int ard_stepper_alloc(ArdStepper *stepper, const ArdStepperParameters *parameters)
{
    if (parameters->buffer_size < 2) return -1;
    if (parameters->command_delay_steps >= parameters->buffer_size) return -2;
    if (parameters->min_pulse_step_size < 2 * ARD_STEPPER_PULSE_DURATION_MICROSECONDS) return -3;

    // buffer sizes
    stepper->buffer_tail = 0;
    stepper->buffer_index = 0;
    stepper->buffer_size = parameters->buffer_size;

    // initalize buffer as NULL
    stepper->buffer = NULL;

    // allocate buffer
    stepper->buffer = (int32_t *)calloc(parameters->buffer_size, sizeof(int32_t));
    if (stepper->buffer == NULL)
    {
        return -4;
    }

    // command step size and delay
    stepper->command_step_size = parameters->command_step_size;
    stepper->command_delay_steps = parameters->command_delay_steps;

    // initialize in disabled state
    stepper->enabled = false;

    // command encoder
    ard_encoder_set_32bit(&stepper->encoder, parameters->pulses_per_rev, parameters->units_per_rev);

    // check min period
    const uint32_t min_pulse_period =
        (uint32_t)(1.0e6 * parameters->units_per_rev /
                   (parameters->max_unit_velocity * ((double)parameters->pulses_per_rev)));
    if (parameters->min_pulse_step_size * 2 > min_pulse_period)
    {
        // min_pulse_step_size, max velocity, or pulses_per_rev too high
        return -5;
    }

    // check maximum pulses per command
    const uint64_t max_pulses_per_command =
        (uint64_t)(1.0e6 * ((double)stepper->command_step_size) / min_pulse_period);
    if (max_pulses_per_command > INT32_MAX)
    {
        // too many pulses per command for 32 bit signed integers
        return -6;
    }

    // pulse state
    const uint32_t pulse_step_size = min_pulse_period / 2;
    ard_stepper_pulse_train_setup(&parameters->pins, pulse_step_size);

    return 0;
}

void ard_stepper_free(ArdStepper *stepper)
{
    ard_stepper_disable(stepper);

    free(stepper->buffer);
    stepper->buffer = NULL;

    stepper->buffer_tail = 0;
    stepper->buffer_index = 0;
    stepper->buffer_size = 0;
}

uint32_t ard_stepper_get_delay_microseconds(ArdStepper *stepper)
{
    return stepper->command_delay_steps * stepper->command_step_size;
}

void ard_stepper_enable(ArdStepper *stepper)
{
    ard_stepper_reset(stepper, stepper->encoder.value, true);
    stepper->enabled = true;
}

void ard_stepper_disable(ArdStepper *stepper)
{
    stepper->enabled = false;
    ard_stepper_reset(stepper, stepper->encoder.value, false);
}

void ard_stepper_reset(ArdStepper *stepper, const double position, const bool enable)
{
    // reset pulse state
    ard_stepper_pulse_train_reset(enable);

    stepper->buffer_tail = 0;
    stepper->buffer_index = 0;
    stepper->enabled = enable;

    // reset encoder
    ard_encoder_reset_absolute(&stepper->encoder, position);
}

int ard_stepper_produce_command(ArdStepper *stepper,
                                const double *restrict position, const size_t size)
{
    // space available in buffer
    size_t available_in_queue =
        (stepper->buffer_tail > stepper->buffer_index
             ? stepper->buffer_tail - stepper->buffer_index
             : stepper->buffer_size - (stepper->buffer_index - stepper->buffer_tail));
    // add positions array to buffer
    size_t k = 0;
    while (k < size && k < available_in_queue)
    {
        // encode: set delta steps to buffer
        stepper->buffer[stepper->buffer_index] = ard_encoder_encode(&stepper->encoder, position[k]);
        // advance
        stepper->buffer_index = (stepper->buffer_index + 1) % stepper->buffer_size;
        k += 1;
    }
    return k;
}
