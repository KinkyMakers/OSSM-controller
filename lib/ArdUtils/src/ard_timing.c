
#include "ard_timing.h"

void ard_timing_reset(const uint32_t step_size, const uint32_t now, ArdTiming *timing)
{
    timing->prev = now;
    timing->now = now;
    timing->step_size = step_size;
}

uint32_t ard_timing_step(ArdTiming *timing, const uint32_t now)
{
    timing->now = now;
    uint32_t elapsed_steps = 0;
    if ((now - timing->prev) >= timing->step_size)
    {
        // elapsed steps
        elapsed_steps = (now - timing->prev) / timing->step_size;
        timing->prev += elapsed_steps * timing->step_size;
    }
    return elapsed_steps;
}

void ard_timeout_set(const uint32_t now, const uint32_t wait_micros, ArdTimeout *timeout)
{
    timeout->is_set = true;
    timeout->init = now;
    timeout->wait_micros = wait_micros;
}

bool ard_timeout_check(ArdTimeout *timeout, const uint32_t now)
{
    if (timeout->is_set && (now - timeout->init) > timeout->wait_micros)
    {
        timeout->is_set = false;
        return true;
    }
    return false;
}
