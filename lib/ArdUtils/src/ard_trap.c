
#include "ard_trap.h"

#include "math.h"

int ard_trap_set_parameters(ArdTrapezoidal *trap, const double step_size, const double max_velocity,
                            const double max_acceleration)
{
    int ret = 0;
    if (step_size < ARD_TRAPEZOID_NUMERICAL_TOLERANCE ||
        max_acceleration < ARD_TRAPEZOID_NUMERICAL_TOLERANCE ||
        max_velocity < ARD_TRAPEZOID_NUMERICAL_TOLERANCE)
    {
        // bad inputs
        ret = -1;
    }
    else
    {
        trap->max_acceleration = max_acceleration;
        trap->max_velocity = max_velocity;
        trap->step_size = step_size;
    }
    return ret;
}

void ard_trap_reset(ArdTrapezoidal *trap, const uint32_t time_us, const double position)
{
    trap->duration_ramp = 0.0;
    trap->duration_plateau = 0.0;
    trap->time_start = time_us;
    trap->initial_position = position;
    trap->final_position = position;
}

void ard_trap_generate(ArdTrapezoidal *trap, const uint32_t time_us, const double initial_position,
                       const double final_position)
{
    const double delta_position = fabs(final_position - initial_position);
    // initial
    trap->duration_ramp =
        ceil(trap->max_velocity / (trap->max_acceleration * trap->step_size)) * trap->step_size;
    trap->peak_acceleration = trap->max_velocity / trap->duration_ramp;

    if ((delta_position + ARD_TRAPEZOID_NUMERICAL_TOLERANCE) <
        (trap->peak_acceleration * trap->duration_ramp * trap->duration_ramp))
    {
        // no plateau
        trap->duration_plateau = 0.0;
        // ramp duration
        trap->duration_ramp =
            ceil(sqrt(delta_position / trap->max_acceleration) / trap->step_size) * trap->step_size;
        // peak velocity
        trap->peak_velocity = trap->duration_ramp * delta_position;
        // peak acceleration
        trap->peak_acceleration = trap->peak_velocity / trap->duration_ramp;
    }
    else
    {
        // has plateau
        trap->duration_plateau =
            ceil((delta_position -
                  trap->peak_acceleration * trap->duration_ramp * trap->duration_ramp) /
                 (trap->max_velocity * trap->step_size)) *
            trap->step_size;
        // peak velocity
        trap->peak_velocity = delta_position / (1.0 + trap->duration_ramp / trap->duration_plateau);
        // peak acceleration
        trap->peak_acceleration = trap->peak_velocity / trap->duration_ramp;

        // refine plateau duration with new acceleration
        trap->duration_plateau =
            ceil((delta_position -
                  trap->peak_acceleration * trap->duration_ramp * trap->duration_ramp) /
                 (trap->peak_velocity * trap->step_size)) *
            trap->step_size;
        // new peak velocity with updated durations
        trap->peak_velocity = delta_position / (1.0 + trap->duration_ramp / trap->duration_plateau);
    }

    // set initial and desired
    trap->final_position = final_position;
    trap->initial_position = trap->initial_position;
    // start time
    trap->time_start = time_us;
    // direction
    trap->direction = (trap->final_position == trap->initial_position
                           ? 0
                           : (trap->final_position > trap->initial_position ? 1 : -1));
}

double ard_trap_evaluate_position(const ArdTrapezoidal *trap, const uint32_t time_us)
{
    // trajectory time and start position
    double t = (double)(time_us - trap->time_start) / 1.0e6;
    double start_position = trap->initial_position;
    // set peak velocity and acceleration sign
    const double peak_velocity = (trap->direction < 0 ? -trap->peak_velocity : trap->peak_velocity);
    const double peak_acceleration =
        (trap->direction < 0 ? -trap->peak_acceleration : trap->peak_acceleration);

    if (t < 0.0 || trap->duration_ramp == 0)
    {   
        // initial position
        return start_position;
    }
    else if (t < trap->duration_ramp)
    {
        // start ramp
        return 0.5 * t * t * peak_acceleration + t * peak_velocity + start_position;
    }
    else if (trap->duration_plateau > 0 && t <= (trap->duration_ramp + trap->duration_plateau))
    {
        // update start time and position
        t -= trap->duration_ramp;
        start_position += 0.5 * trap->duration_ramp * trap->duration_ramp * peak_acceleration +
                          trap->duration_ramp * peak_velocity;
        // plateau
        return start_position + t * peak_velocity;
    }
    else if (t < (2.0 * trap->duration_ramp + trap->duration_plateau))
    {
        // update start time and position
        t -= (trap->duration_ramp + trap->duration_plateau);
        start_position += 0.5 * trap->duration_ramp * trap->duration_ramp * peak_acceleration +
                          trap->duration_ramp * peak_velocity +
                          trap->duration_plateau * peak_velocity;
        // end ramp
        return start_position - 0.5 * t * t * peak_acceleration - t * peak_velocity;
    }
    else
    {
        // final position
        return trap->final_position;
    }
}

// void ard_trap_evaluate_pva(const ArdTrapezoidal *trap, const uint32_t t, double *position,
//                            double *velocity, double *acceleration)
// {
// }
