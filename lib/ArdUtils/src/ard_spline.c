
#include "ard_spline.h"

#include "math.h"

// CUBIC

static void generate_coeffs_zero_vel_cubic(const double duration, const double start_position,
                                           const double end_position, double *restrict coeffs)
{
    const double h = end_position - start_position;

    const double t2 = duration * duration;
    const double t3 = t2 * duration;

    coeffs[3] = -2 * h / t3;
    coeffs[2] = 3 * h / t2;
    coeffs[1] = 0.0;
    coeffs[0] = start_position;
}

static void generate_coeffs_cubic(const double duration, const double start_position,
                                  const double end_position, const double start_velocity,
                                  const double end_velocity, double *restrict coeffs)
{
    const double h = end_position - start_position;

    const double t2 = duration * duration;
    const double t3 = t2 * duration;

    coeffs[3] = (-2 * h + duration * (end_velocity + start_velocity)) / t3;
    coeffs[2] = (3 * h - duration * (end_velocity + 2 * start_velocity)) / t2;
    coeffs[1] = start_velocity;
    coeffs[0] = start_position;
}

// QUINTIC

static void generate_coeffs_zero_vel_quintic(const double duration, const double start_position,
                                             const double end_position, double *restrict coeffs)
{
    const double h = end_position - start_position;

    const double t2 = duration * duration;
    const double t3 = t2 * duration;
    const double t4 = t2 * t2;
    const double t5 = t4 * duration;

    coeffs[5] = 6.0 * h / t5;
    coeffs[4] = -15.0 * h / t4;
    coeffs[3] = 10 * h / t3;
    coeffs[2] = 0.0;
    coeffs[1] = 0.0;
    coeffs[0] = start_position;
}

static void generate_coeffs_zero_acc_quintic(const double duration, const double start_position,
                                             const double end_position, const double start_velocity,
                                             const double end_velocity, double *restrict coeffs)
{
    const double h = end_position - start_position;

    const double t2 = duration * duration;
    const double t3 = t2 * duration;
    const double t4 = t2 * t2;
    const double t5 = t4 * duration;

    coeffs[5] = 3 * (2 * h - duration * (end_velocity + start_velocity)) / t5;
    coeffs[4] = (-15 * h + duration * (7 * end_velocity + 8 * start_velocity)) / t4;
    coeffs[3] = 2 * (5 * h - 2 * duration * end_velocity - 3 * duration * start_velocity) / t3;
    coeffs[2] = 0.0;
    coeffs[1] = start_velocity;
    coeffs[0] = start_position;
}

static void generate_coeffs_quintic(const double duration, const ArdSplinePva *start,
                                    const ArdSplinePva *end, double *restrict coeffs)
{
    const double h = end->position - start->position;

    const double t2 = duration * duration;
    const double t3 = t2 * duration;
    const double t4 = t2 * t2;
    const double t5 = t4 * duration;

    coeffs[5] = -(h * -12.0 + duration * end->velocity * 6.0 + duration * start->velocity * 6.0 -
                  end->acceleration * t2 + start->acceleration * t2) /
                (2.0 * t5);
    coeffs[4] = (h * -30.0 + duration * end->velocity * 14.0 + duration * start->velocity * 16.0 -
                 end->acceleration * t2 * 2.0 + start->acceleration * t2 * 3.0) /
                (2.0 * t4);
    coeffs[3] = -(h * -20.0 + duration * end->velocity * 8.0 + duration * start->velocity * 12.0 -
                  end->acceleration * t2 + start->acceleration * t2 * 3.0) /
                (2.0 * t3);
    coeffs[2] = start->acceleration / 2.0;
    coeffs[1] = start->velocity;
    coeffs[0] = start->position;
}

int ard_spline_generate(ArdSpline *spline, const ArdSplinePva *start, const ArdSplinePva *end,
                        const double duration, const eArdSplineType type)
{
    int retval = 0;
    const double displacement = end->position - start->position;

    if ((duration < ARD_SPLINE_MINIMUM_DURATION) &&
        ((fabs(displacement) >= ARD_SPLINE_TOLERANCE) ||
         (fabs(start->velocity) > ARD_SPLINE_TOLERANCE) ||
         (fabs(end->velocity) > ARD_SPLINE_TOLERANCE) ||
         (fabs(start->acceleration) > ARD_SPLINE_TOLERANCE) ||
         (fabs(end->acceleration) > ARD_SPLINE_TOLERANCE)))
    {
        // must have duration if positions are not the same or velocity/acceleration greater than
        // 0.0
        retval = 1;
    }
    else if ((duration >= ARD_SPLINE_MINIMUM_DURATION) &&
             (fabs(displacement) < ARD_SPLINE_TOLERANCE))
    {
        // must have same position separation if duration greater than minimum
        retval = 2;
    }
    else if (duration < ARD_SPLINE_MINIMUM_DURATION)
    {
        // no duration
        spline->duration = 0.0;
        spline->type = type;

        // coefficients
        spline->coeffs[0] = end->position;
        spline->coeffs[1] = 0.0;
        spline->coeffs[2] = 0.0;
        spline->coeffs[3] = 0.0;
        spline->coeffs[4] = 0.0;
        spline->coeffs[5] = 0.0;
    }
    else if ((fabs(start->velocity) < ARD_SPLINE_TOLERANCE) &&
             (fabs(end->velocity) < ARD_SPLINE_TOLERANCE) &&
             (fabs(start->acceleration) < ARD_SPLINE_TOLERANCE) &&
             (fabs(end->acceleration) < ARD_SPLINE_TOLERANCE))
    {
        // zero velocity/acceleration end conditions
        spline->duration = duration;
        spline->type = type;

        // generate coefficients
        switch (spline->type)
        {
            case EARD_SPLINE_QUINTIC:
            {
                // generate quintic
                generate_coeffs_zero_vel_quintic(duration, start->position, end->position,
                                                 spline->coeffs);
                break;
            }

            case EARD_SPLINE_CUBIC:
            {
                // generate cubic
                generate_coeffs_zero_vel_cubic(duration, start->position, end->position,
                                               spline->coeffs);
                break;
            }

            default:
            {
                // should never reach here
                retval = 3;
                break;
            }
        }
    }
    else if ((fabs(start->acceleration) < ARD_SPLINE_TOLERANCE) &&
             (fabs(end->acceleration) < ARD_SPLINE_TOLERANCE))
    {
        // zero acceleration end conditions
        spline->duration = duration;
        spline->type = type;

        // generate coefficients
        switch (spline->type)
        {
            case EARD_SPLINE_QUINTIC:
            {
                // generate quintic
                generate_coeffs_zero_acc_quintic(duration, start->position, end->position,
                                                 start->velocity, end->velocity, spline->coeffs);
                break;
            }

            case EARD_SPLINE_CUBIC:
            {
                // generate cubic
                generate_coeffs_cubic(duration, start->position, end->position, start->velocity,
                                      end->velocity, spline->coeffs);
                break;
            }

            default:
            {
                // should never reach here
                retval = 3;
                break;
            }
        }
    }
    else
    {
        // generalized end conditions
        spline->duration = duration;
        spline->type = type;

        // generate coefficients
        switch (spline->type)
        {
            case EARD_SPLINE_QUINTIC:
            {
                // generate quintic
                generate_coeffs_quintic(duration, start, end, spline->coeffs);
                break;
            }

            case EARD_SPLINE_CUBIC:
            {
                // generate cubic
                generate_coeffs_cubic(duration, start->position, end->position, start->velocity,
                                      end->velocity, spline->coeffs);
                break;
            }

            default:
            {
                // should never reach here
                retval = 3;
                break;
            }
        }
    }

    return retval;
}

void ard_spline_evaluate(const ArdSpline *spline, double t, ArdSplinePva *out)
{
    t = (t < 0.0 ? 0.0 : (t > spline->duration ? spline->duration : t));
    switch (spline->type)
    {
        case EARD_SPLINE_QUINTIC:
        {
            const double t2 = t * t;
            const double t3 = t2 * t;
            const double t4 = t3 * t;
            const double t5 = t4 * t;
            out->position = spline->coeffs[0] + spline->coeffs[1] * t + spline->coeffs[2] * t2 +
                            spline->coeffs[3] * t3 + spline->coeffs[4] * t4 +
                            spline->coeffs[5] * t5;
            out->velocity = spline->coeffs[1] + 2.0 * spline->coeffs[2] * t +
                            3.0 * spline->coeffs[3] * t2 + 4.0 * spline->coeffs[4] * t3 +
                            5.0 * spline->coeffs[5] * t4;
            out->acceleration = 2.0 * spline->coeffs[2] + 6.0 * spline->coeffs[3] * t +
                                12.0 * spline->coeffs[4] * t2 + 20.0 * spline->coeffs[5] * t3;
            break;
        }

        case EARD_SPLINE_CUBIC:
        {
            const double t2 = t * t;
            const double t3 = t2 * t;
            out->position = spline->coeffs[0] + spline->coeffs[1] * t + spline->coeffs[2] * t2 +
                            spline->coeffs[3] * t3;
            out->velocity =
                spline->coeffs[1] + 2.0 * spline->coeffs[2] * t + 3.0 * spline->coeffs[3] * t2;
            out->acceleration = 2.0 * spline->coeffs[2] + 6.0 * spline->coeffs[3] * t;
            break;
        }

        default:
        {
            // should never reach here
            break;
        }
    }
}

double ard_spline_evaluate_position(const ArdSpline *spline, double t)
{
    t = (t < 0.0 ? 0.0 : (t > spline->duration ? spline->duration : t));
    switch (spline->type)
    {
        case EARD_SPLINE_QUINTIC:
        {
            const double t2 = t * t;
            const double t3 = t2 * t;
            const double t4 = t3 * t;
            const double t5 = t4 * t;
            return spline->coeffs[0] + spline->coeffs[1] * t + spline->coeffs[2] * t2 +
                   spline->coeffs[3] * t3 + spline->coeffs[4] * t4 + spline->coeffs[5] * t5;
        }

        case EARD_SPLINE_CUBIC:
        {
            const double t2 = t * t;
            const double t3 = t2 * t;
            return spline->coeffs[0] + spline->coeffs[1] * t + spline->coeffs[2] * t2 +
                   spline->coeffs[3] * t3;
        }

        default:
        {
            // should never reach here
            break;
        }
    }

    return 0.0;
}


int ard_spline_traj_generate(ArdSplineTrajectory *traj, const ArdSplinePva *start, const ArdSplinePva *end,
                             double duration, const uint16_t step_size, const eArdSplineType type) {
    // initialize with no steps         
    traj->index = 0;
    traj->num_steps = 0;
    traj->step_size = ((double) step_size) / 1.0e6;
    // steps and duration
    if (duration < 0.0) return -1;
    const uint16_t num_steps = (uint16_t) (duration / traj->step_size);
    if (num_steps == 0) return -2;
    duration = ((double) num_steps) * traj->step_size;
    // generate
    int ret = ard_spline_generate(&traj->spline, start, end, duration, type);
    if (ret == 0) {
        traj->num_steps = num_steps;
    }
    return ret;
}

void ard_spline_traj_evaluate(ArdSplineTrajectory *traj, ArdSplinePva *out) {
    if (traj->index <= traj->num_steps) {
        const double t = ((double) traj->index) * traj->step_size;
        traj->index += 1;
        ard_spline_evaluate(&traj->spline, t, out);
    } else {
        ard_spline_evaluate(&traj->spline, traj->spline.duration, out);
    }
}

// void ard_spline_traj_evaluate_at_index(ArdSplineTrajectory *traj, const uint16_t index, ArdSplinePva *out) {
//     if (traj->num_steps == 0) return;

//     traj->index = (index < traj->num_steps ? index : traj->num_steps - 1);
//     const double t = ((double) traj->index) * traj->step_size;
//     traj->index += 1;
//     ard_spline_evaluate(&traj->spline, t, out);
// }

double ard_spline_traj_evaluate_position(ArdSplineTrajectory *traj) {
    if (traj->num_steps == 0) return 0.0;

    if (traj->index <= traj->num_steps) {
        const double t = ((double) traj->index) * traj->step_size;
        traj->index += 1;
        return ard_spline_evaluate_position(&traj->spline, t);
    } else {
        return ard_spline_evaluate_position(&traj->spline, traj->spline.duration);
    }
}
