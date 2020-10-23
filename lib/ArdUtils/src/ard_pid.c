
#include "ard_pid.h"

#include <stdbool.h>

/**
 * @brief      Reset PID (zero everything)
 *
 * @param      state  State container for PID
 */
void ard_pid_reset(ArdPid *pid)
{
    pid->state.e1 = 0.0;
    pid->state.e2 = 0.0;

    pid->state.u1 = 0.0;
    pid->state.u2 = 0.0;
}

double ard_pid_run_fwd(const double des, const double meas, ArdPid *pid, const double ts)
{
    /* error */
    const double e = des - meas;

    /* calculate output */
    const double t2 = 1.0 / pid->tf;
    const double t3 = pid->kp * pid->tf;
    double u =
        pid->state.e2 * t2 *
            (pid->kd + t3 - pid->kp * ts + pid->ki * (ts * ts) - pid->ki * pid->tf * ts) +
        t2 * pid->state.u1 * (pid->tf * 2.0 - ts) + e * t2 * (pid->kd + t3) -
        pid->state.e1 * t2 *
            (pid->kd * 2.0 + pid->kp * pid->tf * 2.0 - pid->kp * ts - pid->ki * pid->tf * ts) -
        t2 * pid->state.u2 * (pid->tf - ts);

    /* saturate output */
    u = (u > pid->umax ? pid->umax : (u < pid->umin ? pid->umin : u));

    /* next state */
    pid->state.u2 = pid->state.u1;
    pid->state.u1 = u;

    pid->state.e2 = pid->state.e1;
    pid->state.e1 = e;

    return u;
}

double ard_pid_run_back(const double des, const double meas, ArdPid *pid, const double ts)
{
    /* error */
    const double e = des - meas;

    /* calculate output */
    double u = (e * pid->kd - pid->state.e1 * pid->kd * 2.0 + pid->state.e2 * pid->kd +
                pid->tf * pid->state.u1 * 2.0 - pid->tf * pid->state.u2 + ts * pid->state.u1 +
                e * pid->ki * (ts * ts) + e * pid->kp * pid->tf + e * pid->kp * ts -
                pid->state.e1 * pid->kp * pid->tf * 2.0 + pid->state.e2 * pid->kp * pid->tf -
                pid->state.e1 * pid->kp * ts + e * pid->ki * pid->tf * ts -
                pid->state.e1 * pid->ki * pid->tf * ts) /
               (pid->tf + ts);

    /* saturate output */
    u = (u > pid->umax ? pid->umax : (u < pid->umin ? pid->umin : u));

    /* next state */
    pid->state.u2 = pid->state.u1;
    pid->state.u1 = u;

    pid->state.e2 = pid->state.e1;
    pid->state.e1 = e;

    return u;
}

double ard_pid_run_trap(const double des, const double meas, ArdPid *pid, const double ts)
{
    /* error */
    const double e = des - meas;

    /* calculate output */
    const double t2 = ts * ts;
    double u = (e * pid->kd * 2.0 - pid->state.e1 * pid->kd * 4.0 + pid->state.e2 * pid->kd * 2.0 +
                pid->tf * pid->state.u1 * 4.0 - pid->tf * pid->state.u2 * 2.0 + ts * pid->state.u2 +
                e * pid->ki * t2 * (1.0 / 2.0) + e * pid->kp * pid->tf * 2.0 +
                pid->state.e1 * pid->ki * t2 + pid->state.e2 * pid->ki * t2 * (1.0 / 2.0) +
                e * pid->kp * ts - pid->state.e1 * pid->kp * pid->tf * 4.0 +
                pid->state.e2 * pid->kp * pid->tf * 2.0 - pid->state.e2 * pid->kp * ts +
                e * pid->ki * pid->tf * ts - pid->state.e2 * pid->ki * pid->tf * ts) /
               (pid->tf * 2.0 + ts);

    /* saturate output */
    u = (u > pid->umax ? pid->umax : (u < pid->umin ? pid->umin : u));

    /* next state */
    pid->state.u2 = pid->state.u1;
    pid->state.u1 = u;

    pid->state.e2 = pid->state.e1;
    pid->state.e1 = e;

    return u;
}
