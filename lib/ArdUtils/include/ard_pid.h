/**
 * @file       ard_pid.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      PID controller
 *
 * @details
 *
 * See group @ref ArdPid
 *
 */

#ifndef ARD_PID_H
#define ARD_PID_H

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @defgroup   ArdPid PID controller
     * @brief      PID controller
     *
     * The PID implementation is based on Matlab's discretized PID controllers as described in
     * [Matlab's PID controller documentation](http://www.mathworks.com/help/control/ref/pid.html)
     *
     * The main PID dicrete-time controller transfer function is
     *
     * \f$C = K_p + K_i IF(z) + \frac{K_d}{T_f + DF(z)}\f$
     *
     * where \f$ IF(z) \f$ and \f$ DF(z) \f$ depend on the type of discretization and the parameters
     * \f$K_p, \; K_i, \; K_d, \; T_f\f$, and the sample time \f$T_f\f$ correspond to the parameters
     * in ArdPidParam \ref
     *
     * @{
     *
     */

    /**
     * State container for PID controller
     */
    typedef struct ArdPidState
    {
        double e1;
        double e2;
        double u1;
        double u2;
    } ArdPidState;

    /**
     * Constant PID parameters
     */
    typedef struct ArdPid
    {
        /** proportional gain */
        double kp;
        /** integral gain */
        double ki;
        /** derivative gain */
        double kd;
        /** minimum control variable output (saturation) */
        double umin;
        /** maximum control variable output (saturation) */
        double umax;
        /** time constant for derivative filter (in seconds) */
        double tf;
        /**
         * @brief Internal state
         *
         */
        ArdPidState state;
    } ArdPid;

    void ard_pid_reset(ArdPid *pid);

    /**
     * @brief      forward euler step discrete-time PID controller
     *
     * This function is equivalent to running a discrete control system generated from Maltab's
     * `pid` function:
     *
     *     sys = pid(kp,ki,kd,tf,ts,'IFormula','ForwardEuler','DFormula','ForwardEuler');
     *
     * @param[in]  des    Desired set value
     * @param[in]  meas   Measured value
     * @param      state  PID state (do not manually edit)
     * @param[in]  ts     Time-step in seconds
     * @param[in]  param  PID parameters (gains, saturation, etc.)
     *
     * @return     controller output
     */
    double ard_pid_run_fwd(const double des, const double meas, ArdPid *pid, const double ts);

    /**
     * @brief      backwards euler step discrete-time PID controller
     *
     * This function is equivalent to running a discrete control system generated from Maltab's
     * `pid` function:
     *
     *     sys = pid(kp,ki,kd,tf,ts,'IFormula','BackwardEuler','DFormula','BackwardEuler');
     *
     * @param[in]  des    Desired set value
     * @param[in]  meas   Measured value
     * @param      state  PID state (do not manually edit)
     * @param[in]  ts     Time-step in seconds
     * @param[in]  param  PID parameters (gains, saturation, etc.)
     *
     * @return     controller output
     */
    double ard_pid_run_back(const double des, const double meas, ArdPid *pid, const double ts);

    /**
     * @brief      trapezoidal discrete-time PID controller
     *
     * This function is equivalent to running a discrete control system generated from Maltab's
     * `pid` function:
     *
     *     sys = pid(kp,ki,kd,tf,ts,'IFormula','Trapezoidal','DFormula','Trapezoidal');
     *
     * @param[in]  des    Desired set value
     * @param[in]  meas   Measured value
     * @param      state  PID state (do not manually edit)
     * @param[in]  ts     Time-step in seconds
     * @param[in]  param  PID parameters (gains, saturation, etc.)
     *
     * @return     controller output
     */
    double ard_pid_run_trap(const double des, const double meas, ArdPid *pid, const double ts);

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
