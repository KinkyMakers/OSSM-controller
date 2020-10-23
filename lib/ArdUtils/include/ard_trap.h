
#ifndef ARD_TRAP_H
#define ARD_TRAP_H

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ARD_TRAPEZOID_NUMERICAL_TOLERANCE 1.0e-8

    /**
     * @brief Trapezoidal trajectory
     *
     * Trapezoidal trajectory generation.
     *
     */
    typedef struct ArdTrapezoidal
    {
        /**
         * @brief Controls step size
         *
         */
        double step_size;
        /**
         * @brief Maximum acceleration
         *
         */
        double max_acceleration;
        /**
         * @brief Maximum jerk
         *
         */
        double max_velocity;

        /**
         * @brief Set peak acceleration
         *
         */
        double peak_acceleration;

        /**
         * @brief Set peak velocity
         *
         */
        double peak_velocity;

        /**
         * @brief Initial position
         *
         */
        double initial_position;
        /**
         * @brief Position set point
         *
         */
        double final_position;

        /**
         * @brief Trapezoid start ramp
         *
         */
        double duration_ramp;

        /**
         * @brief trapezoid plateau
         *
         */
        double duration_plateau;
        /**
         * @brief Start of trajectory in microseconds
         *
         */
        uint32_t time_start;

        /**
         * @brief
         *
         */
        int8_t direction;

    } ArdTrapezoidal;

    int ard_trap_set_parameters(ArdTrapezoidal *trap, const double step_size,
                                const double max_velocity, const double max_acceleration);

    void ard_trap_reset(ArdTrapezoidal *trap, const uint32_t time_us, const double position);

    void ard_trap_generate(ArdTrapezoidal *trap, const uint32_t time_us,
                           const double initial_position, const double final_position);

    double ard_trap_evaluate(const ArdTrapezoidal *trap, const uint32_t time_us);

    // TODO: output position, velocity, acceleration
    // void ard_trap_evaluate_pva(const ArdTrapezoidal *trap, const uint32_t time_us,
    //                                     double *position, double *velocity, double
    //                                     *acceleration);

    // TODO: create trapezoidal velocity profile
    // /**
    //  * @brief Trapezoidal trajectory with velocity set point
    //  *
    //  * Trapezoidal trajectory generation.
    //  *
    //  */
    // typedef struct ArdVelocityTrapezoidal
    // {
    //     /**
    //      * @brief Controls step size
    //      *
    //      */
    //     double step_size;
    //     /**
    //      * @brief Maximum jerk
    //      *
    //      */
    //     double max_jerk;
    //     /**
    //      * @brief Maximum acceleration
    //      *
    //      */
    //     double max_acceleration;

    //     /**
    //      * @brief Set peak jerk
    //      *
    //      */
    //     double peak_jerk;
    //     /**
    //      * @brief Set peak acceleration
    //      *
    //      */
    //     double peak_acceleration;

    //     /**
    //      * @brief Velocity set point
    //      *
    //      */
    //     double desired_velocity;

    //     /**
    //      * @brief Trapezoid start ramp
    //      *
    //      */
    //     double duration_start;

    //     /**
    //      * @brief trapezoid plateau
    //      *
    //      */
    //     double duration_plateau;

    //     /**
    //      * @brief trapezoid end ramp
    //      *
    //      */
    //     double duration_end;

    //     /**
    //      * @brief Initial position
    //      *
    //      */
    //     double initial_position;

    //     /**
    //      * @brief Initial velocity
    //      *
    //      */
    //     double initial_velocity;

    //     /**
    //      * @brief Initial acceleration
    //      *
    //      */
    //     double initial_acceleration;

    //     /**
    //      * @brief Start of trajectory
    //      *
    //      */
    //     uint32_t time_start;

    // } ArdVelocityTrapezoidal;

    // int ard_trap_velocity_set_parameters(ArdPositionTrapezoidal *trap, const double step_size,
    //                                      const double max_acceleration, const double max_jerk);

    // void ard_trap_velocity_reset(ArdPositionTrapezoidal *trap, const uint32_t t,
    //                              const double position);

    // void ard_trap_velocity_set_desired_velocity(ArdPositionTrapezoidal *trap, const uint32_t t,
    //                                             const double velocity);

    // // void ard_trap_velocity_set_desired_velocity_max_jerk(ArdTrapezoidal *trap, const double
    // time,
    // // const double velocity, const double jerk);

    // double ard_trap_velocity_evaluate(const ArdPositionTrapezoidal *trap, const uint32_t t);

    // void ard_trap_velocity_evaluate_pva(const ArdPositionTrapezoidal *trap, const uint32_t t,
    //                                     double *position, double *velocity, double
    //                                     *acceleration);

#ifdef __cplusplus
}
#endif

#endif
