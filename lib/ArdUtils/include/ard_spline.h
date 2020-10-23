
/**
 * @file       ard_spline.h
 * @author     Kyle Chisholm (dev@kylechisholm.ca)
 * @brief      Quintic and Cubic spline generator
 *
 * @details
 *
 * See group @ref ArdSpline
 *
 */

#ifndef ARD_SPLINE_H
#define ARD_SPLINE_H

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
     * @defgroup   ArdSpline Spline Generator
     * @brief      Quintic and Cubic spline generator
     *
     * @{
     *
     */

#define ARD_SPLINE_MINIMUM_DURATION 0.1
#define ARD_SPLINE_TOLERANCE 1.0e-6

    typedef struct ArdSplinePva
    {
        double position;
        double velocity;
        double acceleration;
    } ArdSplinePva;

    typedef enum eArdSplineType
    {
        EARD_SPLINE_QUINTIC = 0,
        EARD_SPLINE_CUBIC
    } eArdSplineType;

    /**
     * @brief Spline polynomial
     *
     */
    typedef struct ArdSpline
    {
        /**
         * @brief Spline Coefficients
         *
         */
        double coeffs[6];
        /**
         * @brief Duration of spline
         *
         * It is suggested to use milliseconds as unit of time
         *
         */
        double duration;
        /**
         * @brief Type of spline interpolation
         *
         */
        eArdSplineType type;
    } ArdSpline;

    /**
     * @brief Generate spline polynomial
     * 
     * @param spline Spline object
     * @param start Initial position and rates
     * @param end Final position and rates
     * @param duration Duration of spline (any units)
     * @param type Type of spline
     * @return int Error code, returns 0 on success
     */
    int ard_spline_generate(ArdSpline *spline, const ArdSplinePva *start, const ArdSplinePva *end,
                            const double duration, const eArdSplineType type);

    /**
     * @brief Evaluate spline position, velocity, and acceleration
     * 
     * @param spline SPline object
     * @param t Time 
     * @param out Output position, velocity, acceleration
     */
    void ard_spline_evaluate(const ArdSpline *spline, double t, ArdSplinePva *out);

    /**
     * @brief Evaluate spline position
     * 
     * @param spline Spline object
     * @param t Trajectory time 
     * @return double Output position
     */
    double ard_spline_evaluate_position(const ArdSpline *spline, double t);

    /**
     * @brief Spline trajectory with discrete time update
     *
     */
    typedef struct ArdSplineTrajectory
    {
        /**
         * @brief spline object
         * 
         */
        ArdSpline spline;

        /**
         * @brief Step size in microseconds
         * 
         */
        double step_size;
        
        /**
         * @brief Current index of trajectory
         * 
         */
        uint16_t index;

        /**
         * @brief Number of steps in trajectory
         * 
         */
        uint16_t num_steps;

    } ArdSplineTrajectory;

    /**
     * @brief Generate spline polynomial
     * 
     * @param spline Spline object
     * @param start Initial position and rates
     * @param end Final position and rates
     * @param duration Duration of spline (seconds)
     * @param step_size Duration of spline (microseconds)
     * @param type Type of spline
     * @return int Error code, returns 0 on success
     */
    int ard_spline_traj_generate(ArdSplineTrajectory *traj, const ArdSplinePva *start, const ArdSplinePva *end,
                             double duration, const uint16_t step_size, const eArdSplineType type);

    /**
     * @brief Evaluate current position, velocity, and acceleration, then increment index
     * 
     * @param spline Spline object
     * @param out Output position, velocity, acceleration
     */
    void ard_spline_traj_evaluate(ArdSplineTrajectory *traj, ArdSplinePva *out);
    
    /**
     * @brief Evaluate current position, velocity, and acceleration, then increment index
     * 
     * @param spline Spline object
     * @return double Output position
     */
    double ard_spline_traj_evaluate_position(ArdSplineTrajectory *traj);

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
