
#ifndef ARD_WAVEFORM_H
#define ARD_WAVEFORM_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARD_WAVEFORM_NUMERICAL_TOLERANCE 1.0e-8

#ifndef M_PI
#ifdef PI
#define M_PI PI
#else
#define M_PI 3.1415926535897932384626433832795
#endif
#endif

/**
 * @brief Wrap to +/- pi
 * 
 * @param x input angle
 * @return angle wrapped to +/- pi
 */
double ard_waveform_wrap_pi(double x);

/**
 * @brief Sinusoidal waveform
 * 
 * Sinusoidal trajectory generated in the form
 * 
 * \f[
 *     p(t) = A \sin{\omega t + \phi} + p_{mid}
 * \f]
 * 
 * where \f$A\f$ is the amplitude, $\omega$ is the frequency, $t$ is time, $\phi$ is the phase, and $p_{mid}$ is the midpoint offset.
 * 
 */
typedef struct ArdSinusoidal {
    /**
     * @brief Amplitude \f$A\f$
     * 
     */
    double amplitude;
    /**
     * @brief Frequency \f$\omega\f$ in radians/sec
     * 
     */
    double freq;
    /**
     * @brief Phase \f$\phi\f$ in radians
     * 
     */
    double phase;
    /**
     * @brief Offset \f$p_{mid}\f$ in scaled units
     * 
     */
    double offset;
    /**
     * @brief Time \f$t\f$ in seconds
     * 
     */
    double time;
} ArdSinusoidal;

int ard_sinusoidal_update_amplitude(
    struct ArdSinusoidal *wave, 
    const double amplitude);
    
int ard_sinusoidal_update_peak_speed(
    struct ArdSinusoidal *wave,
    const double v);

double ard_sinusoidal_evaluate_position(
    struct ArdSinusoidal *wave, const double time);

double ard_sinusoidal_evaluate_velocity(
    struct ArdSinusoidal *wave, const double time);

double ard_sinusoidal_evaluate_acceleration(
    struct ArdSinusoidal *wave, const double time);



#ifdef __cplusplus
}
#endif



#endif
