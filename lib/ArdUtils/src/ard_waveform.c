
#include "ard_waveform.h"

#include "math.h"

double ard_waveform_wrap_pi(double x) {
    if (x > M_PI || x < -M_PI) {
        x = fmod(x + M_PI, 2.0 * M_PI);
        if (x < 0.0) {
            x += 2.0 * M_PI;
        } else {
            x -= M_PI;
        }
    }
    return x;
}

int ard_sinusoidal_update_amplitude(struct ArdSinusoidal *wave, const double amplitude)
{
    if (amplitude < ARD_WAVEFORM_NUMERICAL_TOLERANCE) {
        // check if amplitude too small
        return -1;
    } else {
        // set amplitude
        wave->amplitude = amplitude;
        return 0;
    }
}

int ard_sinusoidal_update_peak_speed(struct ArdSinusoidal *wave, const double v)
{
    if (v < ARD_WAVEFORM_NUMERICAL_TOLERANCE) {
        // check if velocity too small
        return -1;
    } else if (wave->amplitude < ARD_WAVEFORM_NUMERICAL_TOLERANCE) {
        // check if amplitude too small
        return -2;
    } else {
        // adjust phase to compensate for change in frequency
        const double freq = v / wave->amplitude;
        // wrap phase to +- pi
        wave->phase = ard_waveform_wrap_pi((wave->freq - freq) * wave->time + wave->phase);
        // set frequency
        wave->freq = freq;
        return 0;
    }
}

double ard_sinusoidal_evaluate_position(struct ArdSinusoidal *wave, const double time)
{
    wave->time = time;
    return wave->amplitude * sin(wave->freq * wave->time + wave->phase) + wave->offset;
}

double ard_sinusoidal_evaluate_velocity(struct ArdSinusoidal *wave, const double time)
{
    wave->time = time;
    return -wave->amplitude * wave->freq * cos(wave->freq * wave->time + wave->phase);
}

double ard_sinusoidal_evaluate_acceleration(struct ArdSinusoidal *wave, const double time)
{
    wave->time = time;
    return wave->amplitude * wave->freq * wave->freq * sin(wave->freq * wave->time + wave->phase);
}
