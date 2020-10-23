
#include "ard_scale.h"
#include "math.h"

int ard_scale_set(ArdScale *asc, const int in_min, const int in_max, const double out_min,
                  const double out_max)
{
    if (in_min == in_max) return -1;
    asc->input_min = in_min;
    asc->input_max = in_max;

    asc->scale_m = (out_max - out_min) / ((double)(asc->input_max - asc->input_min));
    asc->scale_b = out_min;
    return 0;
}

double ard_scale_evaluate(const ArdScale *asc, int in)
{
    // saturate input
    if (in < asc->input_min)
    {
        in = asc->input_min;
    }
    else if (in > asc->input_max)
    {
        in = asc->input_max;
    }
    // scale and offset
    return round(in - asc->input_min) * asc->scale_m + asc->scale_b;
}
