

#include "ard_encoder.h"

static int32_t pow2_int(uint8_t n)
{
    int32_t out = 0;
    for (uint8_t i = 0; i < n; i++) out *= 2;
    return out;
}

void ard_encoder_set_32bit(ArdEncoder *enc, const uint32_t counts_per_rev,
                                  const double value_per_rev)
{
    enc->value = 0.0;
    enc->count = 0;
    enc->value_per_rev = value_per_rev;
    enc->counts_per_rev = counts_per_rev;
    enc->counter_limit = INT32_MIN;
}

int ard_encoder_set(ArdEncoder *enc, const uint8_t counter_resolution_bits,
                           const uint32_t counts_per_rev, const double value_per_rev)
{
    if (counter_resolution_bits == 0) return -1;
    if (counter_resolution_bits > 32) return -2;

    enc->value = 0.0;
    enc->count = 0;
    enc->value_per_rev = value_per_rev;
    enc->counts_per_rev = counts_per_rev;

    if (counter_resolution_bits == 32)
    {
        enc->counter_limit = INT32_MIN;
    }
    else if (counter_resolution_bits == 16)
    {
        enc->counter_limit = INT16_MIN;
    }
    else if (counter_resolution_bits == 8)
    {
        enc->counter_limit = INT8_MIN;
    }
    else
    {
        enc->counter_limit = -pow2_int(counter_resolution_bits - 1);
    }

    return 0;
}

void ard_encoder_reset(ArdEncoder *enc, const int32_t reset_count, const double reset_value)
{
    enc->count = reset_count;
    enc->value = reset_value;
}

void ard_encoder_reset_absolute(ArdEncoder *enc, const double reset_value)
{
    enc->value = reset_value;
    enc->count = (int32_t)(enc->value * ((double)enc->counts_per_rev) / enc->value_per_rev);
}

double ard_encoder_decode(ArdEncoder *enc, const int32_t new_count)
{
    // change in count
    int32_t delta = new_count - enc->count;
    if (delta < enc->counter_limit)
    {
        delta = delta - 2 * enc->counter_limit;
    }
    else if (delta > -enc->counter_limit)
    {
        delta = delta + 2 * enc->counter_limit;
    }

    // save previous
    enc->count = new_count;

    // add to scaled value
    enc->value += ((((double)delta) * enc->value_per_rev) / ((double)enc->counts_per_rev));

    return enc->value;
}

int32_t ard_encoder_encode(ArdEncoder *enc, const double new_value)
{
    // update absolute counts
    const int32_t new_count =
        (int32_t)(new_value * ((double)enc->counts_per_rev) / enc->value_per_rev);
    // change in count
    const int32_t delta = new_count - enc->count;
    // save count
    enc->count = new_count;
    // update value
    enc->value = new_value;

    return delta;
}
