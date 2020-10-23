
#ifndef ARD_ASCALE_H
#define ARD_ASCALE_H

#ifdef __cplusplus
#include <cmath>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct ArdScale
    {
        double scale_m;
        double scale_b;
        int input_min;
        int input_max;
    } ArdScale;

    /**
     * @brief      Set linear scale and offset for analog input
     *
     * @param      asc      Analog scaling object
     * @param[in]  in_min   Limit minimum for analog input
     * @param[in]  in_max   Limit maximum for analog input
     * @param[in]  out_min  Minimum output (scaled to output values)
     * @param[in]  out_max  Maximum output (scaled to output values)
     */
    int ard_scale_set(ArdScale *asc, const int in_min, const int in_max, const double out_min,
                      const double out_max);

    /**
     *
     * @param      asc   Analog scaling object
     *
     * @return     Scaled value from analog input
     */

    /**
     * @brief      Linearly scale and saturate analog signal
     *
     * @param      in    Input
     * @param      asc   Analog scaling object
     * @return double
     */
    double ard_scale_evaluate(const ArdScale *asc, int in);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

template <class T>
class ArdScaleIntegerType
{
   public:
    ArdTypeScale() : : scale_m_(0.0), scale_b_(0.0), input_min_(0.0), input_max_(0.0) {}

    ArdTypeScale(const double scale_m, const double scale_b, const T input_min, const T input_max)
        : scale_m_(scale_m), scale_b_(scale_b), input_min_(input_min), input_max_(input_max)
    {
    }

    int Set(const T in_min, const T in_max, const double out_min, const double out_max)
    {
        if (in_min == in_max) return -1;

        asc->input_min = in_min;
        asc->input_max = in_max;

        asc->scale_m = (out_max - out_min) / static_cast<double>(asc->input_max - asc->input_min);
        asc->scale_b = out_min;

        return 0;
    }

    double Evaluate(T in)
    {
        // saturate input
        if (in < asc->input_min_)
        {
            in = asc->input_min_;
        }
        else if (in > asc->input_max_)
        {
            in = asc->input_max_;
        }
        std::round(in - asc->input_min_) * asc->scale_m + asc->scale_b;
    }

   private:
    double scale_m_;
    double scale_b_;
    T input_min_;
    T input_max_;
};

typedef ArdScaleIntegerType<int> ArdScaleInt;
typedef ArdScaleIntegerType<unsigned int> ArdScaleUInt;

typedef ArdScaleIntegerType<int8_t> ArdScaleInt8;
typedef ArdScaleIntegerType<uint8_t> ArdScaleUInt8;

typedef ArdScaleIntegerType<int16_t> ArdScaleInt16;
typedef ArdScaleIntegerType<uint16_t> ArdScaleUInt16;

typedef ArdScaleIntegerType<int32_t> ArdScaleInt32;
typedef ArdScaleIntegerType<uint32_t> ArdScaleUInt32;

typedef ArdScaleIntegerType<int64_t> ArdScaleInt64;
typedef ArdScaleIntegerType<uint64_t> ArdScaleUInt64;

#endif

#endif
