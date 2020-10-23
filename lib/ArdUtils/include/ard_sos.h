
#ifndef ARD_SOS_H
#define ARD_SOS_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Second-order (biquadratic) IIR digital filter
     */
    typedef struct ArdSosFilter
    {
        /**
         * @brief state of 4 previous inputs for each sos, size (4 x num_scns)
         *
         */
        double *z;
        /**
         * @brief matrix, size (num_scns x 6)
         *
         */
        double *sos;
        /**
         * @brief  sos matrix rows (number of sections)
         *
         */
        size_t num_scns;
    } ArdSosFilter;

    /**
     * @brief  allocate memory and apply filter coefficients
     *
     * @param filt filter
     * @param sos sos filter coefficients
     * @param len length of sos array
     * @return int 0 on success, error code otherwise
     */
    int ard_sos_alloc(ArdSosFilter *filt, const double *sos, const size_t len);

    /**
     * @brief Free memory allocated to sos filter
     *
     * @param filt sos filter to free
     */
    void ard_sos_free(ArdSosFilter *filt);

    /* run filter */

    /**
     * @brief Run filter
     *
     * @param filt filter
     * @param x raw input
     * @return double filtered output
     */
    double ard_sos_run(ArdSosFilter *filt, double x);

#ifdef __cplusplus
}
#endif

#endif
