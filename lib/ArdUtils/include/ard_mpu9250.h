
#ifndef ARD_MPU9250_H
#define ARD_MPU9250_H

#include "Arduino.h"
#include "ard_bits.h"

/**
 * @brief rate divider conversion to microseconds with a base rate of 1000Hz
 *
 */
#define ARD_MPU9250_DIVIDER_TO_STEP_MICROS(x) (1000 * (x + 1))

#define ARD_MPU9250_IMU_READ_BYTES (7 * sizeof(uint16_t))
#define ARD_MPU9250_AK8963_READ_BYTES (3 * sizeof(uint16_t))
#define ARD_MPU9250_READ_BYTES (ARD_MPU9250_IMU_READ_BYTES + ARD_MPU9250_AK8963_READ_BYTES)

#define ARD_MPU9250_CALIBRATION_MAX_DURATION 10000U
#define ARD_MPU9250_CALIBRATION_MIN_DURATION 200U

#ifdef __cplusplus
extern "C"
{
#endif

    /* ***** */
    /* Enums */
    /* ***** */

    /**
     * @brief Gyro band-pass filter
     *
     */
    typedef enum eArdMpu9250Gband
    {
        EARD_MPU9250_GBAND_184HZ = 0,
        EARD_MPU9250_GBAND_92HZ = 1,
        EARD_MPU9250_GBAND_41HZ = 2,
        EARD_MPU9250_GBAND_20HZ = 3,
        EARD_MPU9250_GBAND_10HZ = 4,
        EARD_MPU9250_GBAND_5HZ = 5
    } eArdMpu9250Gband;

    /**
     * @brief Accelerometer band-pass filter
     *
     */
    typedef enum eArdMpu9250Aband
    {
        EARD_MPU9250_ABAND_420HZ = 0,
        EARD_MPU9250_ABAND_218HZ = 1,
        EARD_MPU9250_ABAND_99HZ = 2,
        EARD_MPU9250_ABAND_45HZ = 3,
        EARD_MPU9250_ABAND_21HZ = 4,
        EARD_MPU9250_ABAND_10HZ = 5,
        EARD_MPU9250_ABAND_5HZ = 6
    } eArdMpu9250Aband;

    /**
     * @brief Magnetometer read mode
     *
     */
    typedef enum eArdMpu9250Mrate
    {
        EARD_MPU9250_MRATE_8HZ = 0,
        EARD_MPU9250_MRATE_100HZ = 1
    } eArdMpu9250Mrate;

    /**
     * @brief Accelerometer scale
     *
     */
    typedef enum eArdMpu9250Ascale
    {
        EARD_MPU9250_AFS_2G = 0,
        EARD_MPU9250_AFS_4G = 1,
        EARD_MPU9250_AFS_8G = 2,
        EARD_MPU9250_AFS_16G = 3
    } eArdMpu9250Ascale;

    /**
     * @brief Gyro scale
     *
     */
    typedef enum eArdMpu9250Gscale
    {
        EARD_MPU9250_GFS_250DPS = 0,
        EARD_MPU9250_GFS_500DPS = 1,
        EARD_MPU9250_GFS_1000DPS = 2,
        EARD_MPU9250_GFS_2000DPS = 3
    } eArdMpu9250Gscale;

    /**
     * @brief Magnetometer resolution
     *
     */
    typedef enum eArdMpu9250Mprecision
    {
        EARD_MPU9250_MPRECISION_14BITS = 0,  // 0.6 mG per LSB
        EARD_MPU9250_MPRECISION_16BITS = 1   // 0.15 mG per LSB
    } eArdMpu9250Mprecision;

    typedef struct ArdMpu9250Config
    {
        uint8_t address;
        uint8_t rate_divider;
        eArdMpu9250Ascale accel_range;
        eArdMpu9250Aband accel_bandwidth;
        eArdMpu9250Gscale gyro_range;
        eArdMpu9250Gband gyro_bandwidth;
        eArdMpu9250Mrate mag_rate;
        eArdMpu9250Mprecision mag_precision;
    } ArdMpu9250Config;

    typedef struct __attribute__((__packed__)) ArdMpu9250ConfigPacked
    {
        uint8_t rate_divider;
        uint8_t accel_range;
        uint8_t accel_bandwidth;
        uint8_t gyro_range;
        uint8_t gyro_bandwidth;
        uint8_t mag_rate;
        uint8_t mag_precision;
    } ArdMpu9250ConfigPacked;

    typedef struct ArdMpu9250Values
    {
        float accel_x;
        float accel_y;
        float accel_z;
        float temperature;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float mag_x;
        float mag_y;
        float mag_z;
    } ArdMpu9250Values;

    typedef struct __attribute__((__packed__)) ArdMpu9250Counts
    {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t temperature;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
        int16_t mag_x;
        int16_t mag_y;
        int16_t mag_z;
    } ArdMpu9250Counts;

    typedef struct ArdMpu9250Fifo
    {
        bool accel;
        bool gyro;
        bool mag;
        bool temp;
        uint16_t frame_size;
        uint8_t buffer[21];
    } ArdMpu9250Fifo;

    /**
     * @brief IMU status flag during read
     *
     */
    typedef enum eArdMpu9250Status
    {
        ARD_MPU9250_OK = 0x00U,
        ARD_MPU9250_IMU_ERRMASK_NOTREADY = BIT_0,
        ARD_MPU9250_IMU_ERRMASK_READ_ERROR = BIT_1,
        ARD_MPU9250_IMU_ERRMASK_OVERFLOW = BIT_2,
        ARD_MPU9250_MAG_ERRMASK_OVERRUN = BIT_3,
        ARD_MPU9250_MAG_ERRMASK_READ_ERROR = BIT_4,
        ARD_MPU9250_MAG_ERRMASK_OVERFLOW = BIT_5
    } eArdMpu9250Status;

    /**
     * @brief IMU configuration result
     *
     */
    typedef enum eArdMpu9250ConfigResult
    {
        ARD_MPU9250_CONFIG_OK = 0,
        ARD_MPU9250_CONFIG_BAD_ID = 1,
        ARD_MPU9250_CONFIG_GBAND_INVALID = 2,
        ARD_MPU9250_CONFIG_GSCALE_INVALID = 3,
        ARD_MPU9250_CONFIG_ASCALE_INVALID = 4,
        ARD_MPU9250_CONFIG_ABAND_INVALID = 5,
        ARD_MPU9250_CONFIG_BAD_MAG_ID = 6,
        ARD_MPU9250_CONFIG_MAG_PRECISION_INVALID = 7,
        ARD_MPU9250_CONFIG_MAG_RATE_INVALID = 8
    } eArdMpu9250ConfigResult;

    typedef enum eArdMpu9250CalibrationResult
    {
        ARD_MPU9250_CALIBRATION_SUCCESS = 0,
        ARD_MPU9250_CALIBRATION_DURATION_TOO_LONG = 1,
        ARD_MPU9250_CALIBRATION_DURATION_TOO_SHORT = 2,
        ARD_MPU9250_CALIBRATION_NUMERICAL_FAULT = 3,
        ARD_MPU9250_CALIBRATION_CONFIG_ERROR = 4
    } eArdMpu9250CalibrationResult;

    typedef struct __attribute__((__packed__)) ArdMpu9250ScaleFactors
    {
        uint32_t time_step_microseconds;

        float accel_range;
        float temperature_scale;
        float temperature_offset;
        float gyro_range;
        float mag_range_x;
        float mag_range_y;
        float mag_range_z;

    } ArdMpu9250ScaleFactors;

    typedef struct __attribute__((__packed__)) ArdMpu9250Calibration
    {
        float accel_bias_x;
        float accel_bias_y;
        float accel_bias_z;

        float accel_scale_x;
        float accel_scale_y;
        float accel_scale_z;

        float gyro_bias_x;
        float gyro_bias_y;
        float gyro_bias_z;

        float mag_bias_x;
        float mag_bias_y;
        float mag_bias_z;

        float mag_scale_x;
        float mag_scale_y;
        float mag_scale_z;

    } ArdMpu9250Calibration;

    typedef struct ArdMpu9250
    {
        ArdMpu9250Config config;
        ArdMpu9250Counts counts;
        ArdMpu9250ScaleFactors scale_factors;
        ArdMpu9250Calibration calibration;
        ArdMpu9250Values values;
    } ArdMpu9250;

    /**
     * @brief Get default configuration
     *
     * @return ArdMpu9250Config
     */
    ArdMpu9250Config ard_mpu9250_default_config();

    /**
     * @brief Configure MPU9250
     *
     * @param imu
     * @param config
     * @return eArdMpu9250ConfigResult
     */
    eArdMpu9250ConfigResult ard_mpu9250_configure(ArdMpu9250 *imu, const ArdMpu9250Config *config);

    eArdMpu9250Status ard_mpu9250_read_counts(ArdMpu9250 *imu);
    void ard_mpu9250_scale_values(ArdMpu9250 *imu);

    eArdMpu9250CalibrationResult ard_mpu9250_calibrate_gyro(ArdMpu9250 *imu, uint32_t duration);
    eArdMpu9250CalibrationResult ard_mpu9250_calibrate_accel(ArdMpu9250 *imu, uint32_t duration);
    eArdMpu9250CalibrationResult ard_mpu9250_calibrate_mag(ArdMpu9250 *imu, uint32_t duration);

    void ard_mpu9250_reset_calibration(ArdMpu9250 *imu);
    void ard_mpu9250_load_calibration(ArdMpu9250 *imu, const size_t ee_index);
    void ard_mpu9250_save_calibration(ArdMpu9250 *imu, const size_t ee_index);

    void ard_mpu9250_fifo_enable(const uint8_t address, ArdMpu9250Fifo *fifo);
    int ard_mpu9250_fifo_read(const uint8_t address, ArdMpu9250Fifo *fifo, const uint16_t size_out,
                              int16_t *out, uint16_t *frames_read);

#ifdef __cplusplus
}
#endif

#endif

