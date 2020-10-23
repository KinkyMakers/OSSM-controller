
#include "ard_mpu9250.h"

#include "ard_i2c.h"

/* ********* */
/* Registers */
/* ********* */

// address
static const uint8_t ARD_MPU9250_ADDRESS_ADO_0 = 0x68u;
static const uint8_t ARD_MPU9250_ADDRESS_ADO_1 = 0x69u;
static const uint8_t ARD_MPU9250_ID = 0x71u;

// who am i
static const uint8_t ARD_MPU9250_WHO_AM_I = 0x75u;

// configuration IMU
static const uint8_t ARD_MPU9250_PWR_MGMT_1 = 0x6Bu;
static const uint8_t ARD_MPU9250_CONFIG = 0x1Au;
static const uint8_t ARD_MPU9250_GYRO_CONFIG = 0x1Bu;
static const uint8_t ARD_MPU9250_ACCEL_CONFIG = 0x1Cu;
static const uint8_t ARD_MPU9250_ACCEL_CONFIG2 = 0x1Du;
static const uint8_t ARD_MPU9250_SMPLRT_DIV = 0x19u;
static const uint8_t ARD_MPU9250_INT_PIN_CFG = 0x37u;
static const uint8_t ARD_MPU9250_INT_ENABLE = 0x38u;

// Master
static const uint8_t ARD_MPU9250_USER_CTRL = 0x6Au;
static const uint8_t ARD_MPU9250_I2C_MST_EN = 0x20u;

// data read IMU
static const uint8_t ARD_MPU9250_READ_STATUS_INT = 0x3Au;
static const uint8_t ARD_MPU9250_READ_START_REGISTER = 0x3Bu;

// magnetomerter AK8963
static const uint8_t ARD_MPU9250_AK8963_ADDRESS = 0x0Cu;
static const uint8_t ARD_MPU9250_AK8963_WHO_AM_I = 0x00u;
static const uint8_t ARD_MPU9250_AK8963_ID = 0x48u;

// configuration mag
static const uint8_t ARD_MPU9250_AK8963_CNTL1 = 0x0Au;
static const uint8_t ARD_MPU9250_AK8963_ASAX = 0x10u;

// data read mag
static const uint8_t ARD_MPU9250_AK8963_READ_STATUS_ST1 = 0x02u;
static const uint8_t ARD_MPU9250_AK8963_READ_START_REGISTER = 0x03u;
static const uint8_t ARD_MPU9250_AK8963_READ_STATUS_ST2 = 0x09u;

// FIFO
static const uint8_t ARD_MPU9250_FIFO_EN = 0x23u;
static const uint8_t ARD_MPU9250_FIFO_TEMP = 0x80u;
static const uint8_t ARD_MPU9250_FIFO_GYRO = 0x70u;
static const uint8_t ARD_MPU9250_FIFO_ACCEL = 0x08u;
static const uint8_t ARD_MPU9250_FIFO_MAG = 0x01u;
static const uint8_t ARD_MPU9250_FIFO_COUNT = 0x72u;
static const uint8_t ARD_MPU9250_FIFO_READ = 0x74u;

/* **************** */
/* Static functions */
/* **************** */

static float ard_mpu9250_range_gyro(const eArdMpu9250Gscale gyro_scale);
static float ard_mpu9250_range_acc(const eArdMpu9250Ascale acc_scale);

static int ard_mpu9250_range_mag(const eArdMpu9250Mprecision config_mag_precision,
                                 const uint8_t *restrict mag_cal_counts, float *mag_range_x,
                                 float *mag_range_y, float *mag_range_z);

static float ard_mpu9250_range_gyro(const eArdMpu9250Gscale gyro_scale)
{
    if (gyro_scale == EARD_MPU9250_GFS_250DPS)
    {
        return 250.0 / 32768.0;
    }
    else if (gyro_scale == EARD_MPU9250_GFS_500DPS)
    {
        return 500.0 / 32768.0;
    }
    else if (gyro_scale == EARD_MPU9250_GFS_1000DPS)
    {
        return 1000.0 / 32768.0;
    }
    else if (gyro_scale == EARD_MPU9250_GFS_2000DPS)
    {
        return 2000.0 / 32768.0;
    }
    else
    {
        return -1.0;
    }
}

static float ard_mpu9250_range_acc(const eArdMpu9250Ascale acc_scale)
{
    if (acc_scale == EARD_MPU9250_AFS_2G)
    {
        return 2.0 / 32768.0;
    }
    else if (acc_scale == EARD_MPU9250_AFS_4G)
    {
        return 4.0 / 32768.0;
    }
    else if (acc_scale == EARD_MPU9250_AFS_8G)
    {
        return 8.0 / 32768.0;
    }
    else if (acc_scale == EARD_MPU9250_AFS_16G)
    {
        return 16.0 / 32768.0;
    }
    else
    {
        return -1.0;
    }
}

static int ard_mpu9250_range_mag(const eArdMpu9250Mprecision config_mag_precision,
                                 const uint8_t *restrict mag_cal_counts, float *mag_range_x,
                                 float *mag_range_y, float *mag_range_z)
{
    float res_mag = 0.0;
    if (config_mag_precision == EARD_MPU9250_MPRECISION_14BITS)
    {
        res_mag = 10. * 0.6;  // milliGauss / LSB
    }
    else if (config_mag_precision == EARD_MPU9250_MPRECISION_16BITS)
    {
        res_mag = 10. * 4912.0 / 32760.0;  // milliGauss / LSB
    }
    else
    {
        return -1;
    }

    (*mag_range_x) = ((((float)mag_cal_counts[0]) - 128.0) / 256.0 + 1.0) * res_mag;
    (*mag_range_y) = ((((float)mag_cal_counts[1]) - 128.0) / 256.0 + 1.0) * res_mag;
    (*mag_range_z) = ((((float)mag_cal_counts[2]) - 128.0) / 256.0 + 1.0) * res_mag;

    return 0;
}

/* ************** */
/* Implementation */
/* ************** */

ArdMpu9250Config ard_mpu9250_default_config()
{
    ArdMpu9250Config config;

    // default address
    config.address = ARD_MPU9250_ADDRESS_ADO_0;

    // 50Hz
    config.rate_divider = 19;

    config.accel_range = EARD_MPU9250_AFS_2G;
    config.accel_bandwidth = EARD_MPU9250_ABAND_21HZ;

    config.gyro_range = EARD_MPU9250_GFS_250DPS;
    config.gyro_bandwidth = EARD_MPU9250_GBAND_20HZ;

    config.mag_rate = EARD_MPU9250_MRATE_8HZ;
    config.mag_precision = EARD_MPU9250_MPRECISION_16BITS;
}

eArdMpu9250ConfigResult ard_mpu9250_configure(ArdMpu9250 *imu, const ArdMpu9250Config *config)
{
    // work variable to set registers
    uint8_t setting_value = 0;

    /*
        PWR_MGMT_1
        Wake up
     */

    // Clear sleep mode, enable all sensors
    ard_i2c_master_write_register(config->address, ARD_MPU9250_PWR_MGMT_1, 0x00);
    delay(100);

    // check if device is responding and ID'd, expected ID is 0x71 or 0x73
    uint8_t whoami = ard_i2c_master_read_byte(config->address, ARD_MPU9250_WHO_AM_I);
    if (whoami != ARD_MPU9250_ID) return ARD_MPU9250_CONFIG_BAD_ID;

    /*
        PWR_MGMT_1
        Set timer clock source
     */

    // Auto select clock source to be PLL gyroscope reference
    // CLKSEL [0] is ON
    ard_i2c_master_write_register(config->address, ARD_MPU9250_PWR_MGMT_1, 0x01);
    delay(100);

    /*
        CONFIG
        Gyro bandwidth
     */

    // set fifo to 0 (new data over-writes old), disable FSYNC,
    // and set digital low-pass filter bandwidth for gyro
    if (config->gyro_bandwidth == EARD_MPU9250_GBAND_184HZ)
    {
        setting_value = 0x01;
    }
    else if (config->gyro_bandwidth == EARD_MPU9250_GBAND_92HZ)
    {
        setting_value = 0x02;
    }
    else if (config->gyro_bandwidth == EARD_MPU9250_GBAND_41HZ)
    {
        setting_value = 0x03;
    }
    else if (config->gyro_bandwidth == EARD_MPU9250_GBAND_20HZ)
    {
        setting_value = 0x04;
    }
    else if (config->gyro_bandwidth == EARD_MPU9250_GBAND_10HZ)
    {
        setting_value = 0x05;
    }
    else if (config->gyro_bandwidth == EARD_MPU9250_GBAND_5HZ)
    {
        setting_value = 0x06;
    }
    else
    {
        return ARD_MPU9250_CONFIG_GBAND_INVALID;
    }
    // DLPF_CFG [1:0]
    ard_i2c_master_write_register(config->address, ARD_MPU9250_CONFIG, setting_value);

    /*
        Output sample rate divider
     */

    // Set output sample rate = 1000 Hz / (config->rate_divider + 1)
    ard_i2c_master_write_register(config->address, ARD_MPU9250_SMPLRT_DIV, config->rate_divider);

    /*
        GYRO_CONFIG
        Gyro scale
     */

    // Set gyroscope full scale range
    if (config->gyro_range == EARD_MPU9250_GFS_250DPS)
    {
        setting_value = 0x00;
    }
    else if (config->gyro_range == EARD_MPU9250_GFS_500DPS)
    {
        setting_value = 0x01;
    }
    else if (config->gyro_range == EARD_MPU9250_GFS_1000DPS)
    {
        setting_value = 0x02;
    }
    else if (config->gyro_range == EARD_MPU9250_GFS_2000DPS)
    {
        setting_value = 0x03;
    }
    else
    {
        return ARD_MPU9250_CONFIG_GSCALE_INVALID;
    }
    // Preserve self-test bits
    int gyro_config = (int)ard_i2c_master_read_byte(config->address, ARD_MPU9250_GYRO_CONFIG);
    // Clear fchoice bits [1:0]
    gyro_config = gyro_config & ~(BIT_MASK_00000011);
    // Clear full scale bits GYRO_FS_SEL [4:3]
    gyro_config = gyro_config & ~(BIT_MASK_00000011 << 3);
    // Set full scale range GYRO_FS_SEL [4:3] for the gyro
    gyro_config = gyro_config | (setting_value << 3);
    // write gyro config settings
    ard_i2c_master_write_register(config->address, ARD_MPU9250_GYRO_CONFIG, (uint8_t)gyro_config);

    /*
        ACCEL_CONFIG
        accelerometer scale
     */

    // Set accelerometer full-scale range configuration
    if (config->accel_range == EARD_MPU9250_AFS_2G)
    {
        setting_value = 0x00;
    }
    else if (config->accel_range == EARD_MPU9250_AFS_4G)
    {
        setting_value = 0x01;
    }
    else if (config->accel_range == EARD_MPU9250_AFS_8G)
    {
        setting_value = 0x02;
    }
    else if (config->accel_range == EARD_MPU9250_AFS_16G)
    {
        setting_value = 0x03;
    }
    else
    {
        return ARD_MPU9250_CONFIG_ASCALE_INVALID;
    }
    // Preserve self-test bits
    int acc_config = (int)ard_i2c_master_read_byte(config->address, ARD_MPU9250_ACCEL_CONFIG);
    // Clear full scale bits ACCEL_FS_SEL [4:3]
    acc_config = acc_config & ~(BIT_MASK_00000011 << 3);
    // Set full scale range for the acc ACCEL_FS_SEL [4:3]
    acc_config = acc_config | setting_value << 3;
    // write acc config settings
    ard_i2c_master_write_register(config->address, ARD_MPU9250_ACCEL_CONFIG, (uint8_t)acc_config);

    /*
        ACCEL_CONFIG2
        accelerometer low-pass filter bandwidth
     */

    // and set digital low-pass filter bandwidth for acc
    if (config->accel_bandwidth == EARD_MPU9250_ABAND_420HZ)
    {
        setting_value = 0x07;
    }
    else if (config->accel_bandwidth == EARD_MPU9250_ABAND_218HZ)
    {
        setting_value = 0x01;
    }
    else if (config->accel_bandwidth == EARD_MPU9250_ABAND_99HZ)
    {
        setting_value = 0x02;
    }
    else if (config->accel_bandwidth == EARD_MPU9250_ABAND_45HZ)
    {
        setting_value = 0x03;
    }
    else if (config->accel_bandwidth == EARD_MPU9250_ABAND_21HZ)
    {
        setting_value = 0x04;
    }
    else if (config->accel_bandwidth == EARD_MPU9250_ABAND_10HZ)
    {
        setting_value = 0x05;
    }
    else if (config->accel_bandwidth == EARD_MPU9250_ABAND_5HZ)
    {
        setting_value = 0x06;
    }
    else
    {
        return ARD_MPU9250_CONFIG_ABAND_INVALID;
    }
    // Preserve self-test bits
    acc_config = ard_i2c_master_read_byte(config->address, ARD_MPU9250_ACCEL_CONFIG2);
    // Clear fchoice bits FCHOICE_B [3:2] and low pass filter bits ACCEL_DLPF_CFG
    // [1:0]
    acc_config = acc_config & ~(BIT_MASK_00001111);
    // Set bandwisth (low pass filter) ACCEL_DLPF_CFG [1:0] for the accelerometer
    acc_config = acc_config | setting_value;
    // write acc config settings
    ard_i2c_master_write_register(config->address, ARD_MPU9250_ACCEL_CONFIG2, (uint8_t)acc_config);

    /*
        INT_PIN_CFG
        i2c pass through and latch interrupt
     */

    setting_value = 0x00;
    // bypass i2C BYPASS_EN [1]
    setting_value = setting_value | BIT_1;
    // latch interrupt until cleared LATCH_INT_EN [5]
    setting_value = setting_value | BIT_5;
    // write settings
    ard_i2c_master_write_register(config->address, ARD_MPU9250_INT_PIN_CFG, setting_value);

    /*
        INT_ENABLE
        enable data ready interrupt
     */

    setting_value = 0x00;
    // bypass i2C RAW_RDY_EN [0]
    setting_value = setting_value | BIT_0;
    // write settings
    ard_i2c_master_write_register(config->address, ARD_MPU9250_INT_ENABLE, setting_value);
    delay(20);

    //   writeByte(MPUnum, USER_CTRL, 0x20);          // Enable I2C Master mode
    //   writeByte(MPUnum, I2C_MST_CTRL, 0x1D);       // I2C configuration STOP after each
    //   transaction, master I2C bus at 400 KHz writeByte(MPUnum, I2C_MST_DELAY_CTRL, 0x81); // Use
    //   blocking data retrieval and enable delay for mag sample rate mismatch writeByte(MPUnum,
    //   I2C_SLV4_CTRL, 0x01);      // Delay mag data retrieval to once every other accel/gyro data
    //   sample

    /* *************** *
     *   magnetometer  *
     * *************** *
     */

    /*
        AK8963_CNTL
        power down magnetometer
     */

    ard_i2c_master_write_register(ARD_MPU9250_AK8963_ADDRESS, ARD_MPU9250_AK8963_CNTL1, 0x00);
    delay(100);  // long wait between AK8963 mode changes

    // check if device is responding and ID
    whoami = ard_i2c_master_read_byte(ARD_MPU9250_AK8963_ADDRESS, ARD_MPU9250_AK8963_WHO_AM_I);
    if (whoami != ARD_MPU9250_AK8963_ID) return ARD_MPU9250_CONFIG_BAD_MAG_ID;

    /*
        AK8963_CNTL
        enter ROM read mode
     */

    ard_i2c_master_write_register(ARD_MPU9250_AK8963_ADDRESS, ARD_MPU9250_AK8963_CNTL1,
                                  BIT_MASK_00001111);
    delay(20);

    /*
        AK8963_ASAX
        factory calibrtion data (sensitivity scaling)
     */

    uint8_t gyro_cal_counts[3];
    ard_i2c_master_read_begin(ARD_MPU9250_AK8963_ADDRESS, ARD_MPU9250_AK8963_ASAX);
    ard_i2c_master_read(ARD_MPU9250_AK8963_ADDRESS, gyro_cal_counts, 3);

    /*
        AK8963_CNTL
        power down magnetometer
     */

    ard_i2c_master_write_register(ARD_MPU9250_AK8963_ADDRESS, ARD_MPU9250_AK8963_CNTL1, 0x00);
    delay(20);

    /*
        AK8963_CNTL
        Specify magnetometer sensor full scale and continuous measurement
       frequency
     */

    setting_value = 0x00;
    if (config->mag_precision == EARD_MPU9250_MPRECISION_14BITS)
    {
        // 1 T = 10000 G, 1uT = 10mG
        // scale at 14-bit (0.6 uT/ LSB)
        // [4] is off (bit 4 already set to 0)
    }
    else if (config->mag_precision == EARD_MPU9250_MPRECISION_16BITS)
    {
        // scale at 16-bit (0.15 uT/ LSB)
        // [4] is ON
        setting_value = setting_value | BIT_4;
    }
    else
    {
        return ARD_MPU9250_CONFIG_MAG_PRECISION_INVALID;
    }

    // continuous measurement mode
    setting_value = setting_value | BIT_1;
    if (config->mag_rate == EARD_MPU9250_MRATE_8HZ)
    {
        // 8 Hz bit [2] is OFF
        // do nothing (bit 2 already set to 0)
    }
    else if (config->mag_rate == EARD_MPU9250_MRATE_100HZ)
    {
        // 100 Hz bit [2] is ON
        setting_value = setting_value | BIT_2;
    }
    else
    {
        return ARD_MPU9250_CONFIG_MAG_RATE_INVALID;
    }

    // write settings
    ard_i2c_master_write_register(ARD_MPU9250_AK8963_ADDRESS, ARD_MPU9250_AK8963_CNTL1,
                                  setting_value);
    delay(20);

    // save configuration
    imu->config = (*config);

    // get range scale factors
    imu->scale_factors.time_step_microseconds = 1000u * (((uint32_t)config->rate_divider) + 1u);
    imu->scale_factors.accel_range = ard_mpu9250_range_acc(config->accel_range);
    imu->scale_factors.gyro_range = ard_mpu9250_range_gyro(config->gyro_range);
    ard_mpu9250_range_mag(config->mag_precision, gyro_cal_counts, &imu->scale_factors.mag_range_x,
                          &imu->scale_factors.mag_range_y, &imu->scale_factors.mag_range_z);
    // temperature
    imu->scale_factors.temperature_scale = 333.87f;
    imu->scale_factors.temperature_offset = 21.0f;

    return ARD_MPU9250_CONFIG_OK;
}

/* ********* */
/* Data Read */
/* ********* */

int ard_mpu9250_fifo_read(const uint8_t address, ArdMpu9250Fifo *fifo, const uint16_t size_out, int16_t *out, uint16_t *frames_read)
{
    (*frames_read) = 0;
    // get the fifo size
    uint8_t buffer[2];
    ard_i2c_master_read(ARD_MPU9250_FIFO_COUNT, 2, buffer);
    const uint16_t fifo_size = (((uint16_t)(buffer[0] & 0x0F)) << 8) + (((uint16_t)buffer[1]));

    // write index
    uint16_t write_index = 0;
    uint16_t out_frame_size = (fifo->mag ? fifo->frame_size - 1 : fifo->frame_size) / sizeof(uint16_t);
    // read and parse the buffer
    for (uint16_t k = 0; k < fifo_size / fifo->frame_size; k++)
    {
        if ((write_index + out_frame_size) >= size_out) {
            return -1;
        }
        // FIFO read begin
        if (ard_i2c_master_read_begin(address, ARD_MPU9250_FIFO_READ) != 0) return -2;
        // Read frame
        uint16_t read_index = 0;
        size_t bytes_read = ard_i2c_master_read(address, fifo->buffer, fifo->frame_size);
        if (bytes_read != fifo->frame_size) return -3;
        // data
        if (fifo->accel) {
            // combine into 16 bit values
            out[write_index] = (((int16_t)fifo->buffer[read_index]) << 8) | fifo->buffer[read_index + 1];  
            out[write_index + 1] = (((int16_t)fifo->buffer[read_index + 2]) << 8) | fifo->buffer[read_index + 3];
            out[write_index + 2] = (((int16_t)fifo->buffer[read_index + 4]) << 8) | fifo->buffer[read_index + 5];
            write_index += 3;
            read_index += 6;
        }
        if (fifo->temp) {
            // combine into 16 bit values
            out[write_index] = (((int16_t)fifo->buffer[read_index]) << 8) | fifo->buffer[read_index + 1];
            write_index += 1;
            read_index += 2;
        }
        if (fifo->gyro) {
            // combine into 16 bit values
            out[write_index] = (((int16_t)fifo->buffer[read_index]) << 8) | fifo->buffer[read_index + 1];  
            out[write_index + 1] = (((int16_t)fifo->buffer[read_index + 2]) << 8) | fifo->buffer[read_index + 3];
            out[write_index + 2] = (((int16_t)fifo->buffer[read_index + 4]) << 8) | fifo->buffer[read_index + 5];
            write_index += 3;
            read_index += 6;
        }
        if (fifo->mag) {
            // combine into 16 bit values
            out[write_index] = (((int16_t)fifo->buffer[read_index]) << 8) | fifo->buffer[read_index + 1];  
            out[write_index + 1] = (((int16_t)fifo->buffer[read_index + 2]) << 8) | fifo->buffer[read_index + 3];
            out[write_index + 2] = (((int16_t)fifo->buffer[read_index + 4]) << 8) | fifo->buffer[read_index + 5];
            write_index += 3;
            read_index;
        }
        (*frames_read) += 1;
    }
    return 0;
}

void ard_mpu9250_fifo_enable(const uint8_t address, ArdMpu9250Fifo *fifo)
{
    // enable i2c master and fifo
    // BIT_5: FIFO_EN
    // BIT_6: I2C_MST_EN
    uint8_t setting = BIT_5 | ARD_MPU9250_I2C_MST_EN;
    ard_i2c_master_write_register(address, ARD_MPU9250_USER_CTRL, setting);
    delay(10);

    // set fifo
    fifo->frame_size = 0;
    setting = 0;
    if (fifo->accel) {
        setting |= ARD_MPU9250_FIFO_ACCEL;
        fifo->frame_size += 6;
    }
    if (fifo->gyro) {
        setting |= ARD_MPU9250_FIFO_GYRO;
        fifo->frame_size += 6;
    }
    if (fifo->mag) {
        setting |= ARD_MPU9250_FIFO_MAG;
        fifo->frame_size += 7;
    }
    if (fifo->temp) {
        setting |= ARD_MPU9250_FIFO_TEMP;
        fifo->frame_size += 2;
    }
    ard_i2c_master_write_register(address, ARD_MPU9250_FIFO_EN, setting);

}

eArdMpu9250Status ard_mpu9250_read_counts(ArdMpu9250 *imu)
{
    // out array is 20 bytes, (10 int16_t values)

    uint8_t raw[ARD_MPU9250_READ_BYTES + 1];
    size_t bytes_read = 0;
    uint8_t error_bitmask = ARD_MPU9250_OK;

    // order is AcX (int16_t)  | 2 bytes | index 0 | bytes 0-1
    //          AcY (int16_t)  | 2 bytes | index 1 | bytes 2-3
    //          AcZ (int16_t)  | 2 bytes | index 2 | bytes 4-5
    //          Tmp (int16_t)  | 2 bytes | index 3 | bytes 6-7
    //          GyX (int16_t)  | 2 bytes | index 4 | bytes 8-9
    //          GyY (int16_t)  | 2 bytes | index 5 | bytes 10-11
    //          GyZ (int16_t)  | 2 bytes | index 6 | bytes 12-13
    //          MagX (int16_t) | 2 bytes | index 7 | bytes 14-15
    //          MagY (int16_t) | 2 bytes | index 8 | bytes 16-17
    //          MagZ (int16_t) | 2 bytes | index 9 | bytes 18-19

    // Check if IMU data ready bit is ready, otherwise return
    if (!(BIT_0 & ard_i2c_master_read_byte(imu->config.address, ARD_MPU9250_READ_STATUS_INT)))
    {
        return (error_bitmask | ARD_MPU9250_IMU_ERRMASK_NOTREADY);
    }

    // IMU read begin
    if (ard_i2c_master_read_begin(imu->config.address, ARD_MPU9250_READ_START_REGISTER) != 0)
    {
        return (error_bitmask | ARD_MPU9250_IMU_ERRMASK_READ_ERROR);
    }

    // read and swap bytes
    bytes_read = ard_i2c_master_read(imu->config.address, raw, ARD_MPU9250_IMU_READ_BYTES);
    if (bytes_read != ARD_MPU9250_IMU_READ_BYTES)
    {
        return (error_bitmask | ARD_MPU9250_IMU_ERRMASK_READ_ERROR);
    }

    // acc/temp/gyro OK

    // check mag overrun in continuous mode
    if (BIT_0 &
        ard_i2c_master_read_byte(ARD_MPU9250_AK8963_ADDRESS, ARD_MPU9250_AK8963_READ_STATUS_ST1))
    {
        error_bitmask = error_bitmask | ARD_MPU9250_MAG_ERRMASK_OVERRUN;
    }

    // MAG read begin
    if (ard_i2c_master_read_begin(ARD_MPU9250_AK8963_ADDRESS,
                                  ARD_MPU9250_AK8963_READ_START_REGISTER) != 0)
    {
        return (error_bitmask | ARD_MPU9250_MAG_ERRMASK_READ_ERROR);
    }

    // Read the six raw data and ST2 registers sequentially into data array
    bytes_read = ard_i2c_master_read(ARD_MPU9250_AK8963_ADDRESS, &raw[ARD_MPU9250_IMU_READ_BYTES],
                                     ARD_MPU9250_AK8963_READ_BYTES + 1);

    // check if not enough bits read
    if (bytes_read != ARD_MPU9250_AK8963_READ_BYTES + 1)
    {
        return (error_bitmask | ARD_MPU9250_MAG_ERRMASK_READ_ERROR);
    }

    // overflow
    if (raw[ARD_MPU9250_READ_BYTES] & BIT_3)
    {
        error_bitmask = error_bitmask | ARD_MPU9250_MAG_ERRMASK_OVERFLOW;
    }

    // Swap MSB and LSB into a signed 16-bit value (little endian)
    size_t index = 0;
    imu->counts.accel_x = ((int16_t)raw[1] << 8) | raw[0];
    imu->counts.accel_y = ((int16_t)raw[3] << 8) | raw[2];
    imu->counts.accel_z = ((int16_t)raw[5] << 8) | raw[4];
    imu->counts.temperature = ((int16_t)raw[7] << 8) | raw[6];
    imu->counts.gyro_x = ((int16_t)raw[9] << 8) | raw[8];
    imu->counts.gyro_y = ((int16_t)raw[11] << 8) | raw[10];
    imu->counts.gyro_z = ((int16_t)raw[13] << 8) | raw[12];
    imu->counts.mag_x = ((int16_t)raw[15] << 8) | raw[14];
    imu->counts.mag_y = ((int16_t)raw[17] << 8) | raw[16];
    imu->counts.mag_z = ((int16_t)raw[19] << 8) | raw[18];

    return error_bitmask;
}

void ard_mpu9250_scale_values(ArdMpu9250 *imu)
{
    imu->values.accel_x = (((float)imu->counts.accel_x) * imu->scale_factors.accel_range -
                           imu->calibration.accel_bias_x) *
                          imu->calibration.accel_scale_x;
    imu->values.accel_y = (((float)imu->counts.accel_y) * imu->scale_factors.accel_range -
                           imu->calibration.accel_bias_y) *
                          imu->calibration.accel_scale_y;
    imu->values.accel_z = (((float)imu->counts.accel_z) * imu->scale_factors.accel_range -
                           imu->calibration.accel_bias_z) *
                          imu->calibration.accel_scale_z;

    imu->values.temperature =
        (((float)imu->counts.temperature) - imu->scale_factors.temperature_offset) /
            imu->scale_factors.temperature_scale +
        imu->scale_factors.temperature_offset;

    imu->values.gyro_x =
        ((float)imu->counts.gyro_x) * imu->scale_factors.gyro_range - imu->calibration.gyro_bias_x;
    imu->values.gyro_y =
        ((float)imu->counts.gyro_y) * imu->scale_factors.gyro_range - imu->calibration.gyro_bias_y;
    imu->values.gyro_z =
        ((float)imu->counts.gyro_z) * imu->scale_factors.gyro_range - imu->calibration.gyro_bias_z;

    imu->values.mag_x = (((float)imu->counts.mag_x) * imu->scale_factors.mag_range_x -
                         imu->calibration.mag_bias_x) *
                        imu->calibration.mag_scale_x;
    imu->values.mag_y = (((float)imu->counts.mag_y) * imu->scale_factors.mag_range_y -
                         imu->calibration.mag_bias_y) *
                        imu->calibration.mag_scale_y;
    imu->values.mag_z = (((float)imu->counts.mag_z) * imu->scale_factors.mag_range_z -
                         imu->calibration.mag_bias_z) *
                        imu->calibration.mag_scale_z;
}

/* *********** */
/* Calibration */
/* *********** */

void ard_mpu9250_reset_calibration(ArdMpu9250 *imu)
{
    imu->calibration.accel_bias_x = 0.0;
    imu->calibration.accel_bias_y = 0.0;
    imu->calibration.accel_bias_z = 0.0;

    imu->calibration.accel_scale_x = 1.0;
    imu->calibration.accel_scale_y = 1.0;
    imu->calibration.accel_scale_z = 1.0;

    imu->calibration.gyro_bias_x = 0.0;
    imu->calibration.gyro_bias_y = 0.0;
    imu->calibration.gyro_bias_z = 0.0;

    imu->calibration.mag_bias_x = 0.0;
    imu->calibration.mag_bias_y = 0.0;
    imu->calibration.mag_bias_z = 0.0;

    imu->calibration.mag_scale_x = 1.0;
    imu->calibration.mag_scale_y = 1.0;
    imu->calibration.mag_scale_z = 1.0;
}

// Calibrate
//-----------

static ArdMpu9250Config ard_mpu9250_calibration_config(const uint8_t address)
{
    ArdMpu9250Config config;

    // default address
    config.address = address;

    // 1000Hz
    config.rate_divider = 0;

    config.accel_range = EARD_MPU9250_AFS_2G;
    config.accel_bandwidth = EARD_MPU9250_ABAND_420HZ;

    config.gyro_range = EARD_MPU9250_GFS_250DPS;
    config.gyro_bandwidth = EARD_MPU9250_GBAND_184HZ;

    config.mag_rate = EARD_MPU9250_MRATE_8HZ;
    config.mag_precision = EARD_MPU9250_MPRECISION_16BITS;

    return config;
}

eArdMpu9250CalibrationResult ard_mpu9250_calibrate_gyro(ArdMpu9250 *imu, uint32_t duration)
{
    if (duration > ARD_MPU9250_CALIBRATION_MAX_DURATION)
    {
        return ARD_MPU9250_CALIBRATION_DURATION_TOO_LONG;
    }
    else if (duration < ARD_MPU9250_CALIBRATION_MIN_DURATION)
    {
        return ARD_MPU9250_CALIBRATION_DURATION_TOO_SHORT;
    }

    /* copy config for calibration */
    ArdMpu9250Config saved_config = imu->config;
    ArdMpu9250Config config = ard_mpu9250_calibration_config(imu->config.address);

    // set calibration config
    eArdMpu9250ConfigResult result = ard_mpu9250_configure(imu, &config);
    if (result != ARD_MPU9250_CONFIG_OK) return ARD_MPU9250_CALIBRATION_CONFIG_ERROR;

    // Fifo
    ArdMpu9250Fifo imu_fifo;
    imu_fifo.gyro = true;
    imu_fifo.accel = false;
    imu_fifo.temp = false;
    imu_fifo.mag = false;
    ard_mpu9250_fifo_enable(imu->config.address, &imu_fifo);

    // Calibrate
    //----------

    // read

    int32_t gyro_bias_accumulator[3];
    size_t samples = (duration / ARD_MPU9250_DIVIDER_TO_STEP_MICROS(config.rate_divider));

    // TODO(ChisholmKyle): wait for data, capture FIFO, and calibrate gyro

    // restore configuration
    result = ard_mpu9250_configure(imu, &saved_config);
    if (result != ARD_MPU9250_CONFIG_OK) return ARD_MPU9250_CALIBRATION_CONFIG_ERROR;

    // all good
    return ARD_MPU9250_CALIBRATION_SUCCESS;
}

eArdMpu9250CalibrationResult ard_mpu9250_calibrate_accel(ArdMpu9250 *imu, uint32_t duration)
{
    // TODO(ChisholmKyle): Calibrate accel
    return ARD_MPU9250_CALIBRATION_SUCCESS;
}

eArdMpu9250CalibrationResult ard_mpu9250_calibrate_mag(ArdMpu9250 *imu, uint32_t duration)
{
    // TODO(ChisholmKyle): Calibrate mag
    return ARD_MPU9250_CALIBRATION_SUCCESS;
}
