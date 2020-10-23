
// #include <Wire.h>

// #include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
// static MPU9250 s_imu(Wire,0x68);
// int status;

// void ard_mpu9250_setup() {
//   // serial to display data
//   Serial.begin(115200);
//   while(!Serial) {}

//   // start communication with s_imu 
//   status = s_imu.begin();

//   // setting the accelerometer full scale range to +/-8G 
//   s_imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
//   // setting the gyroscope full scale range to +/-500 deg/s
//   s_imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
//   // setting DLPF bandwidth to 20 Hz
//   s_imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
//   // setting SRD to 19 for a 50 Hz update rate
//   s_imu.setSrd(19);
// }

#include "EEPROM.h"

#include "ard_mpu9250.h"

void ard_mpu9250_load_calibration(ArdMpu9250 *imu, const size_t ee_index)
{
    EEPROM.get(ee_index, imu->calibration);
    // return ee_index
}

void ard_mpu9250_save_calibration(ArdMpu9250 *imu, const size_t ee_index)
{
    EEPROM.put(ee_index, imu->calibration);
}
