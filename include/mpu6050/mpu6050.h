

//-----------------------MODIFY THESE PARAMETERS-----------------------

#define GYRO_RANGE 0 // Select which gyroscope range to use (see the table below) - Default is 0
//	Gyroscope Range
//	0	+/- 250 degrees/second
//	1	+/- 500 degrees/second
//	2	+/- 1000 degrees/second
//	3	+/- 2000 degrees/second
// See the MPU6000 Register Map for more information

#define ACCEL_RANGE 0 // Select which accelerometer range to use (see the table below) - Default is 0
//	Accelerometer Range
//	0	+/- 2g
//	1	+/- 4g
//	2	+/- 8g
//	3	+/- 16g
// See the MPU6000 Register Map for more information

// Offsets - supply your own here (calculate offsets with getOffsets function)
//      Accelerometer
#define A_OFF_X 0
#define A_OFF_Y 0
#define A_OFF_Z 0
//    Gyroscope
#define G_OFF_X 0
#define G_OFF_Y 0
#define G_OFF_Z 0



/*
Soft Iron scaling 
[[ 1.28713600e+00 -1.29466322e-01  2.91056541e-04]
 [-1.29466322e-01  1.22178940e+00 -9.67048394e-04]
 [ 2.91056541e-04 -9.67048394e-04  1.30284754e+00]]

Hard Iron Offsets
 [[ 423.35377806]
 [-812.98981625]
 [ 102.82541819]]

computed from file data.txt using calibrate.py on 20.08.2023 at 8.30pm
*/

//-----------------------END MODIFY THESE PARAMETERS-----------------------

extern "C"
{
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
}
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

static const uint8_t REG_DEVID_MPU6050 = 0x68; // MPU6050
static const uint8_t REG_DEVID_QMC5883 = 0x0D; // HMC5883
static const uint8_t REG_POWER_CTL = 0x2D;
static const uint8_t REG_DATAX0 = 0x32;

// Select the appropriate settings
#if GYRO_RANGE == 1
#define GYRO_SENS 65.5
#define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
#define GYRO_SENS 32.8
#define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
#define GYRO_SENS 16.4
#define GYRO_CONFIG 0b00011000
#else // Otherwise, default to 0
#define GYRO_SENS 131.0
#define GYRO_CONFIG 0b00000000
#endif
#undef GYRO_RANGE

#if ACCEL_RANGE == 1
#define ACCEL_SENS 8192.0
#define ACCEL_CONFIG 0b00001000
#elif ACCEL_RANGE == 2
#define ACCEL_SENS 4096.0
#define ACCEL_CONFIG 0b00010000
#elif ACCEL_RANGE == 3
#define ACCEL_SENS 2048.0
#define ACCEL_CONFIG 0b00011000
#else // Otherwise, default to 0
#define ACCEL_SENS 16384.0
#define ACCEL_CONFIG 0b00000000
#endif
#undef ACCEL_RANGE

class IMU
{
private:
    // Pins
    const uint sda_pin = 16;
    const uint scl_pin = 17;

    // Ports
    i2c_inst_t *i2c = i2c0;

    float ax_off, ay_off, az_off, gr_off, gp_off, gy_off;
    void getOffsets();
    void getAccelRaw(float *x, float *y, float *z);
    void getGyroRaw(float *roll, float *pitch, float *yaw);
    void getMagRaw(float *x, float *y, float *z);
    int reg_write(i2c_inst_t *i2c,
                  const uint addr,
                  const uint8_t reg,
                  uint8_t *buf,
                  const uint8_t nbytes);

    int reg_read(i2c_inst_t *i2c,
                 const uint addr,
                 const uint8_t reg,
                 uint8_t *buf,
                 const uint8_t nbytes);

    Eigen::Matrix3d soft_iron_scaling;
    Eigen::Vector3d hard_iron_offset;

public:
    IMU();
    void getAccel(float *x, float *y, float *z);
    void getGyro(float *roll, float *pitch, float *yaw);
    void getMag(float *x, float *y, float *z);
    void initialize_mpu6050();
    void initialize_hmc5883();
};
