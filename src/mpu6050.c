#include <stdio.h>
#include <pigpio.h>

#include "mpu6050.h"

#warning change printf to syslog 

/**
 * mpu_init() - initialize mpu6050 sensor and config
 *
 * Return: return < 0 if unsuccessful else >= 0
 */
int mpu_init(void)
{
  int i2c_handle, ret;
  
  i2c_handle = i2cOpen(1, MPU6050_I2C_ADDR, 0);
  if (i2c_handle < 0) {
    //printf("i2cOpen: %d\n", i2c_handle);
    ret = i2c_handle;
    goto mpu_init_fail;
  }
  
  ret = i2cWriteByteData(i2c_handle, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_8MHZ_OSC);
  if (ret) {
    //printf("i2cWriteByteData: %d\n", ret);
    goto mpu_init_fail;
  }
  
  ret = i2cWriteByteData(i2c_handle, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCELCFG_4G);
  if (ret) {
    //printf("i2cWriteByteData: %d\n", ret);
    goto mpu_init_fail;
  }
  
  ret = i2cWriteByteData(i2c_handle, MPU6050_REG_GYRO_CONFIG, MPU6050_GYROCFG_500);
  if (ret) {
    //printf("i2cWriteByteData: %d\n", ret);
    goto mpu_init_fail;
  }
  goto mpu_init_pass;
 mpu_init_fail:
  return ret;
 mpu_init_pass:
  return i2c_handle;
}

#warning make inline function
int get_accel_gyro_data(int i2c_handle, char *acc_gyro_buf)
{
  int ret;
  ret = i2cReadI2CBlockData(i2c_handle, MPU6050_REG_ACCEL_XOUT_H, acc_gyro_buf, ACCEL_GYRO_BUF_RD_BYTES);
  if (ACCEL_GYRO_BUF_RD_BYTES != ret) {
    ret = -1; // read an unexpected number of bytes
  }
  return ret;
}
