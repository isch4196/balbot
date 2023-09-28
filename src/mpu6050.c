/**
 * mpu6050.c: A small non-comprehensive driver for mpu6050, along with
 * some application code. It only implements what is needed for this
 * specific project.
 */

#include <stdio.h>
#include <pigpio.h>
#include <syslog.h>

#include "mpu6050.h"

/**
 * mpu_init() - initialize mpu6050 sensor and config
 *
 * Return: return < 0 if unsuccessful else >= 0
 */
int mpu6050_init(void)
{
  int i2c_handle, ret;
  
  i2c_handle = i2cOpen(1, MPU6050_I2C_ADDR, 0);
  if (i2c_handle < 0) {
    syslog(LOG_ERR, "i2cOpen fail: %d\n", i2c_handle);
    ret = i2c_handle;
    goto mpu_init_fail;
  }
  
  ret = i2cWriteByteData(i2c_handle, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_8MHZ_OSC);
  if (ret) {
    syslog(LOG_ERR, "i2cWriteByteData fail: %d\n", ret);
    goto mpu_init_fail;
  }
  
  ret = i2cWriteByteData(i2c_handle, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCELCFG_2G);
  if (ret) {
    syslog(LOG_ERR, "i2cWriteByteData fail: %d\n", ret);
    goto mpu_init_fail;
  }
  
  ret = i2cWriteByteData(i2c_handle, MPU6050_REG_GYRO_CONFIG, MPU6050_GYROCFG_1000);
  if (ret) {
    syslog(LOG_ERR, "i2cWriteByteData fail: %d\n", ret);
    goto mpu_init_fail;
  }
  goto mpu_init_pass;
 mpu_init_fail:
  return ret;
 mpu_init_pass:
  return i2c_handle;
}
