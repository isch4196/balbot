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

/**
 * tune_mpu6050() - Tune the mpu6050 gyro and accel values
 *
 * The mpu6050 does not seem to be accurate nor precise. Every time the program
 * starts, it has different offsets, so turn the mpu6050 offsets on every start
 * of the program.
 * 
 * Return: 0 on success, else fail
 * 
 */
uint8_t tune_mpu6050(int i2c_handle, char *acc_gyro_buf, float *y_acc_avg_offset, float *z_acc_avg_offset, float *x_gyro_avg_offset)
{
    int ret;
    int16_t y_acc_val, z_acc_val, x_gyro_val;
    
    struct timespec spec;
    time_t sec_to_wait;

    float y_acc_avg_offset_tmp = 0, z_acc_avg_offset_tmp = 0, x_gyro_avg_offset_tmp = 0;
    uint32_t count = 0;

    clock_gettime(CLOCK_MONOTONIC, &spec);
    sec_to_wait = spec.tv_sec + 5;
    while (spec.tv_sec <= sec_to_wait) {
	if ((ret = i2cReadI2CBlockData(i2c_handle, MPU6050_REG_ACCEL_YOUT_H, acc_gyro_buf, 8)) < 0) {
	    printf("i2cReadI2CBlockData fail: %d\n", ret);
	    return -1;
	}
	y_acc_val = (acc_gyro_buf[1] | (acc_gyro_buf[0] << 8));
	z_acc_val = (acc_gyro_buf[3] | (acc_gyro_buf[2] << 8));
	x_gyro_val = (acc_gyro_buf[7] | (acc_gyro_buf[6] << 8));
	
	count += 1;
	y_acc_avg_offset_tmp -= y_acc_avg_offset_tmp/count;
	y_acc_avg_offset_tmp += (float)y_acc_val/count;
	
	z_acc_avg_offset_tmp -= z_acc_avg_offset_tmp/count;
	z_acc_avg_offset_tmp += (float)z_acc_val/count;
	
	x_gyro_avg_offset_tmp -= x_gyro_avg_offset_tmp/count;
	x_gyro_avg_offset_tmp += (float)x_gyro_val/count;
	
	clock_gettime(CLOCK_MONOTONIC, &spec);
    }
    z_acc_avg_offset_tmp -= 16384; // resting position hovesr around 16384
    *y_acc_avg_offset = y_acc_avg_offset_tmp;
    *z_acc_avg_offset = z_acc_avg_offset_tmp;
    *x_gyro_avg_offset = x_gyro_avg_offset_tmp;
    return 0;
}
