#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <pigpio.h>

#include "mpu6050.h"

#define DEBUG		0

#define LOOP_TIME_MS	50
#define LOOP_TIME_US	(LOOP_TIME_MS*1000)
#define LOOP_TIME_S	(LOOP_TIME_MS/1000.0)

#define RAD_TO_DEG	57.298578

static volatile sig_atomic_t stop;

void sigint_handler(int signum);

#warning change printf to syslog

int main(void)
{
  char acc_gyro_buf[ACCEL_GYRO_BUF_RD_BYTES];
  int16_t x_acc_val, y_acc_val, z_acc_val, x_gyro_val, y_gyro_val, z_gyro_val, temp;
  int i2c_handle, ret;
  float x_acc_ang, y_gyro_ang, filtered_ang;

  x_acc_ang, y_gyro_ang, filtered_ang = 0;
  
  // initializations
  ret = gpioInitialise();
  if (ret == PI_INIT_FAILED) {
    printf("gpioInitialise failed\n");
    return 1;
  }
  signal(SIGINT, sigint_handler); // register after gpioInitialise to override pigpio sig handler
  
  i2c_handle = mpu_init();
  if (i2c_handle < 0 ) {
    printf("mpu_init fail: %d\n", ret);
  }

  // main code
  while(!stop) {
    ret = i2cReadI2CBlockData(i2c_handle, MPU6050_REG_ACCEL_XOUT_H, acc_gyro_buf, ACCEL_GYRO_BUF_RD_BYTES);
    if (ret < 0) {
      printf("i2cReadI2CBlockData fail: %d\n", ret);
    }

    x_acc_val = (acc_gyro_buf[ACCEL_X_OUT_L] | (acc_gyro_buf[ACCEL_X_OUT_H] << 8)) + MPU6050_ACCEL_X_OFFSET;
    y_acc_val = (acc_gyro_buf[ACCEL_Y_OUT_L] | (acc_gyro_buf[ACCEL_Y_OUT_H] << 8)) + MPU6050_ACCEL_Y_OFFSET;
    z_acc_val = (acc_gyro_buf[ACCEL_Z_OUT_L] | (acc_gyro_buf[ACCEL_Z_OUT_H] << 8)) + MPU6050_ACCEL_Z_OFFSET;
    temp = (acc_gyro_buf[TEMP_L] | (acc_gyro_buf[TEMP_H] << 8));
    x_gyro_val = (acc_gyro_buf[GYRO_X_OUT_L] | (acc_gyro_buf[GYRO_X_OUT_H] << 8)) + MPU6050_GYRO_X_OFFSET;
    y_gyro_val = (acc_gyro_buf[GYRO_Y_OUT_L] | (acc_gyro_buf[GYRO_Y_OUT_H] << 8)) + MPU6050_GYRO_Y_OFFSET;
    z_gyro_val = (acc_gyro_buf[GYRO_Z_OUT_L] | (acc_gyro_buf[GYRO_Z_OUT_H] << 8)) + MPU6050_GYRO_Z_OFFSET;

    /* // do calculations and print out */
    /* printf("x-axis accel: 0x%x, %d\n", x_acc_val, x_acc_val); */
    /* printf("x-axis accel output: %f\n", x_acc_val/8192.0); */
    /* printf("atan2: %f\n", atan2(x_acc_val/8192.0, z_acc_val/8192.0)*57.3); // x angle, 2-axis sensing */
    /* printf("atan2: %f\n", atan2(x_acc_val/8192.0, sqrt((y_acc_val/8192.0)*(y_acc_val/8192.0)+(z_acc_val/8192.0)*(z_acc_val/8192.0)))*57.3); // x angle, 3-axis sensing */
  
    /* printf("y-axis accel: 0x%x, %d\n", y_acc_val, y_acc_val); */
    /* printf("y-axis accel output: %f\n", y_acc_val/8192.0); */
  
    /* printf("z-axis accel: 0x%x, %d\n", z_acc_val, z_acc_val); */
    /* printf("z-axis accel output: %f\n", z_acc_val/8192.0); */

    //printf("accel outputs: %f %f %f\n", x_acc_val/8192.0, y_acc_val/8192.0, z_acc_val/8192.0);

    //printf("temp %f\n", temp/340.0 + 36.53);
    
    //printf("x-axis gyro: 0x%x, %d\n", x_gyro_val, x_gyro_val);
    //printf("x-axis gyro output: %f\n", (x_gyro_val/131.0)*0.1);

    /* printf("y-axis gyro: 0x%x, %d\n", y_gyro_val, y_gyro_val); */
    /* printf("y-axis gyro output: %f\n", (y_gyro_val/131.0)*0.1); */

    /* printf("z-axis gyro: 0x%x, %d\n", z_gyro_val, z_gyro_val); */
    /* printf("z-axis gyro output: %f\n", z_gyro_val/131.0);     */

    //printf("gyro outputs: %f %f %f\n", x_gyro_val/131.0, y_gyro_val/131.0, z_gyro_val/131.0);    

    y_gyro_ang += (x_gyro_val/65.5)*LOOP_TIME_S;
    x_acc_ang = atan2(y_acc_val/8192.0, z_acc_val/8192.0)*RAD_TO_DEG;
    filtered_ang = (0.95 * y_gyro_ang) + (0.05 * x_acc_ang);
    
    printf("outputs: %f %f %f %d\n", y_gyro_ang, x_acc_ang, filtered_ang, x_gyro_val);

    // use angle to control the motor
    
    usleep(LOOP_TIME_US);
  }
  
  /* // output */
  /* gpioSetMode(23, PI_OUTPUT); */
  /* int i=0; */
  /* while(i++ < 200) { */
  /*   gpioWrite(23, 1); */
  /*   usleep(10000); */
  /*   gpioWrite(23, 0); */
  /*   usleep(10000); */
  /* } */

  // software pwm
  /* gpioSetPWMfrequency(23, 1000); */
  /* gpioSetPWMrange(23, 200); // 2500, 500us; 5000; 250us */
  /* gpioPWM(23, 255); */

  /* gpioHardwarePWM(18, 1000, 500000); */
  
  /* usleep(10000000); */
  i2cClose(i2c_handle);
  gpioTerminate();
  return 0;
}

void sigint_handler(int signum)
{
  
  stop = 1;
}
