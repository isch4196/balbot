#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <pigpio.h>

#include "mpu6050.h"
#include "common.h"
#include "PID-library/pid.h"

static volatile sig_atomic_t stop;

void sigint_handler(int signum);

#warning change printf to syslog

int main(void)
{
  PID_TypeDef TPID;
  char acc_gyro_buf[ACCEL_GYRO_BUF_RD_BYTES];
  int16_t x_acc_val, y_acc_val, z_acc_val, x_gyro_val, y_gyro_val, z_gyro_val, temp;
  int i2c_handle, ret;
  double x_acc_ang, y_gyro_ang, angle, pid_out, angle_set_pt;
  x_acc_ang = y_gyro_ang = angle = pid_out = 0;
  angle_set_pt = ANGLE_SET_PT;
  
  // initializations
  ret = gpioInitialise();
  if (ret == PI_INIT_FAILED) {
    printf("gpioInitialise failed\n");
    return 1;
  }
  signal(SIGINT, sigint_handler); // register after gpioInitialise to override pigpio sig handler
  
  gpioSetMode(24, PI_OUTPUT); // Stepper Motor 1 Direction
  i2c_handle = mpu6050_init();
  if (i2c_handle < 0 ) {
    printf("mpu6050_init fail: %d\n", ret);
  }

  PID(&TPID, &angle, &pid_out, &angle_set_pt, 50, 0, 0, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPID, LOOP_TIME_MS);
  PID_SetOutputLimits(&TPID, -2000, 2000); 
  
  //main code
  while(!stop) {
    ret = i2cReadI2CBlockData(i2c_handle, MPU6050_REG_ACCEL_XOUT_H, acc_gyro_buf, ACCEL_GYRO_BUF_RD_BYTES);
    if (ret < 0) {
      printf("i2cReadI2CBlockData fail: %d\n", ret);
    }

    // may not need all of these variables
    x_acc_val = (acc_gyro_buf[ACCEL_X_OUT_L] | (acc_gyro_buf[ACCEL_X_OUT_H] << 8)) + MPU6050_ACCEL_X_OFFSET;
    y_acc_val = (acc_gyro_buf[ACCEL_Y_OUT_L] | (acc_gyro_buf[ACCEL_Y_OUT_H] << 8)) + MPU6050_ACCEL_Y_OFFSET;
    z_acc_val = (acc_gyro_buf[ACCEL_Z_OUT_L] | (acc_gyro_buf[ACCEL_Z_OUT_H] << 8)) + MPU6050_ACCEL_Z_OFFSET;
    temp = (acc_gyro_buf[TEMP_L] | (acc_gyro_buf[TEMP_H] << 8));
    x_gyro_val = (acc_gyro_buf[GYRO_X_OUT_L] | (acc_gyro_buf[GYRO_X_OUT_H] << 8)) + MPU6050_GYRO_X_OFFSET;
    y_gyro_val = (acc_gyro_buf[GYRO_Y_OUT_L] | (acc_gyro_buf[GYRO_Y_OUT_H] << 8)) + MPU6050_GYRO_Y_OFFSET;
    z_gyro_val = (acc_gyro_buf[GYRO_Z_OUT_L] | (acc_gyro_buf[GYRO_Z_OUT_H] << 8)) + MPU6050_GYRO_Z_OFFSET;

    y_gyro_ang += (x_gyro_val/65.5)*LOOP_TIME_S;
    x_acc_ang = atan2(y_acc_val/8192.0, z_acc_val/8192.0)*RAD_TO_DEG;
    angle = (0.95 * y_gyro_ang) + (0.05 * x_acc_ang);
    PID_Compute(&TPID);
    
    printf("outputs: %f %f %f %d\n", y_gyro_ang, x_acc_ang, angle, abs((int)(pid_out)));
    unsigned char stepper_dir = (pid_out >= 0) ? 1:0;
    gpioWrite(24, stepper_dir);
    
    // use angle to control the motor
    gpioHardwarePWM(18, abs((int)(pid_out)), 500000);
    
    usleep(LOOP_TIME_US);
  }
  
  i2cClose(i2c_handle);
  gpioTerminate();
  return 0;
}

void sigint_handler(int signum)
{
  stop = 1;
}
