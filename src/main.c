#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <signal.h>
#include <pigpio.h>

#include "mpu6050.h"
#include "common.h"
#include "PID-library/pid.h"
#include "server.h"

// to comply with unity
#ifndef TEST
#define MAIN main
#else
#define MAIN testable_main
#endif

#define MOTOR1_CTL_PIN 18
#define MOTOR1_DIR_PIN 24

#define MOTOR2_CTL_PIN 12
#define MOTOR2_DIR_PIN 23

static volatile sig_atomic_t stop;

void sigint_handler(int signum);

#warning change printf to syslog

int MAIN(void)
{
    PID_TypeDef TPID;
    char acc_gyro_buf[ACCEL_GYRO_BUF_RD_BYTES];
    int16_t x_acc_val, y_acc_val, z_acc_val, x_gyro_val, y_gyro_val, z_gyro_val, temp;
    int i2c_handle, ret;
    double x_acc_ang, y_gyro_ang, angle, pid_out, angle_set_pt;
    x_acc_ang = y_gyro_ang = angle = pid_out = 0;
    angle_set_pt = ANGLE_SET_PT;

    int sockfd, new_fd; // listen on sockfd, new conn on new_fd
    int num_bytes, recv_int;
    char ip_str[INET6_ADDRSTRLEN];
    struct sockaddr_storage their_addr; // connector's address information
    socklen_t sin_size;
    
    // initialize gpios & pid
    ret = gpioInitialise();
    if (ret == PI_INIT_FAILED) {
	printf("gpioInitialise failed\n");
	return -1;
    }
    signal(SIGINT, sigint_handler); // register after gpioInitialise to override pigpio sig handler
  
    gpioSetMode(MOTOR1_DIR_PIN, PI_OUTPUT); // Stepper Motor 1 Direction
    gpioSetMode(MOTOR2_DIR_PIN, PI_OUTPUT); // Stepper Motor 2 Direction

    i2c_handle = mpu6050_init();
    if (i2c_handle < 0 ) {
	printf("mpu6050_init fail: %d\n", i2c_handle);
	return -1;
    }

    PID(&TPID, &angle, &pid_out, &angle_set_pt, 120, 8, 5, _PID_P_ON_E, _PID_CD_REVERSE);
    PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&TPID, LOOP_TIME_MS);
    PID_SetOutputLimits(&TPID, -10000, 10000); 

    // initialize socket server
    sockfd = sockfd_setup();

    printf("server: waiting for client to connect...\n");
    sin_size = sizeof(their_addr);
    new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
    if (new_fd == -1) {
	perror("accept");
    }
    if (!inet_ntop(their_addr.ss_family,
		   get_in_addr((struct sockaddr *)&their_addr), ip_str, sizeof(ip_str))) {
	perror("inet_ntop");
    }
    printf("server: got connection from %s\n", ip_str);
    
    //main code
    while(!stop) {
	ret = i2cReadI2CBlockData(i2c_handle, MPU6050_REG_ACCEL_XOUT_H, acc_gyro_buf, ACCEL_GYRO_BUF_RD_BYTES);
	if (ret < 0) {
	    printf("i2cReadI2CBlockData fail: %d\n", ret);
	    goto exit;
	}

	// may not need all of these variables
	//x_acc_val = (acc_gyro_buf[ACCEL_X_OUT_L] | (acc_gyro_buf[ACCEL_X_OUT_H] << 8)) + MPU6050_ACCEL_X_OFFSET;
	y_acc_val = (acc_gyro_buf[ACCEL_Y_OUT_L] | (acc_gyro_buf[ACCEL_Y_OUT_H] << 8)) + MPU6050_ACCEL_Y_OFFSET;
	z_acc_val = (acc_gyro_buf[ACCEL_Z_OUT_L] | (acc_gyro_buf[ACCEL_Z_OUT_H] << 8)) + MPU6050_ACCEL_Z_OFFSET;
	//temp = (acc_gyro_buf[TEMP_L] | (acc_gyro_buf[TEMP_H] << 8));
	x_gyro_val = (acc_gyro_buf[GYRO_X_OUT_L] | (acc_gyro_buf[GYRO_X_OUT_H] << 8)) + MPU6050_GYRO_X_OFFSET;
	/* y_gyro_val = (acc_gyro_buf[GYRO_Y_OUT_L] | (acc_gyro_buf[GYRO_Y_OUT_H] << 8)) + MPU6050_GYRO_Y_OFFSET; */
	/* z_gyro_val = (acc_gyro_buf[GYRO_Z_OUT_L] | (acc_gyro_buf[GYRO_Z_OUT_H] << 8)) + MPU6050_GYRO_Z_OFFSET; */

	y_gyro_ang += (x_gyro_val/65.5)*LOOP_TIME_S;
	//printf("outputs: %d %d\n", y_acc_val, z_acc_val);
	x_acc_ang = atan2(y_acc_val/8192.0, z_acc_val/8192.0)*RAD_TO_DEG;
	angle = (0.98 * y_gyro_ang) + (0.02 * x_acc_ang);
	if(abs(angle) >= 45) {
	  stop = 1;
	}
	PID_Compute(&TPID);
    
	printf("outputs: %f %f %f %d %d\n", y_gyro_ang, x_acc_ang, angle, x_gyro_val, abs((int)(pid_out)));
	unsigned char stepper_dir = (pid_out >= 0) ? 1:0;
	gpioWrite(MOTOR1_DIR_PIN, stepper_dir);
	gpioWrite(MOTOR2_DIR_PIN, !stepper_dir);
    
	// use angle to control the motor
	gpioHardwarePWM(MOTOR1_CTL_PIN, abs((int)(pid_out)), 1000);
	gpioHardwarePWM(MOTOR2_CTL_PIN, abs((int)(pid_out)), 1000);
    
	usleep(LOOP_TIME_US);
    }
 exit:
    gpioHardwarePWM(MOTOR1_CTL_PIN, 0, 500000); // stop motors
    gpioHardwarePWM(MOTOR2_CTL_PIN, 0, 500000);
    i2cClose(i2c_handle);
    gpioTerminate();
    return 0;
}

void sigint_handler(int signum)
{
    stop = 1;
}
