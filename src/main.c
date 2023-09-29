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
#include <fcntl.h>
#include <pigpio.h>

#include "mpu6050.h"
#include "common.h"
#include "PID-library/pid.h"
#include "server.h"

#define TEST_PID	1

#define MOTOR1_CTL_PIN	18
#define MOTOR1_DIR_PIN	24

#define MOTOR2_CTL_PIN	12
#define MOTOR2_DIR_PIN	23

#define USE_DEFAULT_VALUES	1
#define USE_TUNED_VALUES	4

static volatile sig_atomic_t stop;

void sigint_handler(int signum);

#warning change printf to syslog

int main(int argc, char *argv[])
{
    PID_TypeDef TPID;
    char acc_gyro_buf[8];
    int16_t y_acc_val, z_acc_val, x_gyro_val;
    int i2c_handle, ret;
    double x_acc_ang, y_gyro_ang, angle, pid_out, angle_set_pt;
    double p, i, d;
    x_acc_ang = y_gyro_ang = angle = pid_out = 0;
    angle_set_pt = ANGLE_SET_PT;
    
    uint8_t ctrl = 0;
    
#if !TEST_PID
    int sockfd, new_fd; // listen on sockfd, new conn on new_fd
    int num_bytes, recv_int;
    char ip_str[INET6_ADDRSTRLEN];
    struct sockaddr_storage their_addr; // connector's address information
    socklen_t sin_size;
#endif

    if (USE_TUNED_VALUES == argc) { 
	p = atoi(argv[1]); i = atoi(argv[2]); d = atoi(argv[3]);
	printf("Using tuned values p %f, i=%f d=%f\n", p, i, d);
    } else if (USE_DEFAULT_VALUES == argc) {
	p = 130; i = 0; d = 0;
	printf("Using default values p=%f, i=%f d=%f\n", p, i, d);
    } else {
	printf("Wrong number of arguments supplied.\n");
	return -1;
    }
    
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

    PID(&TPID, &angle, &pid_out, &angle_set_pt, p, i, d, _PID_P_ON_E, _PID_CD_REVERSE); // 115, 18, 5
    PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&TPID, LOOP_TIME_MS);
    PID_SetOutputLimits(&TPID, -15000, 15000); 

    // tune the mpu6050 value offset. through testing it has been found to be highly volatile
    float y_acc_avg_offset = 0, z_acc_avg_offset = 0, x_gyro_avg_offset = 0;
    ret = tune_mpu6050(i2c_handle, acc_gyro_buf, &y_acc_avg_offset, &z_acc_avg_offset, &x_gyro_avg_offset);
    if (ret) {
	return -1;
    }
    printf("y_acc_avg_offset: %f, z_acc_avg_offset: %f, x_gyro_avg_offset: %f\n", y_acc_avg_offset, z_acc_avg_offset, x_gyro_avg_offset);

#if !TEST_PID
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

    if(fcntl(new_fd, F_SETFL, fcntl(new_fd, F_GETFL, 0) | O_NONBLOCK) == -1) {
        perror("fcntl");
    }
#endif
    
    while(!stop) {
	if (i2cReadI2CBlockData(i2c_handle, MPU6050_REG_ACCEL_YOUT_H, acc_gyro_buf, 8) < 0) {
	    printf("i2cReadI2CBlockData fail: %d\n", ret);
	    goto exit;
	}
#if !TEST_PID
	// check for any input
	if ((num_bytes = recv(new_fd, &recv_int, sizeof(recv_int), 0)) > 0) {
	    recv_int = ntohl(recv_int);
	    printf("server: received %d\n", recv_int);
	    switch(recv_int) {
	    case 259: // up
		angle_set_pt += 1;
		break;
	    case 258: // down
		angle_set_pt -= 1;
		break;
	    case 260: // left
		ctrl = 1;
		break;
	    case 261: // right
		break;
		ctrl = 1;
	    default:
		break;
	    }
	    
	} else if (!num_bytes) {
	    printf("Socket peer has shutdown\n");
	    stop = 1;
	}
	
#endif
	if (!ctrl) {
	    y_acc_val = (acc_gyro_buf[1] | (acc_gyro_buf[0] << 8)) + (-1)*(int)y_acc_avg_offset;
	    z_acc_val = (acc_gyro_buf[3] | (acc_gyro_buf[2] << 8)) + (-1)*(int)z_acc_avg_offset;
	    x_gyro_val = (acc_gyro_buf[7] | (acc_gyro_buf[6] << 8)) + (-1)*(int)x_gyro_avg_offset;

	    // do calcs
	    y_gyro_ang += (x_gyro_val/32.8)*LOOP_TIME_S;
	    //printf("outputs: %d %d\n", y_acc_val, z_acc_val);
	    x_acc_ang = atan2(y_acc_val/16384.0, z_acc_val/16384.0)*RAD_TO_DEG;
	    angle = (0.95 * y_gyro_ang) + (0.05 * x_acc_ang);
	    if(abs(angle) >= 60) {
		stop = 1;
	    }
	    PID_Compute(&TPID);
	    
	    unsigned char stepper_dir = (pid_out >= 0) ? 1:0;
	    gpioWrite(MOTOR1_DIR_PIN, stepper_dir);
	    gpioWrite(MOTOR2_DIR_PIN, !stepper_dir);
    
	    // use angle to control the motor
	    gpioHardwarePWM(MOTOR1_CTL_PIN, abs((int)(pid_out)), 1000);
	    gpioHardwarePWM(MOTOR2_CTL_PIN, abs((int)(pid_out)), 1000);
	} else {
	    printf("turn\n");
	    gpioWrite(MOTOR1_DIR_PIN, 1);
	    gpioWrite(MOTOR2_DIR_PIN, 1);
	    gpioHardwarePWM(MOTOR1_CTL_PIN, 300, 1000);
	    gpioHardwarePWM(MOTOR2_CTL_PIN, 300, 1000);
	    ctrl = 0;
	}
    
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
