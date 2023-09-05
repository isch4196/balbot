#ifndef MPU6050_H
#define MPU6050_H

#define MPU6050_I2C_ADDR		0x68

// MPU6050 REGS
#define MPU6050_REG_SMPRT_DIV		0x19
#define MPU6050_REG_GYRO_CONFIG		0x1B
#define MPU6050_REG_ACCEL_CONFIG	0x1C
#define MPU6050_REG_ACCEL_XOUT_H	0x3B
#define MPU6050_REG_ACCEL_XOUT_L	0x3C
#define MPU6050_REG_ACCEL_YOUT_H	0x3D
#define MPU6050_REG_ACCEL_YOUT_L	0x3E
#define MPU6050_REG_ACCEL_ZOUT_H	0x3F
#define MPU6050_REG_ACCEL_ZOUT_L	0x40
#define MPU6050_REG_TEMP_H		0x41
#define MPU6050_REG_TEMP_L		0x42
#define MPU6050_REG_GYRO_XOUT_H		0x43
#define MPU6050_REG_GYRO_XOUT_L		0x44
#define MPU6050_REG_GYRO_YOUT_H		0x45
#define MPU6050_REG_GYRO_YOUT_L		0x46
#define MPU6050_REG_GYRO_ZOUT_H		0x47
#define MPU6050_REG_GYRO_ZOUT_L		0x48
#define MPU6050_REG_PWR_MGMT_1		0x6B

// MPU6050_REG_PWR_MGMT_1 BITS
#define MPU6050_PWR1_8MHZ_OSC	0x01

// MPU6050_REG_ACCEL_CONFIG BITS
#define MPU6050_ACCELCFG_2G	(0x00 << 3)
#define MPU6050_ACCELCFG_4G	(0x01 << 3)
#define MPU6050_ACCELCFG_8G	(0x02 << 3)
#define MPU6050_ACCELCFG_16G	(0x03 << 3)

// MPU6050_REG_GYRO_CONFIG BITS
#define MPU6050_GYROCFG_250	(0x00 << 3)
#define MPU6050_GYROCFG_500	(0x01 << 3)
#define MPU6050_GYROCFG_1000	(0x02 << 3)
#define MPU6050_GYROCFG_2000	(0x03 << 3)

#warning do proper calibration later using code of Jeff Rowberg
// MPU6050 CALIBRATION OFFSETS
#define MPU6050_ACCEL_X_OFFSET  -153
#define MPU6050_ACCEL_Y_OFFSET	182
#define MPU6050_ACCEL_Z_OFFSET	-948
#define MPU6050_GYRO_X_OFFSET	51
#define MPU6050_GYRO_Y_OFFSET	-31
#define MPU6050_GYRO_Z_OFFSET	151

// MPU6050 ACCEL DIVISORS
#define MPU6050_ACCEL_2G_LSB	16384
#define MPU6050_ACCEL_4G_LSB	8192
#define MPU6050_ACCEL_8G_LSB	4096
#define MPU6050_ACCEL_16G_LSB	2048

#define ACCEL_GYRO_BUF_RD_BYTES 14
#define ACCEL_X_OUT_H	0
#define ACCEL_X_OUT_L	1
#define ACCEL_Y_OUT_H	2
#define ACCEL_Y_OUT_L	3
#define ACCEL_Z_OUT_H	4
#define ACCEL_Z_OUT_L	5
#define TEMP_H		6
#define TEMP_L		7
#define GYRO_X_OUT_H	8
#define GYRO_X_OUT_L	9
#define GYRO_Y_OUT_H	10
#define GYRO_Y_OUT_L	11
#define GYRO_Z_OUT_H	12
#define GYRO_Z_OUT_L	13

int mpu6050_init(void);

#endif

