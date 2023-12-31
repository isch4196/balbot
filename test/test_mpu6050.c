#ifdef TEST

#include "unity.h"
#include "mock_pigpio.h"

#include "mpu6050.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_mpu6050_mpu_init_i2cWriteByteData_pass(void)
{
    int i2c_handle, ret;

    i2c_handle = 0;
    i2cOpen_ExpectAndReturn(1, MPU6050_I2C_ADDR, 0, i2c_handle);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_8MHZ_OSC, 0); 
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCELCFG_2G, 0);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_CONFIG, 0, 0);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_GYRO_CONFIG, MPU6050_GYROCFG_500, 0);  

    ret = mpu6050_init();
    TEST_ASSERT_EQUAL(0, ret);
}

void test_mpu6050_mpu_init_i2cOpen_fail(void)
{
    int i2c_handle, ret;

    i2c_handle = -1;
    i2cOpen_ExpectAndReturn(1, MPU6050_I2C_ADDR, 0, i2c_handle);

    ret = mpu6050_init();
    TEST_ASSERT_EQUAL(-1, ret);
}

void test_mpu6050_mpu_init_i2cWriteByteData_fail_1(void)
{
    int i2c_handle, ret;

    i2c_handle = 0;
    i2cOpen_ExpectAndReturn(1, MPU6050_I2C_ADDR, 0, i2c_handle);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_8MHZ_OSC, -1);

    ret = mpu6050_init();
    TEST_ASSERT_EQUAL(-1, ret);
}

void test_mpu6050_mpu_init_i2cWriteByteData_fail_2(void)
{
    int i2c_handle, ret;

    i2c_handle = 0;
    i2cOpen_ExpectAndReturn(1, MPU6050_I2C_ADDR, 0, i2c_handle);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_8MHZ_OSC, 0); 
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCELCFG_2G, -1);

    ret = mpu6050_init();
    TEST_ASSERT_EQUAL(-1, ret);
}

void test_mpu6050_mpu_init_i2cWriteByteData_fail_3(void)
{
    int i2c_handle, ret;

    i2c_handle = 0;
    i2cOpen_ExpectAndReturn(1, MPU6050_I2C_ADDR, 0, i2c_handle);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_8MHZ_OSC, 0); 
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCELCFG_2G, 0);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_CONFIG, 0, -1);
    
    ret = mpu6050_init();
    TEST_ASSERT_EQUAL(-1, ret);
}

void test_mpu6050_mpu_init_i2cWriteByteData_fail_4(void)
{
    int i2c_handle, ret;

    i2c_handle = 0;
    i2cOpen_ExpectAndReturn(1, MPU6050_I2C_ADDR, 0, i2c_handle);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_8MHZ_OSC, 0); 
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCELCFG_2G, 0);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_CONFIG, 0, 0);
    i2cWriteByteData_ExpectAndReturn(i2c_handle, MPU6050_REG_GYRO_CONFIG, MPU6050_GYROCFG_500, -1);  
    
    ret = mpu6050_init();
    TEST_ASSERT_EQUAL(-1, ret);
}

#endif // TEST
