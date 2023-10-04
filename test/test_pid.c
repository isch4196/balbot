#ifdef TEST

#include "unity.h"

#include <unistd.h>
#include "pid.h"

void setUp(void) // not working properly?? else I would use setUp and tearDown
{
}

void tearDown(void)
{
}

void test_pid_kp(void)
{
    PID_TypeDef TPID;
    double angle, pid_out, angle_set_point;

    angle_set_point = 0;
    PID(&TPID, &angle, &pid_out, &angle_set_point, 5, 0, 0, _PID_P_ON_E, _PID_CD_REVERSE);
    
    PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&TPID, 1); // set sample time to 1ms for testing
    PID_SetOutputLimits(&TPID, -100, 100); // need to update

    usleep(1000);
    angle = 1;
    PID_Compute(&TPID);
    TEST_ASSERT_EQUAL(5, pid_out); 
}

void test_pid_kd(void)
{
    PID_TypeDef TPID;
    double angle, pid_out, angle_set_point;

    angle_set_point = 0;
    PID(&TPID, &angle, &pid_out, &angle_set_point, 5, 0, 0.001, _PID_P_ON_E, _PID_CD_REVERSE);
    
    PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&TPID, 1); // set sample time to 1ms for testing
    PID_SetOutputLimits(&TPID, -100, 100); // need to update

    // first run
    usleep(1000);
    angle = 5;
    PID_Compute(&TPID);
    // Kp (5*5 = 25) + Kd (1*5 = 5) = 30
    TEST_ASSERT_EQUAL(30, pid_out);

    // second run
    usleep(1000);
    angle = 3;
    PID_Compute(&TPID);
    // Kp (3*5 = 15) + Kd (1*(3-5) = -2) = 13
    TEST_ASSERT_EQUAL(13, pid_out);
}

void test_pid_ki(void)
{
    TEST_IGNORE_MESSAGE("Need to Implement ki");
}

void test_pid_positive_to_negative(void)
{
    // if we read a positive angle, we want to move motors in forward,
    // hence expect a positive
    PID_TypeDef TPID;
    double angle, pid_out, angle_set_point;

    angle_set_point = 0;
    PID(&TPID, &angle, &pid_out, &angle_set_point, 5, 0, 0, _PID_P_ON_E, _PID_CD_REVERSE);
    
    PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&TPID, 1); // set sample time to 1ms for testing
    PID_SetOutputLimits(&TPID, -100, 100); // need to update

    usleep(1000);
    angle = 1;
    PID_Compute(&TPID);
    TEST_ASSERT_GREATER_THAN(0, pid_out);
}

void test_pid_negative_to_positive(void)
{
    // if we read a negative angle, we want to move motors backwards,
    // hence expect a negative num
    PID_TypeDef TPID;
    double angle, pid_out, angle_set_point;

    angle_set_point = 0;
    PID(&TPID, &angle, &pid_out, &angle_set_point, 5, 0, 0, _PID_P_ON_E, _PID_CD_REVERSE);
    
    PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&TPID, 1); // set sample time to 1ms for testing
    PID_SetOutputLimits(&TPID, -100, 100); // need to update

    usleep(1000);
    angle = -1;
    PID_Compute(&TPID);
    
    TEST_ASSERT_LESS_THAN(0, pid_out);
}

#endif // TEST
