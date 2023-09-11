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
    PID(&TPID, &angle, &pid_out, &angle_set_point, 5, 0, 0, _PID_P_ON_E, _PID_CD_DIRECT);
    
    PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&TPID, 1); // set sample time to 1ms for testing
    PID_SetOutputLimits(&TPID, -100, 100); // need to update

    usleep(1000);
    angle = 1;
    PID_Compute(&TPID);
    TEST_ASSERT_EQUAL(pid_out, -5); 
}

void test_pid_kd(void)
{
    TEST_IGNORE_MESSAGE("Need to Implement kd");
}

void test_pid_ki(void)
{
    TEST_IGNORE_MESSAGE("Need to Implement ki");
}

#endif // TEST
