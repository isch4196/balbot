#ifndef COMMON_H
#define COMMON_H

#define	LOOP_TIME_MS		10
#define LOOP_TIME_US		(LOOP_TIME_MS*1000)
#define LOOP_TIME_S		(LOOP_TIME_MS/1000.0)

#define ADJ_ANG_TIME_MS		50
#define ADJ_ANG_TIME_LOOP	(ADJ_ANG_TIME_MS/LOOP_TIME_MS)

#define INPUT_RESET_TIME_MS	100
#define INPUT_RESET_TIME_LOOP	(INPUT_RESET_TIME_MS/LOOP_TIME_MS)

#define DEF_P_VAL		300
#define DEF_I_VAL		50
#define DEF_D_VAL		0.015

#define MOTOR1_CTL_PIN		18
#define MOTOR1_DIR_PIN		24
#define MOTOR2_CTL_PIN		13
#define MOTOR2_DIR_PIN		23

#define USE_DEFAULT_VALUES	1
#define USE_TUNED_VALUES	4

#define STOP_ANGLE		60

#define UP_KEY			259
#define DOWN_KEY		258
#define LEFT_KEY		260
#define RIGHT_KEY		261

#define NO_TURN			0
#define TURN_LEFT		1
#define TURN_RIGHT		2

#define MOVE_ANGLE		5
#define DEF_TURN_SPEED		2000

#endif
