#ifndef _RACECAR_TASK_H_
#define _RACECAR_TASK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#ifndef PI
#define PI 3.1415926 
#endif

#define LIMIT(x, a, b) (((x)<(a)) ? (a) : (((x)>(b)) ? (b) : (x)))

#define RACECAR_SPEED_ZERO 1500
#define RACECAR_STEER_ANGLE_ZERO 90

#define MID_STEER_ANGLE 90

#define SERVO_CAL(X) ((2500 - 500)*(180-X)/180+500)
#define ANGLE_CAL(X) (PI-((X)-1500)/2000*PI)
#define MOTOR_CAL(X) (X + RACECAR_SPEED_ZERO)

#define MOTOR_MAX 1800
#define MOTOR_MIN 1200

#define RACECAR_CTRL_TIMEOUT  1000

void RacecarTaskInit(void);
#endif