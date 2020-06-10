#ifndef _IMU_TASK_H_
#define _IMU_TASK_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern osThreadId ImuTaskHandle;
void ImuTaskInit(void);

#endif
