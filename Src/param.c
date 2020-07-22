#include "param.h"
#include "stm32f4xx_hal_flash.h"
#include "string.h"

const Param_t DefaultParam = {
  PARAM_HEAD,
  0.033,//wheel_r
  7.6,//motor_reduction_ratio
  PI,//max_w
  4.5,//max_speed
  {0.087, 0.13, 17, 30},//{base_a, base_b, pwm_dead_zone, max_steer_angle}
  {100, 10, 3, 300.0, 300.0},//p i d max_output i_limit
  4096,//tick per lap
  65536,//max ticks
  10,//ctrl period
  10,//feedback period
  10,//pose calc period
  PARAM_TAIL
};

Param_t param;

void InitParam(void)
{
  memcpy(&param, (void *)&DefaultParam, sizeof(Param_t));
}