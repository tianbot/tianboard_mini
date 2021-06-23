#include "param.h"
#include "stm32f4xx_hal_flash.h"
#include "string.h"

/******************************
//              ^^
//              ^^
//              ||
//
//           \    /|
//            \  / |
//          A---   |B
//            /  \
//           /    \
*******************************/

const Param_t DefaultParam = {
  PARAM_HEAD,
  0.0675,//wheel_r
  4.6107,//motor_reduction_ratio
  PI,//max_w
  4.5,//max_speed
  {0.0135, 0.0198, 27, 40},//{base_a, base_b, pwm_dead_zone, max_steer_angle}
  {16, 1.5, 0.3, 500.0, 500.0},//p i d max_output i_limit
  4800,//tick per lap
  65536,//max ticks
  50,//ctrl period
  10,//feedback period
  10,//pose calc period
  PARAM_TAIL
};

Param_t param;

void InitParam(void)
{
  memcpy(&param, (void *)&DefaultParam, sizeof(Param_t));
}