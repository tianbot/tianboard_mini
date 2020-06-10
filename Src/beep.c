#include "beep.h"
#include "tim.h"

const uint16_t FreTab[12] = {262, 277, 294, 311, 330, 349, 369, 392, 415, 440, 466, 494};
const uint8_t SignTab[7] = {0, 2, 4, 5, 7, 9, 11};

void BeepOn(uint8_t sound)
{
  if(sound > 6)
  {
    sound = 6;
  }
  TIM12->ARR = 1000000/FreTab[SignTab[sound]] - 1;
  //for mini board
  TIM12->CCR2 = (TIM12->ARR+1)/2;
}

void BeepOff(void)
{
  TIM12->CCR2 = 0;
}

void BeepInit(void)
{
  BeepOff();
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}
