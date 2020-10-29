#include "racecar_task.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "dbus_task.h"
#include "param.h"
#include "tim.h"
#include "protocol_task.h"
//#include "imu.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "pid.h"
#include "beep_task.h"

osThreadId RacecarCtrlTaskHandle;
osThreadId RacecarFeedbackTaskHandle;
osThreadId RacecarPoseTaskHandle;

typedef struct Position
{
  float v_x_m;
  float v_y_m;
  float position_x_m;
  float position_y_m;
  float yaw;
  float wz;
} Pos_t;

static Pos_t pose;
float motor_v;

static void RacecarCtrlTaskEntry(void const *argument)
{
  osEvent evt;
  MotionCtrl_t *p;
  Pid_t MotorPid;
  int timeout = 0;
  int motorPwm;
  float steering = 90.0;
  float v;
  int errorFlag = 0;
  int errorCount = 0;
  int moveDir = MOVE_DIR_FORWARD;
  //mini board
  TIM5->CCR1 = LIMIT(RACECAR_SPEED_ZERO, MOTOR_MIN, MOTOR_MAX);
  TIM5->CCR2 = LIMIT(SERVO_CAL(RACECAR_STEER_ANGLE_ZERO), SERVO_CAL(MID_STEER_ANGLE - param.racecar.max_steer_angle), SERVO_CAL(MID_STEER_ANGLE + param.racecar.max_steer_angle));
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  PidInit(&MotorPid, POSITION_PID, param.pid.max_out, param.pid.i_limit, param.pid.p, param.pid.i, param.pid.d);

  osDelay(5000);
  for (;;)
  {
    if (errorFlag == 0)
    {
      evt = osMailGet(CtrlMail, param.ctrl_period);
      int pwm = TIM5->CCR1 - 1500;
      if (((pwm > STALL_OR_ENCODER_ERROR_PWM) && (fabs(motor_v) < MINIMAL_V)) || ((pwm < -STALL_OR_ENCODER_ERROR_PWM) && (fabs(motor_v) < MINIMAL_V)))
      {
        errorCount++;
        if (errorCount > 30)
        {
          if (errorFlag == 0)
          {
            Beep(1, 500);
            Beep(6, 500);
            Beep(1, 500);
            Beep(6, 500);
            Beep(1, 500);
            Beep(6, 500);
          }
          errorFlag = 1;
          continue;
        }
      }
      else
      {
        errorCount = 0;
      }

      if (evt.status == osEventMail)
      {
        timeout = 0;
        p = evt.value.p;
        steering = p->steering_angle;
        v = p->vx;

        if (v > 0 && v < 0.3)
        {
          v = 0.3;
        }
        else if (v < 0 && v > -0.25)
        {
          v = -0.25;
        }
        motorPwm = PidCalc(&MotorPid, motor_v, v);
        steering += 90;

        //skip dead zone
        if (motorPwm < 0)
        {
          motorPwm -= param.racecar.pwm_dead_zone;
        }
        else if (motorPwm > 0)
        {
          motorPwm += param.racecar.pwm_dead_zone;
        }

        if (v == 0)
        {
          motorPwm = 0;
        }

        if ((moveDir == MOVE_DIR_FORWARD) && (v < 0)) //generate brake pwm
        {
          TIM5->CCR1 = 1200;
          while (motor_v != 0)
          {
            osDelay(10);
          }
          osDelay(100);
          TIM5->CCR1 = 1500;
          osDelay(100);
          moveDir = MOVE_DIR_BACKWARD;
          do
          {
            evt = osMailGet(CtrlMail, 0);
            if (evt.status == osEventMail)
            {
              osMailFree(CtrlMail, evt.value.p);
            }
          } while (evt.status == osEventMail); //drop all ctrl cmd during brake
        }
        else if (v > 0)
        {
          moveDir = MOVE_DIR_FORWARD;
        }

        TIM5->CCR1 = LIMIT(MOTOR_CAL(motorPwm), MOTOR_MIN, MOTOR_MAX);
        TIM5->CCR2 = LIMIT(SERVO_CAL(steering), SERVO_CAL(MID_STEER_ANGLE - param.racecar.max_steer_angle), SERVO_CAL(MID_STEER_ANGLE + param.racecar.max_steer_angle));
        osMailFree(CtrlMail, p);
      }
      else
      {
        timeout++;
        if (timeout >= 200 / param.ctrl_period)
        {
          v = 0;
          steering = RACECAR_STEER_ANGLE_ZERO;
          timeout = 0;
        }
        motorPwm = PidCalc(&MotorPid, motor_v, v);
        //skip dead zone
        if (motorPwm < 0)
        {
          motorPwm -= param.racecar.pwm_dead_zone;
        }
        else if (motorPwm > 0)
        {
          motorPwm += param.racecar.pwm_dead_zone;
        }

        if (v == 0)
        {
          motorPwm = 0;
        }

        TIM5->CCR1 = LIMIT(MOTOR_CAL(motorPwm), MOTOR_MIN, MOTOR_MAX);
        TIM5->CCR2 = LIMIT(SERVO_CAL(steering), SERVO_CAL(MID_STEER_ANGLE - param.racecar.max_steer_angle), SERVO_CAL(MID_STEER_ANGLE + param.racecar.max_steer_angle));
      }
    }
    else
    {
      motorPwm = 0;
      steering = RACECAR_STEER_ANGLE_ZERO;
      TIM5->CCR1 = LIMIT(MOTOR_CAL(motorPwm), MOTOR_MIN, MOTOR_MAX);
      TIM5->CCR2 = LIMIT(SERVO_CAL(steering), SERVO_CAL(MID_STEER_ANGLE - param.racecar.max_steer_angle), SERVO_CAL(MID_STEER_ANGLE + param.racecar.max_steer_angle));
    }
  }
}

static void RacecarFeedbackTaskEntry(void const *argument)
{
  struct odom odom;
  while (1)
  {
    odom.pose.point.x = pose.position_x_m;
    odom.pose.point.y = pose.position_y_m;
    odom.pose.point.z = 0;
    odom.pose.yaw = pose.yaw;
    odom.twist.angular.x = 0;
    odom.twist.angular.y = 0;
    odom.twist.angular.z = pose.wz;
    odom.twist.linear.x = pose.v_x_m;
    odom.twist.linear.y = pose.v_y_m;
    odom.twist.linear.z = 0;

    ProtocolSend(PACK_TYPE_ODOM_RESPONSE, (uint8_t *)&odom, sizeof(struct odom));
    osDelay(param.feedback_period);
  }
}

//int DeltaTicksExt;

static void RacecarPoseTaskEntry(void const *argument)
{
  int DeltaTicks;
  static uint16_t ticks = 0;
  uint16_t preTicks = 0;
  float delta_m;

  float angle;
  memset(&pose, 0, sizeof(pose));
  for (;;)
  {
    ticks = __HAL_TIM_GET_COUNTER(&htim4);

    if (ticks - preTicks > param.max_ticks / 2) //down overflow
    {
      DeltaTicks = (int16_t)((preTicks + param.max_ticks - ticks) % param.max_ticks);
    }
    else if (ticks - preTicks < -param.max_ticks / 2) //up overflow
    {
      DeltaTicks = (int16_t)(-(ticks + param.max_ticks - preTicks) % param.max_ticks);
    }
    else if (ticks > preTicks)
    {
      DeltaTicks = (int16_t)(-(ticks - preTicks));
    }
    else
    {
      DeltaTicks = (int16_t)(preTicks - ticks);
    }
    preTicks = ticks;
    delta_m = (float)DeltaTicks / param.ticks_per_lap * 2 * PI * param.wheel_r / param.motor_reduction_ratio;
    motor_v = delta_m / param.pose_calc_period * 1000.0f; // m/s

    pose.wz = motor_v * tan(ANGLE_CAL(TIM5->CCR2)) / 2 / param.racecar.base_b;
    pose.yaw += delta_m * tan(ANGLE_CAL(TIM5->CCR2)) / 2 / param.racecar.base_b;
    if (pose.yaw > PI)
    {
      pose.yaw -= 2 * PI;
    }
    else if (pose.yaw < -PI)
    {
      pose.yaw += 2 * PI;
    }
    //angle = imu.yaw / RADIAN_COEF;
    angle = pose.yaw;
    pose.position_x_m += delta_m * cos(angle);
    pose.position_y_m += delta_m * sin(angle);

    //pose.v_x_m = motor_v * cos(angle);
    //pose.v_y_m = motor_v * sin(angle);
    pose.v_x_m = motor_v;
    pose.v_y_m = 0;
    osDelay(param.pose_calc_period);
  }
}
osThreadDef(RacecarCtrlTask, RacecarCtrlTaskEntry, osPriorityAboveNormal, 0, 512);
osThreadDef(RacecarFeedbackTask, RacecarFeedbackTaskEntry, osPriorityAboveNormal, 0, 512);
osThreadDef(RacecarPoseTask, RacecarPoseTaskEntry, osPriorityAboveNormal, 0, 512);
void RacecarTaskInit(void)
{
  RacecarCtrlTaskHandle = osThreadCreate(osThread(RacecarCtrlTask), NULL);
  RacecarFeedbackTaskHandle = osThreadCreate(osThread(RacecarFeedbackTask), NULL);
  RacecarPoseTaskHandle = osThreadCreate(osThread(RacecarPoseTask), NULL);
}
