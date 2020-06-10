#include "imu_task.h"
#include "imu.h"
#include "cmsis_os.h"
#include "beep_task.h"
#include "protocol_task.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "mpu_port_driver.h"
#define IMU_DATA_READY 0x01

osThreadId ImuTaskHandle;
osThreadId ImuFeedbackTaskHandle;
imu_t imu = {0};

unsigned char *mpl_key = (unsigned char*)"eMPL 6.12";
struct platform_data_s {
    signed char orientation[9];
};
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

// void CalRPY(long *quat, float *values)
// {
//   long t1, t2, t3;
//   long q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;

//   q00 = inv_q29_mult(quat[0], quat[0]);
//   q01 = inv_q29_mult(quat[0], quat[1]);
//   q02 = inv_q29_mult(quat[0], quat[2]);
//   q03 = inv_q29_mult(quat[0], quat[3]);
//   q11 = inv_q29_mult(quat[1], quat[1]);
//   q12 = inv_q29_mult(quat[1], quat[2]);
//   q13 = inv_q29_mult(quat[1], quat[3]);
//   q22 = inv_q29_mult(quat[2], quat[2]);
//   q23 = inv_q29_mult(quat[2], quat[3]);
//   q33 = inv_q29_mult(quat[3], quat[3]);

//   /* X component of the Ybody axis in World frame */
//   t1 = q12 - q03;

//   /* Y component of the Ybody axis in World frame */
//   t2 = q22 + q00 - (1L << 30);
//   values[2] = -atan2f((float) t1, (float) t2) * 180.f / (float) M_PI;

//   /* Z component of the Ybody axis in World frame */
//   t3 = q23 + q01;
//   values[0] =
//       atan2f((float) t3,
//               sqrtf((float) t1 * t1 +
//                     (float) t2 * t2)) * 180.f / (float) M_PI;
//   /* Z component of the Zbody axis in World frame */
//   t2 = q33 + q00 - (1L << 30);
//   if (t2 < 0) {
//       if (values[0] >= 0)
//           values[0] = 180.f - values[0];
//       else
//           values[0] = -180.f - values[0];
//   }

//   /* X component of the Xbody axis in World frame */
//   t1 = q11 + q00 - (1L << 30);
//   /* Y component of the Xbody axis in World frame */
//   t2 = q12 + q03;
//   /* Z component of the Xbody axis in World frame */
//   t3 = q13 - q02;

//   values[1] =
//       (atan2f((float)(q33 + q00 - (1L << 30)), (float)(q13 - q02)) *
//         180.f / (float) M_PI - 90);
//   if (values[1] >= 90)
//       values[1] = 180 - values[1];

//   if (values[1] < -90)
//       values[1] = -180 - values[1];
// }

void ImuTaskEntry(void const *argument)
{
  /* USER CODE BEGIN ImuTaskEntry */
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  unsigned char accel_fsr;
  unsigned short gyro_rate, gyro_fsr;
  struct int_param_s int_param;
  
  //init mpl
  osDelay(1000);
  mpu_init(&int_param);
  inv_init_mpl();
  inv_enable_quaternion();
  inv_enable_fast_nomot();
  inv_enable_gyro_tc();
  inv_start_mpl();
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(IMU_DATA_RATE);
  mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
  inv_set_gyro_sample_rate(1000000L / gyro_rate);
  inv_set_accel_sample_rate(1000000L / gyro_rate);
  inv_set_gyro_orientation_and_scale(
  inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
          (long)gyro_fsr<<15);
  inv_set_accel_orientation_and_scale(
          inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
          (long)accel_fsr<<15);
  
  //init dmp  
  dmp_load_motion_driver_firmware();
  dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
  dmp_set_fifo_rate(IMU_DATA_RATE);
  mpu_set_dmp_state(1);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  short gyro[3], accel[3], sensors;
  unsigned long sensor_timestamp;
  unsigned char more;
  long quat[4];
  //float values[3];
  float gyro_lsb = 32767.0/gyro_fsr;
  float accel_lsb = 32767.0/accel_fsr;
  Beep(6, 100);
  osDelay(400);
  Beep(6, 100);
  /* Infinite loop */
  for(;;)
  {
    osSignalWait(IMU_DATA_READY, osWaitForever);
    
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    imu.ax = accel[0]/accel_lsb;
    imu.ay = accel[1]/accel_lsb;
    imu.az = accel[3]/accel_lsb;
    imu.wx = gyro[0]/gyro_lsb/RADIAN_COEF;
    imu.wy = gyro[1]/gyro_lsb/RADIAN_COEF;
    imu.wz = gyro[2]/gyro_lsb/RADIAN_COEF;
    if(sensors & INV_WXYZ_QUAT) 
    {
      imu.q[0] = inv_q30_to_double(quat[0]);
      imu.q[1] = inv_q30_to_double(quat[1]);
      imu.q[2] = inv_q30_to_double(quat[2]);
      imu.q[3] = inv_q30_to_double(quat[3]);
      //CalRPY(quat, values);
    }
    if(more)
    {
      osSignalSet(ImuTaskHandle, IMU_DATA_READY);
    }
      
  }
  /* USER CODE END ImuTaskEntry */
}

void ImuFeedbackTaskEntry(void const *argument)
{
  /* USER CODE BEGIN ImuTaskEntry */
  struct imu_feedback imu_feedback;
  /* Infinite loop */
  for (;;)
  {
    imu_feedback.quat.w = imu.q[0];
    imu_feedback.quat.x = imu.q[1];
    imu_feedback.quat.y = imu.q[2];
    imu_feedback.quat.z = imu.q[3];
    imu_feedback.linear_acc.x = imu.ax;
    imu_feedback.linear_acc.y = imu.ay;
    imu_feedback.linear_acc.z = imu.az;
    imu_feedback.angular_vel.x = imu.wx;
    imu_feedback.angular_vel.y = imu.wy;
    imu_feedback.angular_vel.z = imu.wz;
    ProtocolSend(PACK_TYPE_IMU_REPONSE, (uint8_t *)&imu_feedback, sizeof(struct imu_feedback));
    osDelay(20);
  }
  /* USER CODE END ImuTaskEntry */
}

osThreadDef(ImuTask, ImuTaskEntry, osPriorityAboveNormal, 0, 1024);
osThreadDef(ImuFeedbackTask, ImuFeedbackTaskEntry, osPriorityNormal, 0, 128);
void ImuTaskInit(void)
{
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);
  ImuFeedbackTaskHandle = osThreadCreate(osThread(ImuFeedbackTask), NULL);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == MPU_INT_Pin)
  {
    if (ImuTaskHandle != NULL)
    {
      osSignalSet(ImuTaskHandle, IMU_DATA_READY);
    }
  }
}
