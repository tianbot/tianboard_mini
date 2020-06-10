#include "mpu_port_driver.h"

void get_tick_count(unsigned long *time)
{
  *time = HAL_GetTick();
}

int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        unsigned char *data_ptr)
{
  return HAL_I2C_Mem_Write(&hi2c3, slave_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, 10);
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len, 
                                       unsigned char *data_ptr)
{
  return HAL_I2C_Mem_Read(&hi2c3, slave_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, 10);
}
