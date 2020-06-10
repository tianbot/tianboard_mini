#ifndef _MPU_PORT_DRIVER_H_
#define _MPU_PORT_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "i2c.h"

int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        unsigned char *data_ptr);
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len, 
                                       unsigned char *data_ptr);
void get_tick_count(unsigned long *time);
                                       
#endif

