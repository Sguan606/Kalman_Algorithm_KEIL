#ifndef __MPU6050_I2C_H
#define __MPU6050_I2C_H

#include "stm32f10x.h"                  // Device header
#include <stdbool.h>


void I2C_GPIO_Config(void);
void I2C_delay(void);
void delay5ms(void);
bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
bool I2C_WaitAck(void);
void I2C_SendByte(uint8_t SendByte);
unsigned char I2C_RadeByte(void);
void Single_WriteI2C(unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_ReadI2C(unsigned char REG_Address);




#endif // !
