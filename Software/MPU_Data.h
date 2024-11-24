#ifndef __MPU_DATA_H
#define __MPU_DATA_H

#include "stm32f10x.h"
#include "Kalman.h" 
#include <math.h>
#include "MPU6050.h"
#include "HMC5883.h"
#include <stdbool.h>
#include "MPU6050_I2C.h"

#define MAG0MAX 603
#define MAG0MIN -578
#define MAG1MAX 542
#define MAG1MIN -701
#define MAG2MAX 547
#define MAG2MIN -556
#define RAD_TO_DEG 57.295779513082320876798154814105  // 弧度转角度的转换率
#define DEG_TO_RAD 0.01745329251994329576923690768489 // 角度转弧度的转换率



void RCC_Configuration(void);
void InitAll(void);
void send(double xx,double yy,double zz);
void func(void);
void updatePitchRoll(void);
void updateYaw(void);



#endif // !
