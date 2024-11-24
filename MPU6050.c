#include "MPU_Data.h"

#define    SlaveAddressMPU   0x68      //定义器件5883在IIC总线中的从地址
//****************************************
// 定义MPU6050内部地址
//****************************************
#define    SMPLRT_DIV        0x19    //陀螺仪采样率，典型值：0x07(125Hz)
#define    CONFIG            0x1A    //低通滤波频率，典型值：0x06(5Hz)
#define    GYRO_CONFIG       0x1B    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define    ACCEL_CONFIG      0x1C    //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define    ACCEL_XOUT_H      0x3B
#define    ACCEL_XOUT_L      0x3C
#define    ACCEL_YOUT_H      0x3D
#define    ACCEL_YOUT_L      0x3E
#define    ACCEL_ZOUT_H      0x3F
#define    ACCEL_ZOUT_L      0x40
#define    TEMP_OUT_H        0x41
#define    TEMP_OUT_L        0x42
#define    GYRO_XOUT_H       0x43
#define    GYRO_XOUT_L       0x44    
#define    GYRO_YOUT_H       0x45
#define    GYRO_YOUT_L       0x46
#define    GYRO_ZOUT_H       0x47
#define    GYRO_ZOUT_L       0x48
#define    PWR_MGMT_1        0x6B    //电源管理，典型值：0x00(正常启用)
#define    WHO_AM_I          0x75    //IIC地址寄存器(默认数值0x68，只读)
#define    MPU6050_Addr      0xD0    //IIC写入时的地址字节数据，+1为读取


extern int accX, accY, accZ;
extern int gyroX, gyroY, gyroZ;
extern unsigned char SlaveAddress;

//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050(void)
{
    SlaveAddress=MPU6050_Addr;
    Single_WriteI2C(PWR_MGMT_1, 0x00);    //解除休眠状态
    Single_WriteI2C(SMPLRT_DIV, 0x07);// 将采样率设置为 1000Hz - 8kHz/（7+1） = 1000Hz
    Single_WriteI2C(CONFIG, 0x00);// 禁用 FSYNC 并设置 260 Hz Acc 滤波、256 Hz 陀螺仪滤波、8 KHz 采样
    Single_WriteI2C(GYRO_CONFIG, 0x00);// 将 Gyro 满量程范围设置为 ±250deg/s
    Single_WriteI2C(ACCEL_CONFIG, 0x00);// 将 Accelerometer Full Scale Range 设置为 ±2g
    Single_WriteI2C(PWR_MGMT_1, 0x01);// 具有 X 轴陀螺仪参考和禁用睡眠模式的 PLL
}
//**************************************
//// 获取加速度计和陀螺仪值
//**************************************
void updateMPU6050(void)
{
    SlaveAddress=MPU6050_Addr;// 获取加速度计和陀螺仪值

    accX=((Single_ReadI2C(ACCEL_XOUT_H)<<8)+Single_ReadI2C(ACCEL_XOUT_L));
    accY=-((Single_ReadI2C(ACCEL_YOUT_H)<<8)+Single_ReadI2C(ACCEL_YOUT_L));
    accZ=((Single_ReadI2C(ACCEL_ZOUT_H)<<8)+Single_ReadI2C(ACCEL_ZOUT_L));
    
    gyroX=-((Single_ReadI2C(GYRO_XOUT_H)<<8)+Single_ReadI2C(GYRO_XOUT_L));
    gyroY=((Single_ReadI2C(GYRO_YOUT_H)<<8)+Single_ReadI2C(GYRO_YOUT_L));
    gyroZ=-((Single_ReadI2C(GYRO_ZOUT_H)<<8)+Single_ReadI2C(GYRO_ZOUT_L));    
}

