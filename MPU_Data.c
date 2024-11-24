#include "MPU_Data.h"
#include "Serial.h"


#define RESTRICT_PITCH 

struct Kalman kalmanX, kalmanY, kalmanZ; // 创建 Kalman 实例
int accX, accY, accZ;
int gyroX, gyroY, gyroZ;
int magX, magY, magZ;
double roll, pitch, yaw; 
double gyroXangle, gyroYangle, gyroZangle; // 只用陀螺仪计算角度
double compAngleX, compAngleY, compAngleZ; //用电磁计计算角度
double kalAngleX, kalAngleY, kalAngleZ; //用kalman计算角度
float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];

void RCC_Configuration(void)
{   
    SystemInit();
}

void InitAll(void)
{
    /* 设置卡尔曼和陀螺仪起始角度 */
    updateMPU6050();
    updateHMC5883();
    updatePitchRoll();
    updateYaw();
    
    setAngle(&kalmanX,roll); // 首先设置Roll起始角度
    gyroXangle = roll;
    compAngleX = roll;
    
    setAngle(&kalmanY,pitch); // Then pitch
    gyroYangle = pitch;
    compAngleY = pitch;
    
    setAngle(&kalmanZ,yaw); // And finally yaw
    gyroZangle = yaw;
    compAngleZ = yaw;
    
//    timer = micros; // 初始化计时器
}

void send(double xx,double yy,double zz)
{
    int    a[3];
    uint8_t i,sendData[12];       
    a[0]=(int)xx;a[1]=(int)yy;a[2]=(int)zz;
    for(i=0;i<3;i++)
    {
        if(a[i]<0){
            sendData[i*4]='-';
            a[i]=-a[i];
        }
        else sendData[i*4]=' ';
        sendData[i*4+1]=(u8)(a[i]%1000/100+0x30);
        sendData[i*4+2]=(u8)(a[i]%100/10+0x30);
        sendData[i*4+3]=(u8)(a[i]%10+0x30);
    }
    for(i=0;i<12;i++)
    {
        Serial_SendByte(sendData[i]);
    }
    Serial_SendByte(0x0D);
    Serial_SendByte(0x0A);
}

void func(void)
{
    double gyroXrate,gyroYrate,gyroZrate,dt=0.01;
    /*更新所有 IMU 值 */
    updateMPU6050();
    updateHMC5883();
    
//    dt = (double)(micros - timer) / 1000; // 计算增量时间
//    timer = micros;
//    if(dt<0)dt+=(1<<20);    //时间是周期性的，有可能当前时间小于上次时间，因为这个周期远大于两次积分时间，所以最多相差1<<20

    /* Roll and pitch estimation */
    updatePitchRoll();             //用采集的加速计的值计算roll和pitch的值
    gyroXrate = gyroX / 131.0;     //转换为度/秒(deg/s)    把陀螺仪的角加速度按照当初设定的量程转换为°/s
    gyroYrate = gyroY / 131.0;     //转换为度/秒
    
    #ifdef RESTRICT_PITCH        //如果上面有#define RESTRICT_PITCH就采用这种方法计算，防止出现-180和180之间的跳跃
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        setAngle(&kalmanX,roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
    kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // 使用卡尔曼滤波器计算角度
    
    if (fabs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // 反转速率，使其适合受限的加速度计读数
    kalAngleY = getAngle(&kalmanY,pitch, gyroYrate, dt);
    #else
    // 这修复了当加速度计角度在 -180 度和 180 度之间跳跃时的过渡问题
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    } else
    kalAngleY = getAngle(&kalmanY, pitch, gyroYrate, dt); // 使用卡尔曼滤波器计算角度
    
    if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate，使其适合受限加速度计读数
    kalAngleX = getAngle(&kalmanX, roll, gyroXrate, dt); // 使用卡尔曼滤波器计算角度
    #endif
    
    
    /*偏航估计 */
    updateYaw();
    gyroZrate = gyroZ / 131.0; // 转换为度/秒(deg/s) 
    // 这修复了当偏航角在 -180 度和 180 度之间跳跃时的过渡问题
    if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
        setAngle(&kalmanZ,yaw);
        compAngleZ = yaw;
        kalAngleZ = yaw;
        gyroZangle = yaw;
    } else
    kalAngleZ = getAngle(&kalmanZ, yaw, gyroZrate, dt); // 使用卡尔曼滤波器计算角度
    
    
    /* 仅使用陀螺仪估计角度*/
    gyroXangle += gyroXrate * dt; //在没有任何滤波器的情况下计算陀螺仪角度
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // 使用卡尔曼滤波器的无偏率计算陀螺仪角度
    //gyroYangle += kalmanY.getRate() * dt;
    //gyroZangle += kalmanZ.getRate() * dt;
    
    /* 使用互补滤波器估计角度 */
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // 使用 Complimentary 滤波器计算角度
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
    compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;
    
    //当陀螺仪角度漂移太多时重置陀螺仪角度
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
    if (gyroZangle < -180 || gyroZangle > 180)
        gyroZangle = kalAngleZ;
    
    
    send(roll,pitch,yaw);
//    send(gyroXangle,gyroYangle,gyroZangle);
//    send(compAngleX,compAngleY,compAngleZ);
//    send(kalAngleX,kalAngleY,kalAngleZ);
//    send(kalAngleY,compAngleY,gyroYangle);


    /* Print Data */
//    //#if 1
//    printf("%lf %lf %lf %lf\n",roll,gyroXangle,compAngleX,kalAngleX);
//    printf("%lf %lf %lf %lf\n",pitch,gyroYangle,compAngleY,kalAngleY);
//    printf("%lf %lf %lf %lf\n",yaw,gyroZangle,compAngleZ,kalAngleZ);
    //#endif
    
//    //#if 0 // Set to 1 to print the IMU data
//    printf("%lf %lf %lf\n",accX / 16384.0,accY / 16384.0,accZ / 16384.0);
//    printf("%lf %lf %lf\n",gyroXrate,gyroYrate,gyroZrate);
//    printf("%lf %lf %lf\n",magX,magY,magZ);
    //#endif
    
    //#if 0 // Set to 1 to print the temperature
    //Serial.print("\t");
    //
    //double temperature = (double)tempRaw / 340.0 + 36.53;
    //Serial.print(temperature); Serial.print("\t");
    //#endif
//    delay(10);
} 

//****************************************
//根据加速计刷新Pitch和Roll数据
//这里采用两种方法计算roll和pitch，如果最上面没有#define RESTRICT_PITCH就采用第二种计算方法
//****************************************
void updatePitchRoll(void) 
{
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // ATAN2 将 -π 的值输出为 π（弧度）- 请参阅 - see http://en.wikipedia.org/wiki/Atan2
    // 然后将其从弧度转换为度数
    #ifdef RESTRICT_PITCH // Eq. 25 and 26
    roll = atan2(accY,accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
    roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    #endif
}
//****************************************
//根据磁力计刷新Yaw角
//****************************************
void updateYaw(void) 
{
    double rollAngle,pitchAngle,Bfy,Bfx;  
    
    magX *= -1; // 反转轴 - 在这里完成此操作，因为应该在校准后完成
    magZ *= -1;
    
    magX *= magGain[0];
    magY *= magGain[1];
    magZ *= magGain[2];
    
    magX -= magOffset[0];
    magY -= magOffset[1];
    magZ -= magOffset[2];
    
    
    rollAngle  = kalAngleX * DEG_TO_RAD;
    pitchAngle = kalAngleY * DEG_TO_RAD;
    
    Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
    Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
    yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
    
    yaw *= -1;
}

