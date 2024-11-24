#include "MPU_Data.h"

/*模拟IIC端口输出输入定义*/
#define SCL_H         GPIOB->BSRR = GPIO_Pin_10
#define SCL_L         GPIOB->BRR  = GPIO_Pin_10 
#define SDA_H         GPIOB->BSRR = GPIO_Pin_11
#define SDA_L         GPIOB->BRR  = GPIO_Pin_11
#define SCL_read      GPIOB->IDR  & GPIO_Pin_10
#define SDA_read      GPIOB->IDR  & GPIO_Pin_11

char  test=0;
unsigned char SlaveAddress;


/*I2C的端口初始化---------------------------------------*/
void I2C_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*I2C的延时函数-----------------------------------------*/
void I2C_delay(void)
{
    uint8_t i=30; //这里可以优化速度    ，经测试最低到5还能写入
    while(i) 
    { 
        i--; 
    }  
}

/*I2C的等待5ms函数--------------------------------------*/
void delay5ms(void)
{
    int i=5000;  
    while(i) 
    { 
        i--; 
    }  
}

/*I2C启动函数-------------------------------------------*/
bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if(!SDA_read)return 0;    //SDA线为低电平则总线忙,退出
    SDA_L;
    I2C_delay();
    if(SDA_read) return 0;    //SDA线为高电平则总线出错,退出
    SDA_L;
    I2C_delay();
    return 1;
}

/*I2C停止函数-------------------------------------------*/
void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
} 

/*I2C的ACK函数------------------------------------------*/
void I2C_Ack(void)
{    
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}   

/*I2C的NoACK函数----------------------------------------*/
void I2C_NoAck(void)
{    
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
} 

/*I2C等待ACK函数----------------------------------------*/
bool I2C_WaitAck(void)      //返回为:=1有ACK,=0无ACK
{
    SCL_L;
    I2C_delay();
    SDA_H;            
    I2C_delay();
    SCL_H;
    I2C_delay();
    if(SDA_read)
    {
        SCL_L;
        I2C_delay();
        return 0;
    }
    SCL_L;
    I2C_delay();
    return 1;
}

/*I2C发送一个u8数据函数---------------------------------*/
void I2C_SendByte(u8 SendByte) //数据从高位到低位//
{
    uint8_t i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
        if(SendByte&0x80)
            SDA_H;  
        else 
            SDA_L;   
        SendByte<<=1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}  

/*I2C读取一个u8数据函数---------------------------------*/
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;
    
    SDA_H;                
    while(i--)
    {
        ReceiveByte<<=1;      
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();    
        if(SDA_read)
        {
            ReceiveByte|=0x01;
        }
    }
    SCL_L;
    return ReceiveByte;
}  

/*I2C向指定设备指定地址写入u8数据-----------------------*/
void Single_WriteI2C(unsigned char REG_Address,unsigned char REG_data)//单字节写入
{
    if(!I2C_Start())return;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return;}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();    
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    delay5ms();
}

/*I2C向指定设备指定地址读出u8数据-----------------------*/
unsigned char Single_ReadI2C(unsigned char REG_Address)//读取单字节
{   
    unsigned char REG_data;         
    if(!I2C_Start())return 0;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
    I2C_SendByte((u8) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();
    
    REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
    return REG_data;
}

