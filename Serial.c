#include "stm32f10x.h"
#include "Serial.h"
#include <stdio.h>

uint8_t RX_Date;

void Serial_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//复用推挽输出;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1,&USART_InitStructure);

    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1,ENABLE);
}


uint8_t RX_DataGet(void)
{
    return RX_Date;
}

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1,Byte);
    while (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
}

void Serial_SendInt(int number)
{
    char buffer[12];  // 用于存储整数转换后的字符串，假设int最大为10位+1个符号位+1个结尾符号位
    sprintf(buffer, "%d", number);  // 将整数转为字符串

    // 逐个字节发送
    for (int i = 0; buffer[i] != '\0'; i++)
    {
        Serial_SendByte(buffer[i]);  // 使用你原本的 Serial_SendByte 函数发送每个字符
    }
}

void Serial_SendFloatAsString(float value)
{
    char buffer[16];  
    snprintf(buffer, sizeof(buffer), "%.3f", value); 

    for (int i = 0; buffer[i] != '\0'; i++)
    {
        Serial_SendByte(buffer[i]);
    }
}

void USART1_IRQHandler(void)
{
    if ((USART_GetITStatus(USART1,USART_IT_RXNE) == SET))
    {
        RX_Date = USART_ReceiveData(USART1);
    }
}
