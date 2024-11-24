#ifndef __SERIAL_H
#define __SERTAL_H


void Serial_Init(void);
uint8_t RX_DataGet(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendInt(int number);
void Serial_SendFloatAsString(float value);
void USART1_IRQHandler(void);



#endif // 
