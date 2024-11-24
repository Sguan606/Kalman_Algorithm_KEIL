# Kalman_Algorithm_KEIL
Kalman filter algorithm based on STM32 project for fusing HMC5883 and MPU6050 data.

If you need to import the Kalman filter algorithm into a stm32 project developed based on the Keil5 standard library, you need to import the files from the /Hardware folder and the /Software folder and the /Kalman folder into the project.
Then it should be noted that the Serial.c and Serial.h project files are also imported. Or you can open the USART serial port yourself and create the functions you need in KalmanFilter.
