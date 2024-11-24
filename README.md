# Kalman_Algorithm_KEIL 
Kalman filter algorithm based on STM32 project for fusing HMC5883 and MPU6050 data.

If you need to import the Kalman filter algorithm into a stm32 project developed based on the Keil5 standard library, you need to import the files from the /Hardware folder and the /Software folder and the /Kalman folder into the project.
Then it should be noted that the Serial.c and Serial.h project files are also imported. Or you can open the USART serial port yourself and create the functions you need in KalmanFilter.

# MPU6050:
This is a sensor that integrates an accelerometer and a gyroscope. Accelerometers are used to measure linear acceleration, while gyroscopes are used to measure angular velocity. In attitude estimation, MPU6050 typically provide accelerometer data to estimate pitch and roll angles, while gyroscopes are used to estimate angular changes.

# HMC5883: 
This is a magnetometer sensor that measures the strength and direction of the geomagnetic field to estimate the heading angle (Yaw). Since the output of a magnetometer is highly affected by environmental disturbances, such as ferromagnetic substances, it needs to be fused with other sensor data to improve accuracy.

Both MPU6050 and HMC5883 data are noisy, and the Kalman filter weights the inputs of different sensors, combining the advantages and disadvantages of both, to obtain a more accurate output.
When fusing MPU6050 and HMC5883, the system model typically includes the output of an accelerometer, gyroscope, and magnetometer. The goal of Kalman filtering is to estimate an object's attitude (pitch, roll, and heading angle) from the data provided by these sensors.

![image](https://github.com/user-attachments/assets/cf40ed9a-9a7b-40db-ac77-4308bc20979a)

Measurement Vector: Includes measurements from accelerometers, gyroscopes, and magnetometers.

The accelerometer provides estimation of pitch and roll angles;
The gyroscope provides an estimate of angular velocity;
The magnetometer provides an estimate of the heading angle.

# Prediction steps
Prediction Status:
![image](https://github.com/user-attachments/assets/bd268f13-6b2d-4f68-a7b2-b2a159f36e03)

Predicting Covariance:
![image](https://github.com/user-attachments/assets/f9f26495-9948-4595-8e90-bcb7402636e7)

# Update Steps
Calculate the Kalman gain:
![image](https://github.com/user-attachments/assets/b8363c6c-1cdc-4d07-8d87-e94a0c89dfb4)

Update Status Estimates:
![image](https://github.com/user-attachments/assets/5e9fea46-50c2-44a0-beca-450ee0fb5203)

Update covariance:
![image](https://github.com/user-attachments/assets/774f19a8-0c17-433a-b6bd-c9997b085213)

![image](https://github.com/user-attachments/assets/75f0c143-e03b-4d25-9550-6404587c4004)

Through the prediction and update steps of the Kalman filter, the output data of the MPU6050 and HMC5883 are weighted and fused. Accelerometer and gyroscope data are used to estimate pitch and roll angles, while magnetometer data is used to correct heading angles.


