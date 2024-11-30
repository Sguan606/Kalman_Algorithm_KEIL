# Kalman_For_Posture-fusion 
Kalman filter algorithm based on STM32 project for fusing HMC5883 and MPU6050 data.

基于 STM32 项目的卡尔曼滤波算法，用于融合 HMC5883 和 MPU6050 数据。

If you need to import the Kalman filter algorithm into a stm32 project developed based on the Keil5 standard library, you need to import the files from the /Hardware folder and the /Software folder and the /Kalman folder into the project.
Then it should be noted that the Serial.c and Serial.h project files are also imported. Or you can open the USART serial port yourself and create the functions you need in KalmanFilter.

如果需要将卡尔曼滤波算法导入到基于 Keil5 标准库开发的 stm32 工程中，则需要将 /Hardware 文件夹、/Software 文件夹和 /Kalman 文件夹中的文件导入到工程中。
然后应该注意的是，Serial.c 和 Serial.h 项目文件也被导入了。或者您可以自己打开 USART 串口并在 KalmanFilter 中创建您需要的功能。

# MPU6050:
This is a sensor that integrates an accelerometer and a gyroscope. Accelerometers are used to measure linear acceleration, while gyroscopes are used to measure angular velocity. In attitude estimation, MPU6050 typically provide accelerometer data to estimate pitch and roll angles, while gyroscopes are used to estimate angular changes.

这是一个集成了加速度计和陀螺仪的传感器。加速度计用于测量线性加速度，而陀螺仪用于测量角速度。在姿态估计中，MPU6050通常提供加速度计数据来估计俯仰角和滚动角，而陀螺仪则用于估计角度变化。

# HMC5883: 
This is a magnetometer sensor that measures the strength and direction of the geomagnetic field to estimate the heading angle (Yaw). Since the output of a magnetometer is highly affected by environmental disturbances, such as ferromagnetic substances, it needs to be fused with other sensor data to improve accuracy.

这是一种磁力计传感器，可测量地磁场的强度和方向，以估计航向角 （Yaw）。由于磁力计的输出受环境干扰（如铁磁物质）的高度影响，因此需要将其与其他传感器数据融合以提高精度。

Both MPU6050 and HMC5883 data are noisy, and the Kalman filter weights the inputs of different sensors, combining the advantages and disadvantages of both, to obtain a more accurate output.
When fusing MPU6050 and HMC5883, the system model typically includes the output of an accelerometer, gyroscope, and magnetometer. The goal of Kalman filtering is to estimate an object's attitude (pitch, roll, and heading angle) from the data provided by these sensors.

MPU6050 和 HMC5883 数据都是有噪声的，卡尔曼滤波器对不同传感器的输入进行加权，结合两者的优缺点，以获得更准确的输出。
当MPU6050和HMC5883融合时，系统模型通常包括加速度计、陀螺仪和磁力计的输出。卡尔曼滤波的目标是根据这些传感器提供的数据估计物体的姿态（俯仰、滚动和航向角）。

![image](https://github.com/user-attachments/assets/cf40ed9a-9a7b-40db-ac77-4308bc20979a)

Measurement Vector: Includes measurements from accelerometers, gyroscopes, and magnetometers.

Measurement Vector（测量矢量）：包括来自加速度计、陀螺仪和磁力计的测量值。

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


Through the prediction and update steps of the Kalman filter, the output data of the MPU6050 and HMC5883 are weighted and fused. Accelerometer and gyroscope data are used to estimate pitch and roll angles, while magnetometer data is used to correct heading angles.Indeed it is recursion-based optimal estimation algorithm improves the accuracy of system estimation by fusing data from different sensors to reduce noise and uncertainty.

通过卡尔曼滤波的预测和更新步骤，对MPU6050和HMC5883的输出数据进行加权和融合。加速度计和陀螺仪数据用于估计俯仰角和滚动角，而磁力计数据用于校正航向角。事实上，它是基于递归的最优估计算法，通过融合来自不同传感器的数据来减少噪声和不确定性，从而提高了系统估计的准确性。

