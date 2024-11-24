# Kalman_Algorithm_KEIL
Kalman filter algorithm based on STM32 project for fusing HMC5883 and MPU6050 data.

If you need to import the Kalman filter algorithm into a stm32 project developed based on the Keil5 standard library, you need to import the files from the /Hardware folder and the /Software folder and the /Kalman folder into the project.

Then it should be noted that the Serial.c and Serial.h project files are also imported. Or you can open the USART serial port yourself and create the functions you need in KalmanFilter.

Kalman Filter Overview
Introduction
The Kalman filter is an optimal estimation algorithm used in various fields such as signal processing, control systems, robotics, and economics. It combines noisy sensor measurements with a mathematical model of a system's dynamics to provide an optimal estimate of the system's state.

The Kalman filter is recursive, meaning it updates its estimates as new data becomes available. It's widely used in applications where sensor data is uncertain and must be filtered to obtain accurate and reliable state estimates.

Applications
Kalman filters are used in a wide range of applications, including:

Navigation Systems: GPS, IMU (Inertial Measurement Units) for robotics, airplanes, and spacecraft.
Computer Vision: Object tracking and estimation of motion in video frames.
Economics: Estimating economic variables or forecasting.
Robotics: Localization and mapping in SLAM (Simultaneous Localization and Mapping).
Kalman Filter Theory
The Kalman filter operates in two main steps:

Prediction Step: Based on the previous state and the system model, predict the current state of the system.
Update Step: Use the current measurement to correct and update the prediction.
The Kalman filter uses the following key equations:

State Prediction:
x
^
k
∣
k
−
1
=
A
x
^
k
−
1
∣
k
−
1
+
B
u
k
x
^
  
k∣k−1
​
 =A 
x
^
  
k−1∣k−1
​
 +Bu 
k
​
 

Where:

x
^
k
∣
k
−
1
x
^
  
k∣k−1
​
  is the predicted state at time 
k
k,
A
A is the state transition matrix,
x
^
k
−
1
∣
k
−
1
x
^
  
k−1∣k−1
​
  is the state estimate at the previous time step,
B
B is the control matrix, and
u
k
u 
k
​
  is the control input at time 
k
k.
Covariance Prediction:
P
k
∣
k
−
1
=
A
P
k
−
1
∣
k
−
1
A
T
+
Q
P 
k∣k−1
​
 =AP 
k−1∣k−1
​
 A 
T
 +Q

Where:

P
k
∣
k
−
1
P 
k∣k−1
​
  is the predicted state covariance,
Q
Q is the process noise covariance.
Measurement Update:
The measurement update step corrects the state prediction based on the measurement 
z
k
z 
k
​
 .

Kalman Gain:
K
k
=
P
k
∣
k
−
1
H
T
(
H
P
k
∣
k
−
1
H
T
+
R
)
−
1
K 
k
​
 =P 
k∣k−1
​
 H 
T
 (HP 
k∣k−1
​
 H 
T
 +R) 
−1
 

Where:

H
H is the measurement matrix,

R
R is the measurement noise covariance.

State Update:

x
^
k
∣
k
=
x
^
k
∣
k
−
1
+
K
k
(
z
k
−
H
x
^
k
∣
k
−
1
)
x
^
  
k∣k
​
 = 
x
^
  
k∣k−1
​
 +K 
k
​
 (z 
k
​
 −H 
x
^
  
k∣k−1
​
 )

Where:

z
k
z 
k
​
  is the actual measurement at time 
k
k.

Covariance Update:

P
k
∣
k
=
(
I
−
K
k
H
)
P
k
∣
k
−
1
P 
k∣k
​
 =(I−K 
k
​
 H)P 
k∣k−1
​
 

Where:

P
k
∣
k
P 
k∣k
​
  is the updated state covariance.
How the Kalman Filter Works
At each step, the Kalman filter performs a prediction of the next state and then corrects the prediction using the new measurement. The filter combines these two sources of information in a way that minimizes the variance of the estimation error, making it an optimal filter when the system dynamics and measurement noise are Gaussian.

1. Prediction Step
In the prediction step, the filter uses the system's previous state estimate and the state transition model to predict the current state. This prediction is accompanied by an estimate of the uncertainty (covariance) in the prediction.

2. Update Step
In the update step, the filter corrects the predicted state using the current measurement. It computes the Kalman Gain, which determines how much weight should be given to the new measurement versus the previous prediction. The more certain the prediction, the less weight is given to the measurement, and vice versa.

Example Code
Here’s an example of how to implement a simple 1D Kalman filter in Python:

python
import numpy as np

# Initial state estimate
x = np.array([0, 0])  # [position, velocity]

# Initial covariance estimate
P = np.array([[1, 0], [0, 1]])

# State transition matrix
A = np.array([[1, 1], [0, 1]])

# Control input matrix
B = np.array([[0.5], [1]])

# Measurement matrix
H = np.array([1, 0])

# Measurement noise covariance
R = 0.1

# Process noise covariance
Q = np.array([[0.01, 0], [0, 0.01]])

# Control input (constant acceleration)
u = np.array([0.5])

# Simulate measurements
measurements = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8])

for z in measurements:
    # Prediction step
    x = A @ x + B @ u
    P = A @ P @ A.T + Q
    
    # Measurement update step
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (np.eye(2) - K @ H) @ P
    
    # Print the updated state estimate
    print(f"Updated state estimate: {x}")
Explanation:
Initial State and Covariance: The initial state estimate is [0, 0] (assumed to be at rest at position 0), and the initial covariance is assumed to be [1, 0; 0, 1].
Prediction Step: The position and velocity are predicted using the system's state transition matrix A and the control input u.
Update Step: The filter uses the actual measurements to correct the prediction and reduce the uncertainty.
How to Use This Project
Clone this repository to your local machine.

bash
git clone https://github.com/yourusername/kalman-filter.git
Install required dependencies (if any).

bash
pip install -r requirements.txt
Run the example code or modify it to fit your use case.

bash
python kalman_filter_example.py
Conclusion
The Kalman filter is a powerful tool for estimating the state of a system based on noisy measurements. It has widespread applications in various fields and is fundamental in robotics, control systems, and sensor fusion. This repository provides a simple example of how to implement a Kalman filter, and you can adapt it to suit more complex systems.

Further Reading
Kalman Filter Wikipedia
Understanding Kalman Filters by Greg Welch and Gary Bishop
