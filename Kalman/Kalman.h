#ifndef __KALMAN_H
#define __KALMAN_H


#include <stdbool.h>

struct Kalman 
{
    /* Kalman filter variables */
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error
};

void Init(struct Kalman* klm);
double getAngle(struct Kalman * klm, double newAngle, double newRate, double dt);
void setAngle(struct Kalman* klm, double newAngle);
double getRate(struct Kalman* klm);
void setQangle(struct Kalman* klm, double newQ_angle);
void setQbias(struct Kalman* klm, double newQ_bias);
void setRmeasure(struct Kalman* klm, double newR_measure);
double getQangle(struct Kalman* klm);
double getQbias(struct Kalman* klm);
double getRmeasure(struct Kalman* klm);



// Copyright (C) 2012 Kristian Lauszus, TKJ Electronics-> All rights reserved->
//  This software may be distributed and modified under the terms of the GNU
//  General Public License version 2 (GPL2) as published by the Free Software
//  Foundation and appearing in the file GPL2->TXT included in the packaging of
//  this file-> Please note that GPL2 Section 2[b] requires that all works based
//  on this software must also be made publicly available under the terms of
//  the GPL2 ("Copyleft")->
//  Contact information
//  -------------------
//  Kristian Lauszus, TKJ Electronics
//  Web      :  http://www->tkjelectronics->com
//  e-mail   :  kristianl@tkjelectronics->com



#endif

