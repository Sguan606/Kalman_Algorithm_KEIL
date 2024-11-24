#include "MPU_Data.h"


void Init(struct Kalman* klm)
{
    /* 我们将像这样设置变量，这些变量也可以由用户调整 */
    klm->Q_angle = 0.001;
    klm->Q_bias = 0.003;
    klm->R_measure = 0.03;

    klm->angle = 0; // 重置角度
    klm->bias = 0; // 重置偏差

    klm->P[0][0] = 0; // 我们假设偏差为 0 并且我们知道起始角度（使用 setAngle）
    klm->P[0][1] = 0;
    klm->P[1][0] = 0;
    klm->P[1][1] = 0;
}

// 角度应以度为单位，速率应以度/秒为单位，增量时间应以秒为单位
double getAngle(struct Kalman * klm, double newAngle, double newRate, double dt) 
{
    // KasBot V2 - 卡尔曼滤波器模块 - http://www->x-firm->com/？page_id=145
    // 修改者 Kristian Lauszus
    // 有关更多信息，请参阅我的博客文章：http://blog->tkjelectronics->dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
    
    // 离散卡尔曼滤波时间更新方程 - 时间更新 （“Predict”）
    // 更新 xhat - 预测未来的状态
    /* Step 1 */
    klm->rate = newRate - klm->bias;
    klm->angle += dt * klm->rate;
    
    //更新估计误差协方差 - 提前预测误差协方差
    /* Step 2 */
    klm->P[0][0] += dt * (dt*klm->P[1][1] - klm->P[0][1] - klm->P[1][0] + klm->Q_angle);
    klm->P[0][1] -= dt * klm->P[1][1];
    klm->P[1][0] -= dt * klm->P[1][1];
    klm->P[1][1] += klm->Q_bias * dt;
    
    //离散卡尔曼滤波器测量更新方程 - Measurement Update 
		//计算卡尔曼增益 - 计算卡尔曼增益
    /* Step 4 */
    klm->S = klm->P[0][0] + klm->R_measure;
    /* Step 5 */
    klm->K[0] = klm->P[0][0] / klm->S;
    klm->K[1] = klm->P[1][0] / klm->S;
    
    // 计算角度和偏差 - 使用测量 zk 更新估计值 （newAngle）
    /* Step 3 */
    klm->y = newAngle - klm->angle;
    /* Step 6 */
    klm->angle += klm->K[0] * klm->y;
    klm->bias += klm->K[1] * klm->y;
    
    // 计算估计误差协方差 - 更新误差协方差
    /* Step 7 */
    klm->P[0][0] -= klm->K[0] * klm->P[0][0];
    klm->P[0][1] -= klm->K[0] * klm->P[0][1];
    klm->P[1][0] -= klm->K[1] * klm->P[0][0];
    klm->P[1][1] -= klm->K[1] * klm->P[0][1];
    
    return klm->angle;
}



void setAngle(struct Kalman* klm, double newAngle) 
{ 
    klm->angle = newAngle; 
} // 用于设置角度，应将其设置为起始角度


double getRate(struct Kalman* klm) 
{ 
    return klm->rate; 
} // 返回无偏率


/* 这些用于调整卡尔曼滤波器*/
void setQangle(struct Kalman* klm, double newQ_angle) 
{ 
    klm->Q_angle = newQ_angle; 
}


void setQbias(struct Kalman* klm, double newQ_bias) 
{ 
    klm->Q_bias = newQ_bias; 
}

void setRmeasure(struct Kalman* klm, double newR_measure) 
{ 
    klm->R_measure = newR_measure; 
}

double getQangle(struct Kalman* klm) 
{ 
    return klm->Q_angle; 
}

double getQbias(struct Kalman* klm) 
{ 
    return klm->Q_bias; 
}

double getRmeasure(struct Kalman* klm) 
{ 
    return klm->R_measure; 
}

