#include "MPU_Data.h"


void Init(struct Kalman* klm)
{
    /* ���ǽ����������ñ�������Щ����Ҳ�������û����� */
    klm->Q_angle = 0.001;
    klm->Q_bias = 0.003;
    klm->R_measure = 0.03;

    klm->angle = 0; // ���ýǶ�
    klm->bias = 0; // ����ƫ��

    klm->P[0][0] = 0; // ���Ǽ���ƫ��Ϊ 0 ��������֪����ʼ�Ƕȣ�ʹ�� setAngle��
    klm->P[0][1] = 0;
    klm->P[1][0] = 0;
    klm->P[1][1] = 0;
}

// �Ƕ�Ӧ�Զ�Ϊ��λ������Ӧ�Զ�/��Ϊ��λ������ʱ��Ӧ����Ϊ��λ
double getAngle(struct Kalman * klm, double newAngle, double newRate, double dt) 
{
    // KasBot V2 - �������˲���ģ�� - http://www->x-firm->com/��page_id=145
    // �޸��� Kristian Lauszus
    // �йظ�����Ϣ��������ҵĲ������£�http://blog->tkjelectronics->dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
    
    // ��ɢ�������˲�ʱ����·��� - ʱ����� ����Predict����
    // ���� xhat - Ԥ��δ����״̬
    /* Step 1 */
    klm->rate = newRate - klm->bias;
    klm->angle += dt * klm->rate;
    
    //���¹������Э���� - ��ǰԤ�����Э����
    /* Step 2 */
    klm->P[0][0] += dt * (dt*klm->P[1][1] - klm->P[0][1] - klm->P[1][0] + klm->Q_angle);
    klm->P[0][1] -= dt * klm->P[1][1];
    klm->P[1][0] -= dt * klm->P[1][1];
    klm->P[1][1] += klm->Q_bias * dt;
    
    //��ɢ�������˲����������·��� - Measurement Update 
		//���㿨�������� - ���㿨��������
    /* Step 4 */
    klm->S = klm->P[0][0] + klm->R_measure;
    /* Step 5 */
    klm->K[0] = klm->P[0][0] / klm->S;
    klm->K[1] = klm->P[1][0] / klm->S;
    
    // ����ǶȺ�ƫ�� - ʹ�ò��� zk ���¹���ֵ ��newAngle��
    /* Step 3 */
    klm->y = newAngle - klm->angle;
    /* Step 6 */
    klm->angle += klm->K[0] * klm->y;
    klm->bias += klm->K[1] * klm->y;
    
    // ����������Э���� - �������Э����
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
} // �������ýǶȣ�Ӧ��������Ϊ��ʼ�Ƕ�


double getRate(struct Kalman* klm) 
{ 
    return klm->rate; 
} // ������ƫ��


/* ��Щ���ڵ����������˲���*/
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

