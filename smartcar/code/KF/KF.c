/*
 * KF.c — 一维卡尔曼滤波器
 * 移植自 MCZSCS-V2-MM32 (P3)
 */

#include "KF.h"

KalmanFilter Kf_L = { .x_hat = 0, .P = 1.0f, .Q = KF_SPEED_Q, .R = KF_SPEED_R, .Kk = 0 };
KalmanFilter Kf_R = { .x_hat = 0, .P = 1.0f, .Q = KF_SPEED_Q, .R = KF_SPEED_R, .Kk = 0 };

void KF_init(KalmanFilter *kf, float Q, float R)
{
    kf->Kk    = 0.0f;
    kf->P     = 1.0f;
    kf->Q     = Q;
    kf->R     = R;
    kf->x_hat = 0.0f;
}

float kalman_update(KalmanFilter *kf, float measurement)
{
    /* 预测 */
    float predicted_x_hat = kf->x_hat;
    float predicted_P     = kf->P + kf->Q;

    /* 卡尔曼增益 */
    kf->Kk = predicted_P / (predicted_P + kf->R);

    /* 更新 */
    kf->x_hat = predicted_x_hat
              + kf->Kk * (measurement - predicted_x_hat);
    kf->P = (1.0f - kf->Kk) * predicted_P;

    return kf->x_hat;
}
