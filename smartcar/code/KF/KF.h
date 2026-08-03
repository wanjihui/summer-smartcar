/*
 * KF.h — 一维卡尔曼滤波器
 * 移植自 MCZSCS-V2-MM32 (P3)
 *
 * 用途: 对编码器速度信号做自适应低通滤波
 *   Q 越小 → 越信任模型(更平滑, 响应慢)
 *   R 越小 → 越信任测量(更灵敏, 噪声多)
 */

#ifndef __KF_H__
#define __KF_H__

#include "zf_common_headfile.h"

/* ---- 左右轮速度滤波参数 ---- */
#define KF_SPEED_Q  (0.1f)     // 过程噪声协方差（τ≈10ms，一周期响应）
#define KF_SPEED_R  (0.1f)     // 测量噪声协方差

typedef struct {
    float x_hat;  // 估计值（滤波输出）
    float P;      // 估计方差
    float Q;      // 过程噪声协方差
    float R;      // 测量噪声协方差
    float Kk;     // 卡尔曼增益
} KalmanFilter;

extern KalmanFilter Kf_L;  // 左轮速度滤波器
extern KalmanFilter Kf_R;  // 右轮速度滤波器

void  KF_init       (KalmanFilter *kf, float Q, float R);
float kalman_update (KalmanFilter *kf, float measurement);

#endif
