#ifndef PID_H
#define PID_H
#include "zf_common_headfile.h"

typedef struct {
    float kp, ki, kd;               // PID 系数
    float integral;                  // 积分累加（当前 ki=0，不用）
    float err;                       // 当前误差
    float last_err;                  // 上一次误差
    float prev_err;                  // 上上次误差（增量式用）
    float output;                    // 计算结果
    float i_limit;                   // 积分限幅
    float omin, omax;                // 输出限幅
} pid_t;

void pid_init(pid_t *p, float kp, float ki, float kd,
              float ilimit, float omin, float omax);//pid参数初始化
float pid_step(pid_t *p, float error);   //增量式
float pid_pos(pid_t *p, float error);    //位置式

#endif