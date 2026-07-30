#ifndef CONTROL_H
#define CONTROL_H
#include "zf_common_headfile.h"
#include "servo.h"
#include "motor.h"
#include "vision.h"

//全局变量

//=====舵机=====
extern float servo_kp1;       // 基础P
extern float servo_kp2;       // 二次P: err×|err|
extern float servo_kd;
extern float gyro_kd;         // 陀螺仪直道阻尼
extern float gyro_kd_curve;   // 陀螺仪弯道阻尼
extern float servo_center;   // 中位角度
extern float servo_max_cha;  // 最大偏角
extern float servo_dead;     // 死区
extern float servo_max_add;  // 步进限制
extern int   servo_dir;      // 方向翻转
//=====电机=====
extern int   motor_base_duty; // 基础占空比
extern int   motor_max_duty;  // 最大正转占空比
extern float motor_bend_cut; // 弯道减速系数
extern float motor_kp;
extern float motor_kd;
extern int   motor_diff_max;  // 差速上限（%），防漂
extern bool  car_run;        // 是否运行小车（true=搜线+控制, false=仅菜单）

void control_init(void);
void control_update(void);

#endif