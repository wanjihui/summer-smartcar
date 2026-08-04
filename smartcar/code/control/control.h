#ifndef CONTROL_H
#define CONTROL_H
#include "zf_common_headfile.h"
#include "servo.h"
#include "motor.h"
#include "vision.h"
#include "attitude.h"
#include "pid.h"
#include "KF.h"

//全局变量

//=====舵机=====
extern float servo_kp1;       // 基础P
extern float servo_kp2;       // 二次P: err×|err|
extern float servo_kd;
extern float servo_kd_str;    // 直道/小err D
extern float gyro_kd;         // 陀螺仪直道阻尼
extern float gyro_kd_curve;   // 陀螺仪弯道阻尼
extern float servo_center;   // 中位角度
extern float servo_max_cha;  // 最大偏角
extern float servo_dead;     // 死区
extern float servo_max_add;  // 步进限制
extern int   servo_dir;      // 方向翻转
//=====电机=====
extern int   motor_base_duty; // 基础占空比
extern int   motor_curve_duty; // 弯道占空比（is_straight判定为弯道时使用）
extern int   motor_max_duty;  // 最大正转占空比
extern float motor_kp;
extern float motor_kd;
extern int   motor_diff_max;  // 差速上限（脉冲/10ms），防漂
extern float motor_speed_kp_l;  // 左轮 Kp
extern float motor_speed_ki_l;  // 左轮 Ki
extern float motor_speed_kd_l;  // 左轮 Kd
extern float motor_speed_kp_r;  // 右轮 Kp
extern float motor_speed_ki_r;  // 右轮 Ki
extern float motor_speed_kd_r;  // 右轮 Kd
extern volatile bool  car_run;          // 是否运行小车
extern float gyro_z_dbg;       // 诊断：量化后 gyro_z
extern float steer_dbg;        // 诊断：舵机偏离中心角度

void control_init(void);
void control_update(void);
void control_vofa_send(void);

#endif