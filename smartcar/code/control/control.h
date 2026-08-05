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
extern float servo_kd;       // 统一D系数
extern float gyro_kd;         // 陀螺仪阻尼
extern float servo_center;   // 中位角度
extern float servo_max_cha;  // 最大偏角
extern float servo_dead;     // 死区
extern float servo_max_add;  // 步进限制
extern int   servo_dir;      // 方向翻转
//=====电机=====
extern int   motor_base_duty; // 基础占空比
extern int   motor_curve_duty; // 弯道占空比
extern int   motor_max_duty;  // 最大正转占空比
extern float motor_kp;
extern float motor_kd;
extern int   motor_diff_max;  // 差速上限（脉冲/10ms），防漂
extern float ackermann_gain;  // Ackermann差速增益
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
void servo_calc(void);              /* 50Hz 主循环：P+P²+D → steer_dbg */
void servo_inject_gyro(void);       /* 200Hz ISR：gyro阻尼注入 → steer_dbg */
void motor_speed_calc(void);        /* 50Hz 主循环：速度过渡+LPF → motor_base_cms */
void motor_inject_ackermann(void);  /* 200Hz ISR：Ackermann差速 → Motor_*.Target */
void control_sync_params(void);     /* PID参数同步（菜单修改后调用） */

#endif