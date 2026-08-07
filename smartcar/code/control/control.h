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
extern int   servo_dir;      // 方向翻转
//=====电机=====
extern int   motor_base_duty; // 基础占空比
extern int   motor_curve_duty; // 弯道占空比
extern int   motor_max_duty;  // 最大正转占空比
extern float ackermann_gain;  // Ackermann差速增益
extern float motor_speed_kp_l;  // 左轮 Kp
extern float motor_speed_ki_l;  // 左轮 Ki
extern float motor_speed_kd_l;  // 左轮 Kd
extern float motor_speed_kp_r;  // 右轮 Kp
extern float motor_speed_ki_r;  // 右轮 Ki
extern float motor_speed_kd_r;  // 右轮 Kd
/* 小车运行状态机 */
typedef enum {
    CAR_IDLE      = 0,  // 空闲，等待发车指令
    CAR_LAUNCHING = 1,  // 起步斜坡中（500ms线性爬升）
    CAR_RUNNING   = 2,  // 正常运行
    CAR_STOP      = 3,  // 停车过渡（清理后自动切 IDLE）
} CarState;

#define LAUNCH_RAMP_STEPS  25   // 起步斜坡: 25步 × 20ms(帧周期) = 500ms

extern volatile CarState car_state;    // 当前状态（ISR + 主循环）
extern volatile bool     car_cmd;      // 菜单代理: false=停车 true=发车
extern float motor_base_cms;           // 速度目标缓存（motor_speed_calc写，ISR读）
extern float gyro_z_dbg;       // 诊断：量化后 gyro_z
extern float steer_dbg;        // 诊断：舵机偏离中心角度

void control_init(void);
void control_update(void);
void control_vofa_send(void);
void servo_calc(void);              /* 50Hz 主循环：P+P²+D → steer_dbg */
void motor_speed_calc(void);        /* 50Hz 主循环：速度过渡+LPF → motor_base_cms */
void motor_inject_ackermann(void);  /* 200Hz ISR：Ackermann差速 → Motor_*.Target */
void control_sync_params(void);     /* PID参数同步（菜单修改后调用） */
void control_state_launch(void);    /* IDLE → LAUNCHING，重置斜坡计数器 */
void control_state_stop(void);      /* → STOP，主循环清理后自动切 IDLE */

#endif