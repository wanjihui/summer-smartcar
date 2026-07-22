#ifndef CONTROL_H
#define CONTROL_H
#include "zf_common_headfile.h"
#include "servo.h"
#include "motor.h"
#include "vision.h"

//全局变量

//=====舵机=====
extern float servo_kp;       
extern float servo_kd;     
extern float servo_center;   // 中位角度
extern float servo_max_cha;  // 最大偏角
extern float servo_dead;     // 死区
extern float servo_max_add;  // 步进限制
extern int   servo_dir;      // 方向翻转
//=====电机=====
extern int   motor_base_duty; // 基础占空比
extern int   motor_max_duty;  // 最大占空比
extern float motor_kp;       
extern float motor_kd;       

void control_init(void);
void control_update(void);

#endif