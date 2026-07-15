/*********************************************************************************************************************
* 文件名称          motor
* 适用平台          MM32F327X_G8P
* 描述              DRV8701E 双电机驱动模块
* 引脚说明
*       模块管脚            单片机管脚
*       1DIR                A0
*       1PWM                A1 (TIM5_CH2)
*       GND                 GND
*       2DIR                A2
*       2PWM                A3 (TIM5_CH4)
*       GND                 GND
*       接线端子 +          电池正极
*       接线端子 -          电池负极
*********************************************************************************************************************/

#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

// 引脚定义
#define MOTOR_DIR_L             (A0)                // 左电机方向引脚
#define MOTOR_PWM_L             (TIM5_PWM_CH2_A1)   // 左电机 PWM 引脚
#define MOTOR_DIR_R             (A2)                // 右电机方向引脚
#define MOTOR_PWM_R             (TIM5_PWM_CH4_A3)   // 右电机 PWM 引脚

#define MOTOR_PWM_FREQ          17000               // PWM 频率 17KHz
#define MOTOR_MAX_DUTY          50                  // 最大占空比 50% 防止过流

// 电机初始化
void motor_init(void);

// 设置左电机占空比 范围 -MOTOR_MAX_DUTY ~ +MOTOR_MAX_DUTY（负值反转 正值正转）
void motor_set_left(int8_t duty);

// 设置右电机占空比 范围 -MOTOR_MAX_DUTY ~ +MOTOR_MAX_DUTY（负值反转 正值正转）
void motor_set_right(int8_t duty);

// 同时设置两个电机
void motor_set_both(int8_t left_duty, int8_t right_duty);

// 停止两个电机
void motor_stop(void);

#endif
