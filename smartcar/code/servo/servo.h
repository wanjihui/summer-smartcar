/*********************************************************************************************************************
* 文件名称          servo
* 适用平台          MM32F327X_G8P
* 描述              舵机控制模块（PWM 驱动）
* 引脚说明
*       模块管脚            单片机管脚
*       PWM                 A15 (TIM2_CH1)
*       GND                 舵机电源 GND 联通 核心板电源地 GND
*       VCC                 舵机电源输出（需独立供电 ≥5V）
*********************************************************************************************************************/

#ifndef __SERVO_H_
#define __SERVO_H_

#include "zf_common_headfile.h"

// 引脚定义
#define SERVO_PWM               (TIM2_PWM_CH1_A15)      // 舵机 PWM 引脚 PA15

// PWM 参数
#define SERVO_FREQ              (50)                     // PWM 频率 50Hz（舵机标准频率 范围 50~300Hz）

// 舵机角度限制
#define SERVO_ANGLE_MIN         (0.0f)                   // 舵机最小角度 0°
#define SERVO_ANGLE_MAX         (180.0f)                 // 舵机最大角度 180°
#define SERVO_ANGLE_L_LIMIT     (75.0f)                  // 左转极限角度（可根据机械结构调节）
#define SERVO_ANGLE_R_LIMIT     (105.0f)                 // 右转极限角度（可根据机械结构调节）
#define SERVO_ANGLE_CENTER      (90.0f)                  // 舵机中位角度

/*
 *  舵机占空比计算方式：
 *  舵机 0°~180° 对应控制脉冲的 0.5ms~2.5ms 高电平
 *  在 50Hz（周期 20ms）下：
 *    0°   → 0.5ms 脉冲 → 占空比 = 2.5%
 *    90°  → 1.5ms 脉冲 → 占空比 = 7.5%
 *    180° → 2.5ms 脉冲 → 占空比 = 12.5%
 *
 *  占空比计算公式：
 *    PWM_DUTY_MAX / (1000/freq) * (0.5 + angle/90)
 *  在 50Hz 下简化为：
 *    PWM_DUTY_MAX / 20 * (0.5 + angle/90)
 */
#define SERVO_DUTY(angle)       ((float)PWM_DUTY_MAX / (1000.0f / (float)SERVO_FREQ) * (0.5f + (float)(angle) / 90.0f))

// 编译时检查 PWM 频率合法性
#if (SERVO_FREQ < 50 || SERVO_FREQ > 300)
    #error "SERVO_FREQ must be between 50 and 300 Hz!"
#endif

// ============================================================
// API 函数声明
// ============================================================

// 舵机初始化（配置 PWM 输出）
void servo_init(void);

// 设置舵机角度
// 参数 angle: 目标角度 0.0° ~ 180.0°（自动限幅）
void servo_set_angle(float angle);

// 舵机回中（设为 90°）
void servo_set_center(void);

#endif
