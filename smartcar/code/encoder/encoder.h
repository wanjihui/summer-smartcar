/*********************************************************************************************************************
* 文件名称          encoder
* 适用平台          MM32F327X_G8P
* 描述              双路正交编码器模块
* 引脚说明
*       编码器1 (左电机编码器)
*       A/LSB               B4  (TIM3_ENCODER_CH1)
*       B/DIR               B5  (TIM3_ENCODER_CH2)
*       GND                 GND
*       3V3                 3V3
*       编码器2 (右电机编码器)
*       A/LSB               B6  (TIM4_ENCODER_CH1)
*       B/DIR               B7  (TIM4_ENCODER_CH2)
*       GND                 GND
*       3V3                 3V3
*********************************************************************************************************************/

#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "zf_common_headfile.h"

// 编码器硬件定义
#define ENCODER_LEFT            (TIM3_ENCODER)          // 左电机编码器 使用 TIM3
#define ENCODER_LEFT_A          (TIM3_ENCODER_CH1_B4)   // 左编码器 A 相
#define ENCODER_LEFT_B          (TIM3_ENCODER_CH2_B5)   // 左编码器 B 相

#define ENCODER_RIGHT           (TIM4_ENCODER)          // 右电机编码器 使用 TIM4
#define ENCODER_RIGHT_A         (TIM4_ENCODER_CH1_B6)   // 右编码器 A 相
#define ENCODER_RIGHT_B         (TIM4_ENCODER_CH2_B7)   // 右编码器 B 相

#define ENCODER_PIT             (TIM6_PIT)              // 编码器采集周期定时器
#define ENCODER_PIT_PERIOD_MS   10                      // 采集周期 10ms（100Hz）

// 全局编码器速度变量（在 PIT 中断中更新，单位：脉冲/采集周期）
extern int16 encoder_left_speed;
extern int16 encoder_right_speed;

// 编码器初始化（正交解码 + PIT 定时中断）
void encoder_init(void);

// 获取左编码器速度（脉冲/ENCODER_PIT_PERIOD_MS ms）
int16 encoder_get_left_speed(void);

// 获取右编码器速度（脉冲/ENCODER_PIT_PERIOD_MS ms）
int16 encoder_get_right_speed(void);

// PIT 中断回调（由 isr.c 的 TIM6_IRQHandler 调用）
void encoder_pit_callback(void);

#endif
