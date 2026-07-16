/*********************************************************************************************************************
* 文件名称          mpu6050
* 适用平台          MM32F327X_G8P
* 描述              MPU6050 六轴陀螺仪/加速度计模块
* 引脚说明
*       模块管脚            单片机管脚
*       SCL                 B13 (软 IIC SCL)
*       SDA                 B15 (软 IIC SDA)
*       VCC                 3.3V
*       GND                 GND
*********************************************************************************************************************/

#ifndef __MPU6050_H_
#define __MPU6050_H_

#include "zf_common_headfile.h"

// 初始化 MPU6050（带重试，失败时 LED1 闪烁提示）
void mpu6050_module_init(void);

// PIT 中断回调（由 isr.c 的 TIM6_IRQHandler 调用，与编码器共享 TIM6）
void mpu6050_pit_callback(void);

// 获取三轴加速度原始值
int16 mpu6050_get_acc_x(void);
int16 mpu6050_get_acc_y(void);
int16 mpu6050_get_acc_z(void);

// 获取三轴角速度原始值
int16 mpu6050_get_gyro_x(void);
int16 mpu6050_get_gyro_y(void);
int16 mpu6050_get_gyro_z(void);

#endif
