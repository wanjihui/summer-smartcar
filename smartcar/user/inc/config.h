#ifndef _CONFIG_H_
#define _CONFIG_H_

//逐飞模块
#include "zf_common_headfile.h"
#include "zf_device_key.h"

//移植模块
#include "motor.h"  //电机驱动
#include "encoder.h"//编码器
#include "mpu6050.h"//MPU6050
#include "servo.h"  //舵机

//自写模块
#include  "menu.h"  //菜单框架
#include  "Mymenu.h"//菜单具体实现
#include  "vision.h"//视觉算法
#include  "pid.h"   //pid框架
#include  "control.h"

// ============================================================
// 全局可调参数默认值
// ============================================================

// --- 双阈值 ---
#define DEFAULT_VIS_LOW        200
#define DEFAULT_VIS_HIGH       240

// --- 舵机 ---
#define DEFAULT_SERVO_KP       0.3f  // P 系数
#define DEFAULT_SERVO_KD       0.05f // D 系数
#define DEFAULT_SERVO_CENTER   95.2f // 中位角度
#define DEFAULT_SERVO_MAX_CHA  15.0f // 最大偏角
#define DEFAULT_SERVO_DEAD     2.0f  // 死区（像素）
#define DEFAULT_SERVO_MAX_ADD  2.0f  // 步进限制（度/帧）
#define DEFAULT_SERVO_DIR      1     // 0=正常 1=翻转

// --- 电机 ---
#define DEFAULT_MOTOR_BASE     10    // 基础占空比 %
#define DEFAULT_MOTOR_MAX      30    // 最大占空比 %
#define DEFAULT_MOTOR_KP       0.0f  // 差速 P 系数
#define DEFAULT_MOTOR_KD       0.0f  // 差速 D 系数
#endif