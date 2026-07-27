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
#include  "control.h"

// ============================================================
// 全局可调参数默认值
// ============================================================

// --- 双阈值 ---
#define DEFAULT_VIS_LOW        170
#define DEFAULT_VIS_HIGH       190
#define DEFAULT_LOOKAHEAD      30    // 预瞄距离（行）
#define DEFAULT_HALF_WIDTH     90    // 赛道半宽（px）

// --- 舵机 ---
#define DEFAULT_SERVO_KP1      0.70f  // 线性P（小偏差）
#define DEFAULT_SERVO_KP2      0.0f // 平方P（大偏差）
#define DEFAULT_SERVO_KD       0.0f  // D 系数
#define DEFAULT_GYRO_KD        0.0f   // 陀螺仪直道阻尼（0=关闭）
#define DEFAULT_GYRO_THRESHOLD 10.0f  // 陀螺仪触发阈值（|err|小于此值启用）
#define DEFAULT_SERVO_CENTER   95.2f // 中位角度
#define DEFAULT_SERVO_MAX_CHA  15.0f // 最大偏角
#define DEFAULT_SERVO_DEAD     2.0f  // 死区（像素）
#define DEFAULT_SERVO_MAX_ADD  4.0f  // 步进限制（度/帧）
#define DEFAULT_SERVO_DIR      1     // 0=正常 1=翻转

// --- 电机 ---
#define DEFAULT_MOTOR_BASE     25    // 基础占空比 %
#define DEFAULT_MOTOR_MAX      30    // 最大占空比 %
#define DEFAULT_MOTOR_BEND_CUT 0.0f // 弯道减速系数
#define DEFAULT_MOTOR_KP       0.0f  // 差速 P 系数
#define DEFAULT_MOTOR_KD       0.0f  // 差速 D 系数
#endif