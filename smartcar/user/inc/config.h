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

// --- 预瞄位置 ---
#define DEFAULT_ASC_FAR        20   // ASC采样远行（顶部行），10≈90行范围
// --- 自适应预瞄常量（编译期，基于调好的far，按|err|分段偏移）---
#define ADAPT_ERR_TH1     10    // 小弯err阈值（像素），|err|≤th1=直道
#define ADAPT_ERR_TH2     20    // 大弯err阈值（像素），th1<|err|≤th2=小弯
#define ADAPT_OFF_STR    (-10)  // 直道far偏移（负=看更远）
#define ADAPT_OFF_SML      5    // 小弯far偏移
#define ADAPT_OFF_BIG     20    // 大弯far偏移（减10前瞻）

// --- 舵机 ---
#define DEFAULT_SERVO_KP1      0.3f  // 基础P 
#define DEFAULT_SERVO_KP2      0.026f  // 二次P: err×|err|
#define DEFAULT_SERVO_KD       0.0f  // D系数（弯道/大err），弯道无震荡无需阻尼
#define DEFAULT_SERVO_KD_STR   0.35f  // D系数（直道/小err）
#define DEFAULT_GYRO_KD        0.03f  // 陀螺仪阻尼（直道）
#define DEFAULT_GYRO_KD_CURVE  0.03f  // 陀螺仪阻尼（弯道）
#define DEFAULT_SERVO_CENTER   95.2f // 中位角度
#define DEFAULT_SERVO_MAX_CHA  13.0f // 最大偏角
#define DEFAULT_SERVO_DEAD     2.0f  // 死区（像素）
#define DEFAULT_SERVO_MAX_ADD  5.5f  // 步进限制（度/帧）
#define DEFAULT_SERVO_DIR      1     // 0=正常 1=翻转

// --- 电机目标速度---
#define DEFAULT_MOTOR_BASE       14    // 直道目标速度(dm/s) = 1.0m/s
#define DEFAULT_MOTOR_CURVE_DUTY 11     // 弯道目标速度(dm/s) = 0.7m/s
#define DEFAULT_MOTOR_MAX        40    // 最大占空比（PID Out限制）
#define DEFAULT_MOTOR_KP         0.05f // 差速 P 系数
#define DEFAULT_MOTOR_KD         0.0f  // 差速 D 系数
#define DEFAULT_MOTOR_DIFF_MAX   13    // 差速上限（cm/s）（旧方案保留）
#define DEFAULT_ACKERMANN_GAIN  1.0f  // Ackermann差速增益

// --- 速度环PID（增量式，ISR 5ms，200Hz，左右独立）---
#define DEFAULT_SPEED_KP_L       0.5f  // 左轮 Kp（高速降增益防抖）
#define DEFAULT_SPEED_KI_L       0.12f // 左轮 Ki
#define DEFAULT_SPEED_KD_L       0.10f // 左轮 Kd
#define DEFAULT_SPEED_KP_R       0.6f  // 右轮 Kp（编码器修复后降增益）
#define DEFAULT_SPEED_KI_R       0.12f // 右轮 Ki
#define DEFAULT_SPEED_KD_R       0.10f // 右轮 Kd
#define DEFAULT_SPEED_DELTA_MAX  6.0f  // 单次增量限幅（duty/5ms）

// --- 姿态角融合 ---
#define DEFAULT_ANGLE_KP_A          0.0f   // Angle PID 基础P
#define DEFAULT_ANGLE_KP_B          0.0f   // Angle PID 二次P系数
#define DEFAULT_ANGLE_KD            0.02f  // Angle PID D项
#define DEFAULT_ANGLE_LOWPASS       0.8f   // Angle PID D项低通
#define DEFAULT_SERVO_FUSION_ALPHA  0.0f   // 融合系数 0=纯视觉
#endif