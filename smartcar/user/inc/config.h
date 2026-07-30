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
#define DEFAULT_LOOKAHEAD      45    // 预瞄距离（行）

// --- 舵机 ---
#define DEFAULT_SERVO_KP1      1.0f  // 基础P
#define DEFAULT_SERVO_KP2      0.004f   // 二次P: err×|err|
#define DEFAULT_SERVO_KD       0.1f  // D 系数
#define DEFAULT_GYRO_KD        -0.006f   // 陀螺仪直道阻尼
#define DEFAULT_GYRO_KD_CURVE  -0.0068f  // 陀螺仪弯道阻尼（0=关闭）
#define DEFAULT_SERVO_CENTER   95.2f // 中位角度
#define DEFAULT_SERVO_MAX_CHA  15.0f // 最大偏角
#define DEFAULT_SERVO_DEAD     2.0f  // 死区（像素）
#define DEFAULT_SERVO_MAX_ADD  5.5f  // 步进限制（度/帧）
#define DEFAULT_SERVO_DIR      1     // 0=正常 1=翻转
#define DEFAULT_ERR_ALPHA      1.0f // err平滑系数（EMA，0.2=强抑抖 0.6=快响应）

// --- 电机 ---
#define DEFAULT_MOTOR_BASE     24    // 基础占空比 %
#define DEFAULT_MOTOR_MAX      30    // 最大占空比 %
#define DEFAULT_MOTOR_BEND_CUT 0.01f // 弯道减速系数
#define DEFAULT_MOTOR_KP       0.001f  // 差速 P 系数
#define DEFAULT_MOTOR_KD       0.0f  // 差速 D 系数
#define DEFAULT_MOTOR_DIFF_MAX 8     // 差速上限（%）防漂
#endif