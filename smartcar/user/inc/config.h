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
#define DEFAULT_ASC_FAR        30   // ASC采样远行（顶部行），10≈90行范围
#define DEFAULT_STRAIGHT_FAR   30    // 直线判定采样远行（顶部行），默认30

// --- 自适应预瞄常量（编译期，基于调好的far，按|err|分段偏移）---
#define ADAPT_ERR_TH1     10    // 小弯err阈值（像素），|err|≤th1=直道
#define ADAPT_ERR_TH2     20    // 大弯err阈值（像素），th1<|err|≤th2=小弯
#define ADAPT_OFF_STR    (-10)  // 直道far偏移（负=看更远）
#define ADAPT_OFF_SML      5    // 小弯far偏移
#define ADAPT_OFF_BIG     10    // 大弯far偏移

// --- 舵机 ---
#define DEFAULT_SERVO_KP1      0.85f  // 基础P
#define DEFAULT_SERVO_KP2      0.0f   // 二次P: err×|err|
#define DEFAULT_SERVO_KD       0.06f  // D 系数
#define DEFAULT_GYRO_KD        0.02f // 陀螺仪前馈（直道），正=减Gyro项
#define DEFAULT_GYRO_KD_CURVE  0.02f  // 陀螺仪前馈（弯道），正=减Gyro项
#define DEFAULT_SERVO_CENTER   95.2f // 中位角度
#define DEFAULT_SERVO_MAX_CHA  13.0f // 最大偏角
#define DEFAULT_SERVO_DEAD     2.0f  // 死区（像素）
#define DEFAULT_SERVO_MAX_ADD  5.5f  // 步进限制（度/帧）
#define DEFAULT_SERVO_DIR      1     // 0=正常 1=翻转

// --- 电机 ---
#define DEFAULT_MOTOR_BASE     21    // 基础占空比 %
#define DEFAULT_MOTOR_CURVE_DUTY 18  // 弯道占空比 %（is_straight判定为弯道时使用）
#define DEFAULT_MOTOR_MAX      30    // 最大占空比 %
#define DEFAULT_MOTOR_KP       0.0f  // 差速 P 系数
#define DEFAULT_MOTOR_KD       0.0f  // 差速 D 系数 
#define DEFAULT_MOTOR_DIFF_MAX 8     // 差速上限（%）防漂
#endif