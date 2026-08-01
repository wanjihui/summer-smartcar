/*********************************************************************************************************************
* 文件名称          attitude
* 描述              AHRS 姿态解算 + 角度 PID + 舵机融合
* 依赖             zf_device_mpu6050 (mpu6050_acc/gyro 全局变量)
*********************************************************************************************************************/

#ifndef __ATTITUDE_H_
#define __ATTITUDE_H_

#include "zf_common_headfile.h"

// ==================== 姿态解算（AHRS） ====================
extern float atti_yaw;                     // 当前偏航角（°），左=负，右=正

void atti_init(void);                      // 四元数复位，上电调用一次
void atti_update(void);                    // 每10ms调用（TIM6 ISR），读取 mpu6050 全局变量更新偏航角
void atti_yaw_reset_ref(void);             // 复位 Angle PID 内部状态（停车后重新出发时调用）

// ==================== 角度 PID ====================
extern float angle_kp_a;                   // 基础P，菜单可调
extern float angle_kp_b;                   // 二次P系数 (kp = kp_a + err² × kp_b)
extern float angle_kd;                     // D 项，菜单可调
extern float angle_lowpass;                // D 项低通系数

extern float angle_pid_out;                // Angle PID 当前输出（Debug 只读）

float angle_pid_set(float target, float actual);
// target: 期望偏航角（上一帧 yaw）
// actual: 当前偏航角（atti_yaw）
// 返回:   舵机补偿量（°），范围 ±12°

// ==================== 舵机融合 ====================
extern float servo_fusion_alpha;           // 融合系数 0=纯视觉 1=纯角度PID

float servo_fusion(float angle_out, float IMU_out);
// angle_out: angle_pid_set() 的输出（偏差量，±12°）
// IMU_out:   servo_update() 的 PD 计算结果（偏差量 angle_cha，±13°）
// 返回:      融合后的偏差量，(1-α)×IMU_out + α×angle_out，限幅 ±13°

#endif
