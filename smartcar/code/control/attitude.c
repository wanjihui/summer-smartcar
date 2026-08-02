/*********************************************************************************************************************
* 文件名称          attitude
* 描述              AHRS 姿态解算（Mahony 互补滤波）+ 角度 PID + 舵机融合
* 定时要求          atti_update() 必须每 10ms 调用一次（TIM6 PIT ISR）
*********************************************************************************************************************/

#include "attitude.h"
#include "vision.h"     // err (视觉置信度自适应)
#include <math.h>

// ==================== 互补滤波参数（编译期常量） ====================
#define ATTI_KP             0.0001f        // 加速度计修正比例增益
#define ATTI_KI             0.0f           // 积分增益（暂关，避免漂移）
#define ATTI_DT             0.01f          // 采样周期 10ms（= TIM6 PIT）
#define ATTI_ACC_ALPHA      0.3f           // 加速度低通滤波系数

// ==================== 四元数状态 ====================
static float atti_q0 = 1.0f, atti_q1 = 0.0f, atti_q2 = 0.0f, atti_q3 = 0.0f;
static float atti_I_ex = 0.0f, atti_I_ey = 0.0f, atti_I_ez = 0.0f;

// ==================== 输出 ====================
float atti_yaw = 0.0f;                     // 当前偏航角（°），左=负，右=正

// ==================== 角度 PID 参数（菜单可调） ====================
float angle_kp_a     = 0.0f;               // 基础P
float angle_kp_b     = 0.0f;               // 二次P系数
float angle_kd       = 0.02f;              // D项
float angle_lowpass  = 0.8f;               // D项低通系数

// ==================== 融合系数 ====================
float servo_fusion_alpha = 0.0f;           // 0=纯视觉, 1=纯角度PID

// ==================== 诊断输出 ====================
float angle_pid_out = 0.0f;                // Angle PID 当前输出（Debug 只读）

// ==================== 内部：Angle PID 复位标志 ====================
static uint8_t angle_pid_need_reset = 1;   // 首帧/停车重启后复位内部状态

// ===== AHRS 姿态解算 + Angle PID + 舵机融合 已禁用 =====
#if 0

void atti_yaw_reset_ref(void)
{
    angle_pid_need_reset = 1;
}

// ---- 姿态解算初始化 ----
void atti_init(void)
{
    atti_q0 = 1.0f; atti_q1 = 0.0f; atti_q2 = 0.0f; atti_q3 = 0.0f;
    atti_I_ex = 0.0f; atti_I_ey = 0.0f; atti_I_ez = 0.0f;
    atti_yaw = 0.0f;
    angle_pid_out = 0.0f;
}

// ---- 姿态解算更新（TIM6 ISR，10ms 周期）----
void atti_update(void)
{
    float ax, ay, az;
    float gx, gy, gz;
    float norm;
    float ex, ey, ez;
    float q0, q1, q2, q3;
    static float acc_fx = 0.0f, acc_fy = 0.0f, acc_fz = 0.0f;
    static uint8_t first = 1;

    // 1. 读取加速度计原始值并转换为 g（低通滤波）
    ax = mpu6050_acc_transition(mpu6050_acc_x);
    ay = mpu6050_acc_transition(mpu6050_acc_y);
    az = mpu6050_acc_transition(mpu6050_acc_z);
    if (first)
    {
        acc_fx = ax; acc_fy = ay; acc_fz = az;
        first = 0;
    }
    else
    {
        acc_fx = ATTI_ACC_ALPHA * ax + (1.0f - ATTI_ACC_ALPHA) * acc_fx;
        acc_fy = ATTI_ACC_ALPHA * ay + (1.0f - ATTI_ACC_ALPHA) * acc_fy;
        acc_fz = ATTI_ACC_ALPHA * az + (1.0f - ATTI_ACC_ALPHA) * acc_fz;
    }

    // 2. 读取陀螺仪原始值并转换为 rad/s（Z轴减零偏）
    gx = mpu6050_gyro_transition(mpu6050_gyro_x) * 3.1415926f / 180.0f;
    gy = mpu6050_gyro_transition(mpu6050_gyro_y) * 3.1415926f / 180.0f;
    gz = mpu6050_gyro_transition(mpu6050_gyro_z - mpu6050_gyro_z_offset) * 3.1415926f / 180.0f;

    // 3. 归一化加速度计（零向量守卫：I2C 故障时跳过本周期）
    float a2 = acc_fx * acc_fx + acc_fy * acc_fy + acc_fz * acc_fz;
    if (a2 < 1e-6f) return;   // 无有效加速度数据，保持四元数不变
    norm = 1.0f / sqrtf(a2);
    ax = acc_fx * norm;
    ay = acc_fy * norm;
    az = acc_fz * norm;

    // 4. 估计重力方向（从当前四元数）
    q0 = atti_q0; q1 = atti_q1; q2 = atti_q2; q3 = atti_q3;
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 5. 误差 = 测量加速度 × 估计重力（叉积）
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 6. PI 修正陀螺仪
    float halfT = 0.5f * ATTI_DT;
    atti_I_ex += halfT * ex;
    atti_I_ey += halfT * ey;
    atti_I_ez += halfT * ez;
    gx = gx + ATTI_KP * ex + ATTI_KI * atti_I_ex;
    gy = gy + ATTI_KP * ey + ATTI_KI * atti_I_ey;
    gz = gz + ATTI_KP * ez + ATTI_KI * atti_I_ez;

    // 7. 一阶龙格库塔更新四元数
    q0 = atti_q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = atti_q1 + ( atti_q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = atti_q2 + ( atti_q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = atti_q3 + ( atti_q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 8. 归一化四元数
    norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    atti_q0 = q0 * norm; atti_q1 = q1 * norm;
    atti_q2 = q2 * norm; atti_q3 = q3 * norm;

    // 9. 提取偏航角 Yaw（°），左=负，右=正
    atti_yaw = atan2f(2.0f * (atti_q1 * atti_q2 + atti_q0 * atti_q3),
                      -2.0f * atti_q2 * atti_q2 - 2.0f * atti_q3 * atti_q3 + 1.0f)
               * 57.29578f;
}

// ---- 角度 PID：偏航角稳定控制 ----
float angle_pid_set(float target, float actual)
{
    static uint8_t first = 1;
    static float angle_pid_outp = 0.0f;  // 上一帧误差
    static float angle_pid_outd = 0.0f;  // 上一帧D项滤波值

    float error = target - actual;

    if (first || angle_pid_need_reset)
    {
        angle_pid_outp = error;
        angle_pid_outd = 0.0f;
        first = 0;
        angle_pid_need_reset = 0;
        angle_pid_out = 0.0f;
        return 0.0f;
    }

    // D 项：一阶低通滤波后的误差变化率
    float error_delta = error - angle_pid_outp;
    angle_pid_outd = error_delta * angle_lowpass
                   + angle_pid_outd * (1.0f - angle_lowpass);
    angle_pid_outp = error;

    // 非线性增益：kp = kp_a + err² × kp_b
    float kp = angle_kp_a + (error * error) * angle_kp_b;

    // 视觉置信度自适应：弯道（|err|大）→ 压低 Angle PID，不拖转向
    {
        float abs_img_err = (err > 0.0f) ? err : -err;
        if (abs_img_err >= 12.0f) kp = angle_kp_a * 0.1f;
    }

    // 位置式 PD
    float out = -(kp * error + angle_kd * angle_pid_outd);

    // 限幅 ±12°
    if (out > 12.0f)  out = 12.0f;
    if (out < -12.0f) out = -12.0f;

    angle_pid_out = out;
    return out;
}

// ---- 舵机融合：α×Angle_PID + (1-α)×IMU_PD ----
float servo_fusion(float angle_out, float IMU_out)
{
    float out = servo_fusion_alpha * angle_out + (1.0f - servo_fusion_alpha) * IMU_out;
    if (out != out) out = 0.0f;      // NaN 兜底（IMU 故障时安全回退）
    if (out > 13.0f)  out = 13.0f;
    if (out < -13.0f) out = -13.0f;
    return out;
}

#endif // AHRS 姿态解算 + Angle PID + 舵机融合 已禁用
