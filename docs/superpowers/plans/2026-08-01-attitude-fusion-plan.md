# 姿态角融合 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 引入 AHRS 互补滤波 + Angle PID + Servo Fusion，直道抑制蛇形走线（方案A加法叠加层）

**Architecture:** 新建 `attitude.c/h` 包含 Mahony 互补滤波（100Hz TIM6 ISR）、非线性 Angle PID（视觉置信度自适应）、线性融合。TIM6 ISR 接管 MPU6050 读取，主循环删掉 `mpu6050_update()`，`servo_update()` 末尾加融合调用。

**Tech Stack:** MM32F327X (ARM Cortex-M3 120MHz), 逐飞 ZF 库 V3.11.1, Keil MDK 5.37, 软IIC MPU6050

## Global Constraints

- α=0 起步，完全等价现有行为，不可引入回归
- 所有浮点限幅 ±12°（舵机物理范围）
- ISR 内不得调用阻塞函数（软IIC 除外，200µs 可接受）
- 菜单参数需与现有 PID/Debug 结构一致
- 编译 0 error 0 warning

---

## 文件结构

```
smartcar/code/control/
├── attitude.h    (NEW — AHRS + Angle PID + Fusion 接口)
├── attitude.c    (NEW — 全部实现)
├── control.h     (MODIFY — 新增 extern)
└── control.c     (MODIFY — servo_update 加 fusion)

smartcar/user/
├── inc/config.h  (MODIFY — 新增默认参数宏)
├── src/main.c    (MODIFY — atti_init, 删 mpu6050_update)
└── src/isr.c     (MODIFY — TIM6 ISR 加 AHRS)

smartcar/code/menu/
└── Mymenu.c      (MODIFY — 菜单新增 PID/Debug 项)
```

---

### Task 1: 创建 attitude.h — 接口声明

**Files:**
- Create: `smartcar/code/control/attitude.h`

**Interfaces:**
- Produces: `atti_yaw`, `angle_pid_out`, `servo_fusion_alpha`, `angle_kp_a`, `angle_kp_b`, `angle_kd`, `angle_lowpass`, `atti_init()`, `atti_update()`, `angle_pid_set()`, `servo_fusion()`

- [ ] **Step 1: 写入 attitude.h**

```c
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
// angle_out: angle_pid_set() 的输出
// IMU_out:   servo_update() 的 PD 计算结果（舵机目标角度）
// 返回:      (1-α)×IMU_out + α×angle_out, 限幅 ±12°

#endif
```

- [ ] **Step 2: 编译验证头文件语法**

在 Keil 中 Rebuild，预期：找不到 attitude.c 的链接错误（`atti_init` 等未定义），头文件本身无语法错误。

---

### Task 2: 创建 attitude.c — AHRS + Angle PID + Fusion 实现

**Files:**
- Create: `smartcar/code/control/attitude.c`

**Interfaces:**
- Consumes: `mpu6050_acc_x/y/z`, `mpu6050_gyro_x/y/z`, `mpu6050_gyro_z_offset` (全局变量, zf_device_mpu6050.h)
- Consumes: `mpu6050_acc_transition()`, `mpu6050_gyro_transition()` (zf_device_mpu6050.h)
- Consumes: `err` (vision.h — 用于视觉置信度自适应)
- Produces: `atti_yaw`, `angle_pid_out`, `servo_fusion_alpha`, `angle_kp_a/b`, `angle_kd`, `angle_lowpass` — 所有接口实现

- [ ] **Step 1: 写入 attitude.c**

```c
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
float angle_pid_out = 0.0f;                // Angle PID 当前输出

// ==================== 内部状态 ====================
static float prev_angle_yaw = 0.0f;        // 上一帧偏航角（angle_pid_set 用）

// ---- 姿态解算初始化 ----
void atti_init(void)
{
    atti_q0 = 1.0f; atti_q1 = 0.0f; atti_q2 = 0.0f; atti_q3 = 0.0f;
    atti_I_ex = 0.0f; atti_I_ey = 0.0f; atti_I_ez = 0.0f;
    atti_yaw = 0.0f;
    prev_angle_yaw = 0.0f;
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

    // 2. 读取陀螺仪原始值并转换为 rad/s
    gx = mpu6050_gyro_transition(mpu6050_gyro_x) * 3.1415926f / 180.0f;
    gy = mpu6050_gyro_transition(mpu6050_gyro_y) * 3.1415926f / 180.0f;
    gz = mpu6050_gyro_transition(mpu6050_gyro_z) * 3.1415926f / 180.0f;

    // 3. 归一化加速度计（MM32 有硬件 FPU，直接用 sqrtf）
    norm = 1.0f / sqrtf(acc_fx * acc_fx + acc_fy * acc_fy + acc_fz * acc_fz);
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

    if (first)
    {
        angle_pid_outp = error;
        first = 0;
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

    // 视觉置信度自适应：err 小（视觉跟踪好）→ 压低 Angle PID 介入
    extern volatile float err;  // vision.h 的像素偏差
    {
        float abs_img_err = (err > 0.0f) ? err : -err;
        if (abs_img_err < 12.0f) kp = angle_kp_a;
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
    if (out > 12.0f)  out = 12.0f;
    if (out < -12.0f) out = -12.0f;
    return out;
}
```

- [ ] **Step 2: 编译验证**

在 Keil 中 Rebuild。预期：`attitude.c` 编译通过（0 error 0 warning），链接报 `main.c` 中 `atti_init` 未调用（正常，Task 7 修复）。

---

### Task 3: 修改 control.h — 暴露融合相关 extern

**Files:**
- Modify: `smartcar/code/control/control.h`

**Interfaces:**
- Produces: `angle_pid_out` (extern, Debug 菜单用)

- [ ] **Step 1: 在 control.h 末尾加声明**

打开 `smartcar/code/control/control.h`，在 `void control_update(void);` 之前加入：

```c
// ===== 姿态角融合（Debug 菜单用） =====
extern float angle_pid_out;
```

确切位置：第 31 行 `void control_init(void);` 上方。

修改后 control.h 末尾变为：

```c
// ===== 姿态角融合（Debug 菜单用） =====
extern float angle_pid_out;

void control_init(void);
void control_update(void);

#endif
```

- [ ] **Step 2: 编译验证**

Rebuild，预期 0 error 0 warning。

---

### Task 4: 修改 config.h — 加默认参数宏

**Files:**
- Modify: `smartcar/user/inc/config.h`

**Interfaces:**
- Produces: `DEFAULT_ANGLE_KP_A`, `DEFAULT_ANGLE_KP_B`, `DEFAULT_ANGLE_KD`, `DEFAULT_ANGLE_LOWPASS`, `DEFAULT_SERVO_FUSION_ALPHA`

- [ ] **Step 1: 在 config.h 末尾 `#endif` 前加入**

打开 `smartcar/user/inc/config.h`，在 `#define DEFAULT_MOTOR_DIFF_MAX 8` 之后、`#endif` 之前加入：

```c

// --- 姿态角融合 ---
#define DEFAULT_ANGLE_KP_A          0.0f   // Angle PID 基础P
#define DEFAULT_ANGLE_KP_B          0.0f   // Angle PID 二次P系数
#define DEFAULT_ANGLE_KD            0.02f  // Angle PID D项
#define DEFAULT_ANGLE_LOWPASS       0.8f   // Angle PID D项低通
#define DEFAULT_SERVO_FUSION_ALPHA  0.0f   // 融合系数 0=纯视觉
```

- [ ] **Step 2: 编译验证**

Rebuild，预期 0 error 0 warning（宏本身不产生代码，仅在 Task 5/6 引用时生效）。

---

### Task 5: 修改 isr.c — TIM6 ISR 加入 AHRS 更新

**Files:**
- Modify: `smartcar/user/src/isr.c:109-117`

**Interfaces:**
- Consumes: `mpu6050_get_acc()`, `mpu6050_get_gyro()` (zf_device_mpu6050.h)
- Consumes: `atti_update()` (attitude.h)

- [ ] **Step 1: 在 isr.c 头部加 include**

在 `#include "mpu6050.h"` 之后加入：

```c
#include "attitude.h"
```

- [ ] **Step 2: 修改 TIM6_IRQHandler**

将 [isr.c:109-117](smartcar/user/src/isr.c#L109-L117)：

```c
void TIM6_IRQHandler (void)
{
    encoder_pit_callback();
    // 此处编写用户代码
    // MPU6050 读取已移至主循环（mpu6050_update），软IIC忙等不再占用中断

    // 此处编写用户代码
    TIM6->SR &= ~TIM6->SR;                                                      // 清空中断状态
}
```

改为：

```c
void TIM6_IRQHandler (void)
{
    encoder_pit_callback();

    // MPU6050 读取 + AHRS 姿态解算（100Hz）
    mpu6050_get_acc();
    mpu6050_get_gyro();
    atti_update();

    TIM6->SR &= ~TIM6->SR;                                                      // 清空中断状态
}
```

- [ ] **Step 3: 编译验证**

Rebuild，预期 0 error 0 warning。

---

### Task 6: 修改 control.c — servo_update() 加融合调用

**Files:**
- Modify: `smartcar/code/control/control.c:83-91`

**Interfaces:**
- Consumes: `atti_yaw`, `angle_pid_set()`, `servo_fusion()` (attitude.h)
- Consumes: `servo_fusion_alpha`, `angle_kp_a`, `angle_kp_b`, `angle_kd`, `angle_lowpass` (attitude.h extern)

- [ ] **Step 1: 在 control.c 头部加 include**

在 `#include "config.h"` 之后、PID 参数定义之前加入：

```c
#include "attitude.h"
```

- [ ] **Step 2: 修改 servo_update() 函数末尾**

将 [control.c:83-91](smartcar/code/control/control.c#L83-L91)：

```c
    float target = servo_center + angle_cha;

    //----步进限制----
    float add = target - servo_last_angle;
    if (add >  servo_max_add) target = servo_last_angle + servo_max_add;
    if (add < -servo_max_add) target = servo_last_angle - servo_max_add;

    servo_set_angle(target);                       //舵机调整角度函数
    servo_last_angle = target;                     //存储当前目标角度
}
```

改为：

```c
    float raw_angle = servo_center + angle_cha;

    //---- Angle PID（偏航角稳定）----
    static float prev_yaw = 0.0f;
    float angle_out = angle_pid_set(prev_yaw, atti_yaw);
    prev_yaw = atti_yaw;

    //---- 舵机融合 ----
    float target = servo_fusion(angle_out, raw_angle);

    //----步进限制----
    float add = target - servo_last_angle;
    if (add >  servo_max_add) target = servo_last_angle + servo_max_add;
    if (add < -servo_max_add) target = servo_last_angle - servo_max_add;

    servo_set_angle(target);                       //舵机调整角度函数
    servo_last_angle = target;                     //存储当前目标角度
}
```

- [ ] **Step 3: 编译验证**

Rebuild，预期 0 error 0 warning。

---

### Task 7: 修改 main.c — atti_init + 删除 mpu6050_update

**Files:**
- Modify: `smartcar/user/src/main.c:79,92`

**Interfaces:**
- Consumes: `atti_init()` (attitude.h)

- [ ] **Step 1: 在 mpu6050_module_init() 之后加 atti_init()**

在 [main.c:79](smartcar/user/src/main.c#L79) `mpu6050_module_init();` 行后插入：

```c
    atti_init();                // 初始化姿态解算（六轴互补滤波 AHRS）
```

- [ ] **Step 2: 删除主循环中的 mpu6050_update()**

删除 [main.c:92](smartcar/user/src/main.c#L92) 这一行：

```c
    mpu6050_update();     //主循环读MPU6050，数据紧跟控制时刻（原TIM6中断100Hz读取已移除）
```

删除后该区域变为：

```c
    if(car_run)
    {
        control_update();     //舵机 电机控制
    }
```

- [ ] **Step 3: 编译验证**

Rebuild，预期 0 error 0 warning。

---

### Task 8: 修改 Mymenu.c — 菜单新增项

**Files:**
- Modify: `smartcar/code/menu/Mymenu.c:37-89`

**Interfaces:**
- Consumes: `servo_fusion_alpha`, `angle_kp_a`, `angle_kd` (attitude.h extern)
- Consumes: `atti_yaw`, `angle_pid_out` (attitude.h + control.h extern)

- [ ] **Step 1: 在 Mymenu.c 头部加 include**

在 `#include "config.h"` 之后加入：

```c
#include "attitude.h"
```

- [ ] **Step 2: PID 文件夹新增 3 个可调参数**

在 `dynamicCreate_Menu_Number(pid, "car_run", ...)` 之前（即 "电机" 组的最后一项之后）加入：

```c

        // 姿态角融合
        v = dynamicCreate_Menu_Number(pid, "fusion_a",  &servo_fusion_alpha, float_Box);
        Menu_Set_Limit(v, 0.0f, 1.0f, 0.05f);
        v = dynamicCreate_Menu_Number(pid, "ang_kpa",   &angle_kp_a, float_Box);
        Menu_Set_Limit(v, 0.0f, 5.0f, 0.01f);
        v = dynamicCreate_Menu_Number(pid, "ang_kd",    &angle_kd, float_Box);
        Menu_Set_Limit(v, 0.0f, 2.0f, 0.01f);
```

位置：在 `dynamicCreate_Menu_Number(pid, "car_run", &car_run, bool_Box);` 之前一行插入。

- [ ] **Step 3: Debug 文件夹新增 2 个只读诊断项**

在 `v = dynamicCreate_Menu_Number(dbg, "range", ...)` 之后加入：

```c
        v = dynamicCreate_Menu_Number(dbg, "yaw",       &atti_yaw, float_Box);
        v = dynamicCreate_Menu_Number(dbg, "ang",       &angle_pid_out, float_Box);
```

- [ ] **Step 4: 编译验证**

Rebuild，预期 0 error 0 warning。

---

### Task 9: 手动操作 — Keil 工程添加 attitude.c + 全编译验证

**Files:**
- Modify: Keil 工程文件 (手动操作)

- [ ] **Step 1: 在 Keil IDE 中添加 attitude.c**

1. 打开 Keil 工程 `smartcar/mdk/mm32f327x_g8p.uvprojx`
2. 在 Project 窗口的 `user/code/control` 组（或手动找到 control 组）右键 → "Add Existing Files to Group"
3. 选择 `smartcar/code/control/attitude.c`
4. 确认文件出现在工程列表中

- [ ] **Step 2: 全编译验证**

Keil → Project → Rebuild all target files

预期输出：
```
Build target 'MM32F327X_G8P'
...
0 Error(s), 0 Warning(s).
```

- [ ] **Step 3: 烧录验证**

1. 下载到小车
2. 上电 → 进入菜单 → Debug 文件夹
3. 确认 `yaw` 值有变化（左右旋转车身，yaw 应跟随变化）
4. 确认 `ang` 初始为 0（ang_kpa=0, fusion_a=0）
5. 确认 `fusion_a`、`ang_kpa`、`ang_kd` 出现在 PID 文件夹中

- [ ] **Step 4: Commit**

```bash
git add smartcar/code/control/attitude.h smartcar/code/control/attitude.c
git add smartcar/code/control/control.h smartcar/code/control/control.c
git add smartcar/code/menu/Mymenu.c
git add smartcar/user/inc/config.h smartcar/user/src/main.c smartcar/user/src/isr.c
git commit -m "feat: 添加姿态角融合（AHRS + Angle PID + Servo Fusion）

- 新建 attitude.c/h: Mahony互补滤波 + 非线性Angle PID + 线性融合
- TIM6 ISR 接管 MPU6050 100Hz 读取 + AHRS 更新
- servo_update() 末尾加融合: α×Angle_PID + (1-α)×视觉PD
- 菜单新增 fusion_a/ang_kpa/ang_kd (PID) + yaw/ang (Debug)
- α=0 起步，完全等价现有行为"
```

---

## 自检清单

- [x] Spec 全覆盖：AHRS ✅ Angle PID ✅ Fusion ✅ ISR ✅ 菜单 ✅ 参数默认值 ✅
- [x] 无占位符/TODO
- [x] 类型一致性：`atti_yaw` 全程 float，`angle_pid_out` 全程 float，`servo_fusion_alpha` 全程 float
- [x] 编译零警告零错误预期已标注
- [x] 每任务有独立编译验证步骤
