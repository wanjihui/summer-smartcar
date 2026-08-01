# 姿态角融合 — 设计文档

**日期**: 2026-08-01  
**目标**: 引入 AHRS 互补滤波 + Angle PID + Servo Fusion，在直道抑制蛇形走线  
**方案**: A（加法叠加层），AHRS 运行在 TIM6 PIT（100Hz）

---

## 1. 架构概览

```
TIM6 ISR (10ms, 100Hz):                主循环 (每帧, ~50-100fps):
  mpu6050_get_acc() ──┐                  camera帧 → vis_deal() → err
  mpu6050_get_gyro() ─┤                                ↓
  atti_update() ──────→ atti_yaw        control_update():
                              ↓            servo_update():
                              ↓              PD(err) - gyro_kd×gyro_z → raw_angle
                              ↓                     +
                              ↓              angle_pid(yawΔ) → α×补偿
                              ↓                     ↓
                              └──────────→  fusion(raw, angle_comp) → 舵机
```

**关键变化**:
- MPU6050 读取从主循环移到 TIM6 ISR（100Hz 固定周期，保证 AHRS 积分精度）
- 主循环 和 ISR 共享 `mpu6050_gyro_z`（int16 原子读），不会数据撕裂
- 新增 3 个菜单可调参数 + 2 个 Debug 只读值

---

## 2. 文件清单

| 操作 | 文件 | 说明 |
|------|------|------|
| **新建** | `code/control/attitude.h` | 外部接口声明 |
| **新建** | `code/control/attitude.c` | AHRS + Angle PID + Fusion 实现 |
| **修改** | `user/src/isr.c` | TIM6 ISR: 加 mpu6050_get_acc/gyro + atti_update() |
| **修改** | `code/control/control.c` | servo_update() 末尾加融合调用 |
| **修改** | `code/control/control.h` | 暴露 atti_yaw, angle_pid_out, 融合参数 |
| **修改** | `user/inc/config.h` | 加默认参数宏 |
| **修改** | `code/menu/Mymenu.c` | 菜单新增 PID/角度PID 子项 + Debug 项 |
| **修改** | `user/src/main.c` | 删主循环 mpu6050_update(), 加 atti_init() |
| **手动** | Keil 工程 | 添加 attitude.c 到编译列表 |

---

## 3. 模块设计

### 3.1 attitude.c — 核心模块

#### AHRS (Mahony 互补滤波)

```c
// 参数（编译期常量，不可调）
#define ATTI_KP         0.0001f    // 加速度修正比例增益
#define ATTI_KI         0.0f       // 积分增益（先关闭，原版负值疑似笔误）
#define ATTI_DT         0.01f      // 采样周期 10ms (=TIM6_PIT)
#define ATTI_ACC_ALPHA  0.3f       // 加速度低通滤波系数

// 内部状态
static float atti_q0..q3;          // 四元数
static float atti_I_ex, ey, ez;    // 积分项

// 输出
float atti_yaw;                    // 偏航角 (°), 左负右正

// 接口
void atti_init(void);              // 四元数复位
void atti_update(void);            // 每10ms调用, 读取mpu6050全局变量, 更新atti_yaw
```

**实现要点**:
- 直接读取 `mpu6050_acc_x/y/z` 和 `mpu6050_gyro_x/y/z` 全局变量（ISR上下文，不调IIC读函数）
- 使用 `mpu6050_acc_transition()` 和 `mpu6050_gyro_transition()` 转换为物理单位
- 陀螺仪数据转 rad/s: `× π / 180`
- 保留 `fast_inv_sqrt()` 但改用标准 `1.0f/sqrtf()`（MM32有硬件FPU）
- ATTI_KI=0 先关闭积分（避免原版负值问题）

#### Angle PID

```c
// 可调参数（菜单修改）
float angle_kp_a = 0.0f;     // 基础P
float angle_kp_b = 0.0f;     // 二次P系数
float angle_kd  = 0.02f;     // D项
float angle_lowpass = 0.8f;  // D项低通

// 诊断输出
float angle_pid_out = 0.0f;  // Angle PID 输出值

// 接口
float angle_pid_set(float target, float actual);
// 调用方式: angle_pid_set(prev_yaw, atti_yaw)
// target=上一帧yaw（期望不变）, actual=当前yaw
// 返回: 舵机补偿量 (°), 范围 ±12°
```

**实现要点**:
- 非线性增益: `kp = kp_a + error² × kp_b`（大偏航时强拉回）
- 视觉置信度自适应: `if (|img_err| < 12) kp = kp_a`（视觉跟踪好时压低Angle介入）
- D项一阶低通滤波抑制噪声
- 输出限幅 ±12°

#### Servo Fusion

```c
float servo_fusion_alpha = 0.0f;  // 融合系数, 0=纯视觉

float servo_fusion(float angle_out, float IMU_out);
// IMU_out = servo_update()的PD输出 (即现有控制量)
// angle_out = angle_pid_set()的输出
// 返回: (1-α)×IMU_out + α×angle_out, 限幅±12°
```

### 3.2 ISR 修改

```c
// TIM6_IRQHandler (user/src/isr.c)
void TIM6_IRQHandler(void)
{
    encoder_pit_callback();
    
    // ★新增: MPU6050读取 + AHRS更新
    mpu6050_get_acc();
    mpu6050_get_gyro();
    atti_update();
    
    TIM6->SR &= ~TIM6->SR;
}
```

**设计决策**: 在 ISR 里直接调 `mpu6050_get_acc/gyro`（软IIC）。10ms周期里软IIC约200-300µs，占比 ~3%，可接受。这是原版TC264方案的做法。

### 3.3 control.c 修改

`servo_update()` 末尾替换为:

```c
// ---- 融合前保存原始PD输出 ----
float raw_angle = servo_center + angle_cha;  // 等价于原来的 target

// ---- Angle PID ----
static float prev_yaw = 0.0f;
float angle_out = angle_pid_set(prev_yaw, atti_yaw);
prev_yaw = atti_yaw;

// ---- 融合 ----
float fused_angle = servo_fusion(angle_out, raw_angle);

// 后续步进限制、限幅、输出等用 fused_angle 替代原来的 target
```

### 3.4 main.c 修改

```c
// 初始化部分: mpu6050_module_init() 之后加
atti_init();

// 主循环: 删除 mpu6050_update() (移到ISR)
if(car_run)
{
    // mpu6050_update();  ← 删除这行
    control_update();
}
```

---

## 4. 参数默认值

```c
// config.h 新增
#define DEFAULT_ANGLE_KP_A          0.0f
#define DEFAULT_ANGLE_KP_B          0.0f
#define DEFAULT_ANGLE_KD            0.02f
#define DEFAULT_ANGLE_LOWPASS       0.8f
#define DEFAULT_SERVO_FUSION_ALPHA  0.0f
```

α=0 起步，完全等价现有行为。调参时逐步加大。

---

## 5. 菜单新增项

### PID 文件夹新增

| 菜单名 | 变量 | 类型 | 范围 | 步进 |
|--------|------|------|------|------|
| `fusion_a` | `servo_fusion_alpha` | float | 0.0~1.0 | 0.05 |
| `ang_kpa` | `angle_kp_a` | float | 0.0~5.0 | 0.01 |
| `ang_kd` | `angle_kd` | float | 0.0~2.0 | 0.01 |

### Debug 文件夹新增

| 菜单名 | 变量 | 类型 | 说明 |
|--------|------|------|------|
| `yaw` | `atti_yaw` | float | 当前偏航角 |
| `ang` | `angle_pid_out` | float | Angle PID输出 |

---

## 6. 调参指南

1. Debug→yaw 确认有值（AHRS正常）
2. `fusion_a` = 0.10, `ang_kpa` = 0.5
3. 直道跑，观察 Debug→ang 波动
4. 蛇形没改善 → +ang_kpa；震荡 → +ang_kd
5. 有效果后 +fusion_a 到 0.2~0.3
6. 弯道如果车转不动 → -fusion_a 或 -ang_kpa

---

## 7. 坐标系约定

MPU6050 安装方向需确认。假设:
- X轴 = 车前进方向
- Y轴 = 车左侧
- Z轴 = 垂直向上

Yaw = 绕Z轴旋转，左转为负，右转为正。
与实际安装方向不一致时需交换/取反坐标轴。
