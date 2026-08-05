#include "config.h"
#include "vofa_uart.h"

//PID参数定义
float servo_kp1      = DEFAULT_SERVO_KP1;  // 基础P
float servo_kp2      = DEFAULT_SERVO_KP2;  // 二次P: err×|err|
float servo_kd       = DEFAULT_SERVO_KD;  // 统一D（已精简，不再区分直道/弯道）
float servo_center   = DEFAULT_SERVO_CENTER;
float servo_max_cha  = DEFAULT_SERVO_MAX_CHA;
float servo_dead     = DEFAULT_SERVO_DEAD;
int   servo_dir      = DEFAULT_SERVO_DIR;
int   motor_base_duty= DEFAULT_MOTOR_BASE;
int   motor_curve_duty = DEFAULT_MOTOR_CURVE_DUTY;  // 弯道占空比（|err|≥8时使用）
int   motor_max_duty = DEFAULT_MOTOR_MAX;
float gyro_kd        = DEFAULT_GYRO_KD;  // 陀螺仪阻尼（已精简，不再区分直道/弯道）
float motor_kp       = DEFAULT_MOTOR_KP;
float motor_kd       = DEFAULT_MOTOR_KD;
int   motor_diff_max = DEFAULT_MOTOR_DIFF_MAX;
float ackermann_gain = DEFAULT_ACKERMANN_GAIN;  // Ackermann差速增益
float motor_speed_kp_l = DEFAULT_SPEED_KP_L;
float motor_speed_ki_l = DEFAULT_SPEED_KI_L;
float motor_speed_kd_l = DEFAULT_SPEED_KD_L;
float motor_speed_kp_r = DEFAULT_SPEED_KP_R;
float motor_speed_ki_r = DEFAULT_SPEED_KI_R;
float motor_speed_kd_r = DEFAULT_SPEED_KD_R;
volatile CarState car_state   = CAR_IDLE;
volatile bool     car_cmd     = false;   /* 菜单代理: false=停车 true=发车 */
static uint8_t    launch_ramp_cnt = 0;   /* 起步斜坡计数器 */
float gyro_z_dbg     = 0.0f;
float steer_dbg      = 0.0f;

static float servo_last_err   = 0;       // 上一帧偏差，舵机D项用（50Hz servo_calc专用）
float motor_base_cms    = 20.0f;         // 速度目标缓存（50Hz motor_speed_calc写，200Hz motor_inject_ackermann读）


void control_init(void)
{
    servo_set_angle(servo_center);
    Speed_PID_Init();                    // 初始化速度环PID中间量
    KF_init(&Kf_L, KF_SPEED_Q, KF_SPEED_R);  // 初始化卡尔曼滤波器
    KF_init(&Kf_R, KF_SPEED_Q, KF_SPEED_R);
    /* 用菜单可调的 motor_max_duty 覆盖静态初始值 */
    Motor_L_PID.OutMax =  (float)motor_max_duty;
    Motor_L_PID.OutMin = -(float)motor_max_duty;
    Motor_R_PID.OutMax =  (float)motor_max_duty;
    Motor_R_PID.OutMin = -(float)motor_max_duty;
    car_state = CAR_IDLE;
    car_cmd   = false;
    launch_ramp_cnt = 0;
}

/* 发车：从菜单触发，重置斜坡 + PID，IDLE → LAUNCHING */
void control_state_launch(void)
{
    if (car_state == CAR_IDLE) {
        launch_ramp_cnt = 0;
        servo_last_err  = 0;   /* 防第一帧D爆冲 */
        uint32 primask = interrupt_global_disable();
        Speed_PID_Init();
        interrupt_global_enable(primask);
        car_state = CAR_LAUNCHING;
    }
}

/* 停车：从 vision 出界或菜单触发，→ STOP。
 * STOP 的清理（motor_stop + PID_Init + →IDLE）由 main.c 主循环执行 */
void control_state_stop(void)
{
    if (car_state >= CAR_LAUNCHING) {
        car_state = CAR_STOP;
        car_cmd   = false;   /* 同步菜单显示 */
    }
}

/* servo_calc: 50Hz 主循环调用，PPDD+Gyro 一次性算出 steer_dbg
 * gyro_z_dbg 由 ISR 200Hz 更新，此处始终取最新值 */
void servo_calc(void)
{
    float e = err;
    if (e > -servo_dead && e < servo_dead) e = 0;
    if (servo_dir) e = -e;

    float e_abs = (e > 0) ? e : -e;
    float delta = e - servo_last_err;

    /* 统一 PPDD + Gyro：四项一起算，不拆链 */
    float pd = servo_kp1 * e
             + servo_kp2 * e * e_abs
             + servo_kd  * delta
             - gyro_kd   * gyro_z_dbg;

    servo_last_err = e;

    float angle_cha = pd * (servo_max_cha / 35.0f);
    if (angle_cha >  servo_max_cha) angle_cha =  servo_max_cha;
    if (angle_cha < -servo_max_cha) angle_cha = -servo_max_cha;

    float target = servo_center + angle_cha;
    steer_dbg = angle_cha;
    servo_set_angle(target);
}


/* motor_speed_calc: 50Hz 主循环调用，依赖 err 计算基础速度 + 非对称LPF
 * 结果写入 motor_base_cms，供 ISR 的 motor_inject_ackermann 消费 */
void motor_speed_calc(void)
{
    float e = err;
    if (e > -servo_dead && e < servo_dead) e = 0;
    if (servo_dir) e = -e;

    float e_abs = (e > 0) ? e : -e;
    float ratio = 0.0f;
    if (e_abs > 8.0f) {
        ratio = (e_abs - 8.0f) / 10.0f;
        if (ratio > 1.0f) ratio = 1.0f;
    }
    float base_cms = ((float)motor_base_duty * (1.0f - ratio)
                    + (float)motor_curve_duty * ratio) * 10.0f;
    if (base_cms < 20.0f) base_cms = 20.0f;

    /* 起步斜坡：LAUNCHING 状态下 base_cms 从0线性爬升到目标值。
     * 25步 × 20ms(帧周期) = 500ms，完成后自动切 RUNNING */
    if (car_state == CAR_LAUNCHING) {
        if (launch_ramp_cnt < LAUNCH_RAMP_STEPS) {
            launch_ramp_cnt++;
            float factor = (float)launch_ramp_cnt / (float)LAUNCH_RAMP_STEPS;
            base_cms *= factor;
        } else {
            car_state = CAR_RUNNING;
        }
    }

    /* 非对称低通：入弯降速α=1.0(快)，出弯加速α=0.5(慢防振荡) */
    static float filtered = 0;
    if (filtered < 1.0f) filtered = base_cms;
    else {
        float alpha = (base_cms < filtered) ? 1.0f : 0.5f;
        filtered += alpha * (base_cms - filtered);
    }
    motor_base_cms = filtered;
}

/* motor_inject_ackermann: 200Hz TIM6 ISR 调用，用最新的 steer_dbg 和 motor_base_cms
 * 计算 Ackermann 乘法差速目标。前馈预载逻辑保留于此。 */
void motor_inject_ackermann(void)
{
    float steer_abs = (steer_dbg > 0.0f) ? steer_dbg : -steer_dbg;
    float diff_ratio = 0.0f;
    if (steer_abs > 4.0f) {
        diff_ratio = steer_dbg * 0.017453293f * 0.05475f / 0.20f * ackermann_gain;
    }

    float new_tgt_l = motor_base_cms * (1.0f + diff_ratio);
    float new_tgt_r = motor_base_cms * (1.0f - diff_ratio);

    /* 前馈预载：目标跳变 >50% 时复位 PID + 预载估算 duty */
    static float prev_l = 0, prev_r = 0;
    if (prev_l > 10.0f
        && (new_tgt_l > prev_l * 1.5f || new_tgt_l < prev_l * 0.5f)) {
        PID_INC_Init(&Motor_L_PID);
        PID_INC_Init(&Motor_R_PID);
        Motor_L_PID.Out = new_tgt_l * 0.125f;
        Motor_R_PID.Out = new_tgt_r * 0.125f;
    }
    prev_l = new_tgt_l; prev_r = new_tgt_r;

    Motor_L_PID.Target = new_tgt_l;
    Motor_R_PID.Target = new_tgt_r;
}

/* PID参数同步：仅在菜单修改参数时调用（替代每帧同步） */
void control_sync_params(void)
{
    Motor_L_PID.Kp = motor_speed_kp_l; Motor_R_PID.Kp = motor_speed_kp_r;
    Motor_L_PID.Ki = motor_speed_ki_l; Motor_R_PID.Ki = motor_speed_ki_r;
    Motor_L_PID.Kd = motor_speed_kd_l; Motor_R_PID.Kd = motor_speed_kd_r;
}

// VOFA Firewater: 无线串口发送速度数据（非阻塞，TX空中断发送）
void control_vofa_send(void)
{
    /* Firewater: Target(cm/s),Actual(cm/s),Out(duty),err,steer */
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f\r\n",
        Motor_L_PID.Target, Motor_L_PID.Actual, Motor_L_PID.Out,
        Motor_R_PID.Target, Motor_R_PID.Actual, Motor_R_PID.Out,
        (double)err, (double)steer_dbg, (double)gyro_z_dbg);
    if (len > 0 && len < (int)sizeof(buf))
        vofa_uart_send((uint8 *)buf, (uint32)len);
}

/* 控制总调用接口（拆链方案：err驱动的计算在50Hz主循环，gyro/Ackermann在200Hz ISR）
 * servo_calc + motor_speed_calc 依赖 err，仅在摄像头出新帧时计算一次 */
void control_update(void)
{
    servo_calc();            /* err驱动: P+P²+D → steer_dbg（50Hz）*/
    motor_speed_calc();      /* err驱动: 速度过渡+LPF → motor_base_cms（50Hz）*/
    control_sync_params();
    Speed_PID_Enable = 1;
}


