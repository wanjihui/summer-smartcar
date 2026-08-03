#include "config.h"

//PID参数定义
float servo_kp1      = DEFAULT_SERVO_KP1;  // 基础P
float servo_kp2      = DEFAULT_SERVO_KP2;  // 二次P: err×|err|
float servo_kd       = DEFAULT_SERVO_KD;
float servo_kd_str    = DEFAULT_SERVO_KD_STR;  // 直道/小err D
float servo_center   = DEFAULT_SERVO_CENTER;
float servo_max_cha  = DEFAULT_SERVO_MAX_CHA;
float servo_dead     = DEFAULT_SERVO_DEAD;
float servo_max_add  = DEFAULT_SERVO_MAX_ADD;
int   servo_dir      = DEFAULT_SERVO_DIR;
int   motor_base_duty= DEFAULT_MOTOR_BASE;
int   motor_curve_duty = DEFAULT_MOTOR_CURVE_DUTY;  // 弯道占空比（|err|≥8时使用）
int   motor_max_duty = DEFAULT_MOTOR_MAX;
float gyro_kd        = DEFAULT_GYRO_KD;
float gyro_kd_curve   = DEFAULT_GYRO_KD_CURVE;
float motor_kp       = DEFAULT_MOTOR_KP;
float motor_kd       = DEFAULT_MOTOR_KD;
int   motor_diff_max = DEFAULT_MOTOR_DIFF_MAX;
float motor_speed_kp = DEFAULT_SPEED_KP;
float motor_speed_ki = DEFAULT_SPEED_KI;
float motor_speed_kd = DEFAULT_SPEED_KD;
volatile bool  car_run        = false;
float gyro_z_dbg     = 0.0f;
float steer_dbg      = 0.0f;

static float servo_last_err   = 0;       // 上一帧偏差，舵机D项用
static float servo_last_angle = 0;      // 上一帧舵机角度，步进限制用
static float motor_last_err   = 0;      // 上一帧偏差，电机D项用


void control_init(void)
{
    servo_last_angle = servo_center;
    servo_set_angle(servo_center);
    Speed_PID_Init();                    // 初始化速度环PID中间量
    KF_init(&Kf_L, KF_SPEED_Q, KF_SPEED_R);  // 初始化卡尔曼滤波器
    KF_init(&Kf_R, KF_SPEED_Q, KF_SPEED_R);
    /* 用菜单可调的 motor_max_duty 覆盖静态初始值 */
    Motor_L_PID.OutMax =  (float)motor_max_duty;
    Motor_L_PID.OutMin = -(float)motor_max_duty;
    Motor_R_PID.OutMax =  (float)motor_max_duty;
    Motor_R_PID.OutMin = -(float)motor_max_duty;
}

//舵机控制
static void servo_update(void)
{
    // ----死区处理----
    float e = err;
    if (e > -servo_dead && e < servo_dead) e = 0;
    if (servo_dir) e = -e;

    float e_abs = (e > 0) ? e : -e;
    float delta = e - servo_last_err;
    float d_abs = (delta > 0) ? delta : -delta;

    // ---- kp: 正常=kp1+kp2×|err|, 回正(err缩小)时软化防过冲 ----
    float kp = servo_kp1 + servo_kp2 * e_abs;

    // 回正软化: e×Δerr<0 → 车在靠近中线, |err|大+回正快 → 软P防冲过头
    if (e * delta < 0.0f && e_abs > 5.0f)
    {
        float r_err = (e_abs - 5.0f) / 10.0f;   // |err| 5→15, 软化0→1
        if (r_err > 1.0f) r_err = 1.0f;
        float r_spd = d_abs / 10.0f;             // |Δerr| 0→10, 因子0→1
        if (r_spd > 1.0f) r_spd = 1.0f;
        float soft = r_err * r_spd;               // 双因子: 大err+快速回正=强力软化
        float kp_soft = servo_kp1 * 0.6f;
        kp = kp + (kp_soft - kp) * soft;
    }

    // ---- kd: |err| 5→15 从直道D平滑过渡到弯道D ----
    float e_clamp;
    if (e_abs < 5.0f)      e_clamp = 0.0f;
    else if (e_abs > 15.0f) e_clamp = 1.0f;
    else                    e_clamp = (e_abs - 5.0f) / 10.0f;
    float kd = servo_kd_str + (servo_kd - servo_kd_str) * e_clamp;

    // ----位置式 PD ----
    float pd = kp * e + kd * delta;

    servo_last_err = e;

    // ----像素err转换为角度----
    float angle_cha = pd * (servo_max_cha / 35.0f);

    if (angle_cha >  servo_max_cha) angle_cha =  servo_max_cha;
    if (angle_cha < -servo_max_cha) angle_cha = -servo_max_cha;

    float target = servo_center + angle_cha;

    //----步进限制----
    float add = target - servo_last_angle;
    if (add >  servo_max_add) target = servo_last_angle + servo_max_add;
    if (add < -servo_max_add) target = servo_last_angle - servo_max_add;

    steer_dbg = target - servo_center;
    servo_set_angle(target);
    servo_last_angle = target;
}


//电机控制 — 速度闭环（PID.Target = 目标速度(编码器脉冲/10ms)）
static void motor_update(void)
{
    /* 同步菜单参数到PID结构体 */
    Motor_L_PID.Kp = motor_speed_kp; Motor_R_PID.Kp = motor_speed_kp;
    Motor_L_PID.Ki = motor_speed_ki; Motor_R_PID.Ki = motor_speed_ki;
    Motor_L_PID.Kd = motor_speed_kd; Motor_R_PID.Kd = motor_speed_kd;

    float e = err;
    if (e > -servo_dead && e < servo_dead) e = 0;
    if (servo_dir) e = -e;
    float cha = e - motor_last_err;
    float Cha = motor_kp * e + motor_kd * cha;
    motor_last_err = e;

    /* 目标速度: 菜单值(dm/s) → cm/s → PID.Target(cm/s)（对齐P7做法）*/
    #define ENC_TO_CMS 1.117f    /* 1脉冲/5ms → cm/s (PPR=4096=1024线×4x) */
    #define DMS_TO_CMS 10.0f     /* 1 dm/s = 10 cm/s */

    static bool is_curve = false;
    float e_abs = (e > 0) ? e : -e;
    if (!is_curve && e_abs > 12.0f)      is_curve = true;
    else if (is_curve && e_abs < 6.0f)   is_curve = false;
    float base_cms = (is_curve
        ? (float)motor_curve_duty : (float)motor_base_duty) * DMS_TO_CMS;
    if (base_cms < 20.0f) base_cms = 20.0f;

    /* 差速量（cm/s）*/
    int diff = (int)(Cha * ENC_TO_CMS);
    if (diff >  motor_diff_max) diff =  motor_diff_max;
    if (diff < -motor_diff_max) diff = -motor_diff_max;

    /* PID Target 和 Actual 统一用 cm/s（对齐P7）*/
    Motor_L_PID.Target = base_cms + (float)diff;
    Motor_R_PID.Target = base_cms - (float)diff;
}

// VOFA Firewater: 无线串口发送速度数据（每N帧调用一次，供上位机调参）
void control_vofa_send(void)
{
    /* Firewater: Target(cm/s),Actual(cm/s),Out(duty),... */
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f\r\n",
        Motor_L_PID.Target, Motor_L_PID.Actual, Motor_L_PID.Out,
        Motor_R_PID.Target, Motor_R_PID.Actual, Motor_R_PID.Out,
        (double)err);
    if (len > 0 && len < (int)sizeof(buf))
        wireless_uart_send_buffer((uint8 *)buf, (uint32)len);
}

//控制总调用接口
void control_update(void)
{
    servo_update();
    motor_update();
    Speed_PID_Enable = 1;   /* 速度环使能（car_run=false时由main.c关闭）*/
}


