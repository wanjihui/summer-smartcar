#include "config.h"
#include "vofa_uart.h"

//PID参数定义
float servo_kp1      = DEFAULT_SERVO_KP1;  // 基础P
float servo_kp2      = DEFAULT_SERVO_KP2;  // 二次P: err×|err|
float servo_kd       = DEFAULT_SERVO_KD;  // 统一D（已精简，不再区分直道/弯道）
float servo_center   = DEFAULT_SERVO_CENTER;
float servo_max_cha  = DEFAULT_SERVO_MAX_CHA;
float servo_dead     = DEFAULT_SERVO_DEAD;
float servo_max_add  = DEFAULT_SERVO_MAX_ADD;
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
volatile bool  car_run        = false;
float gyro_z_dbg     = 0.0f;
float steer_dbg      = 0.0f;

static float servo_last_err   = 0;       // 上一帧偏差，舵机D项用
static float servo_last_angle = 0;      // 上一帧舵机角度，步进限制用


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

//舵机控制 — 统一 PPDD + Gyro 公式（阶段一精简，与参考项目 STEER_CTRL 同构）
static void servo_update(void)
{
    // ----死区处理----
    float e = err;
    if (e > -servo_dead && e < servo_dead) e = 0;
    if (servo_dir) e = -e;

    float e_abs = (e > 0) ? e : -e;
    float delta = e - servo_last_err;

    /* 统一 PPDD + Gyro：Out = P + P² + D - Gyro
     *   servo_kp1 × e          → 线性比例
     *   servo_kp2 × e × |e|    → 非线性P²（大弯更强，自动替代回正软化）
     *   servo_kd  × Δe         → D抑制振荡
     *   gyro_kd   × gyro_z     → 陀螺仪前馈阻尼                    */
    float pd = servo_kp1 * e
             + servo_kp2 * e * e_abs
             + servo_kd  * delta
             - gyro_kd   * gyro_z_dbg;

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


//电机控制 — 速度闭环
static void motor_update(void)
{
    /* 同步菜单参数到PID结构体（左右独立）*/
    Motor_L_PID.Kp = motor_speed_kp_l; Motor_R_PID.Kp = motor_speed_kp_r;
    Motor_L_PID.Ki = motor_speed_ki_l; Motor_R_PID.Ki = motor_speed_ki_r;
    Motor_L_PID.Kd = motor_speed_kd_l; Motor_R_PID.Kd = motor_speed_kd_r;

    float e = err;
    if (e > -servo_dead && e < servo_dead) e = 0;
    if (servo_dir) e = -e;

    #define DMS_TO_CMS 10.0f

    /* 连续速度过渡：|e| 8→18，基础速度从直道平滑过渡到弯道 */
    float e_abs = (e > 0) ? e : -e;
    float ratio = 0.0f;
    if (e_abs > 8.0f)
    {
        ratio = (e_abs - 8.0f) / 10.0f;       /* 8→18, ratio 0→1 */
        if (ratio > 1.0f) ratio = 1.0f;
    }
    float base_spd_dms = (float)motor_base_duty * (1.0f - ratio)
                       + (float)motor_curve_duty * ratio;
    float base_cms = base_spd_dms * DMS_TO_CMS;
    if (base_cms < 20.0f) base_cms = 20.0f;

    /* 非对称低通：入弯降速α=0.8(快)，出弯加速α=0.2(慢防振荡)*/
    static float base_cms_filtered = 0;
    if (base_cms_filtered < 1.0f) base_cms_filtered = base_cms;
    else
    {
        float alpha = (base_cms < base_cms_filtered) ? 1.0f : 0.5f;
        base_cms_filtered += alpha * (base_cms - base_cms_filtered);
    }
    base_cms = base_cms_filtered;

    /* === Ackermann差速：舵角→差速比（乘法，速度自适应）=== */
    #define ACKERMANN_WHEELBASE  0.20f       // 轴距 L (m)
    #define ACKERMANN_TRACK      0.05475f    // 后轮轮距 W (m)
    #define ACKERMANN_DEADZONE   4.0f        // 死区 (°)
    #define DEG2RAD              0.017453293f // PI/180

    float steer_abs = (steer_dbg > 0.0f) ? steer_dbg : -steer_dbg;
    float diff_ratio = 0.0f;
    if (steer_abs > ACKERMANN_DEADZONE)
    {
        float ang_rad = steer_dbg * DEG2RAD;  // tan(θ)≈θ, 16°误差~3%
        diff_ratio = ang_rad * ACKERMANN_TRACK / ACKERMANN_WHEELBASE * ackermann_gain;
    }

    /* 前馈: 25duty≈200cm/s → 0.125 duty/(cm/s) */
    #define FF_GAIN 0.125f

    /* 目标速度：乘法差速（差速量随速度自动缩放）*/
    static float prev_target_l = 0, prev_target_r = 0;
    float new_tgt_l = base_cms * (1.0f + diff_ratio);
    float new_tgt_r = base_cms * (1.0f - diff_ratio);
    if (prev_target_l > 10.0f
        && (new_tgt_l > prev_target_l * 1.5f || new_tgt_l < prev_target_l * 0.5f))
    {
        PID_INC_Init(&Motor_L_PID);
        PID_INC_Init(&Motor_R_PID);
        Motor_L_PID.Out = new_tgt_l * FF_GAIN;  /* 预载估算duty */
        Motor_R_PID.Out = new_tgt_r * FF_GAIN;
    }
    prev_target_l = new_tgt_l; prev_target_r = new_tgt_r;

    Motor_L_PID.Target = new_tgt_l;
    Motor_R_PID.Target = new_tgt_r;
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

//控制总调用接口
void control_update(void)
{
    servo_update();
    motor_update();
    Speed_PID_Enable = 1;   /* 速度环使能（car_run=false时由main.c关闭）*/
}


