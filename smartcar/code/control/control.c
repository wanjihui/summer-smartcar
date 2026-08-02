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
bool  car_run        = false;    // 小车运行开关，菜单可切换
float gyro_z_dbg     = 0.0f;     // 诊断：量化后 gyro_z
float steer_dbg      = 0.0f;     // 诊断：舵机偏离中心角度

static float servo_last_err   = 0;       // 上一帧偏差，舵机D项用
static float servo_last_angle = 0;      // 上一帧舵机角度，步进限制用
static float motor_last_err   = 0;      // 上一帧偏差，电机D项用


void control_init(void)
{
    servo_last_angle = servo_center;     // 初始化舵机角度记录
    servo_set_angle(servo_center);       // 立即输出PWM，舵机进入工作状态
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


//电机控制
static void motor_update(void)
{
    // ----PD算出差速----
    float e=err;                                     // 偏差
    if (e > -servo_dead && e < servo_dead) e = 0;    // 死区（与舵机一致）
    if (servo_dir) e = -e;                           // 方向翻转（与舵机一致）
    float cha = e - motor_last_err;                  // 偏差变化

    float Cha = motor_kp * e + motor_kd * cha;
    motor_last_err = e;

    // ----直道/弯道基础占空比: |err|<8→直道高速, else→弯道减速----
    float e_abs = (e > 0) ? e : -e;
    int base = (e_abs < 8.0f) ? motor_base_duty : motor_curve_duty;
    if (base < 5) base = 5;                         // 死区12已是最低可用duty，base保底降到5

    // 差速量：限幅防漂
    int diff = (int)Cha;
    if (diff >  motor_diff_max) diff =  motor_diff_max;
    if (diff < -motor_diff_max) diff = -motor_diff_max;

    int left  = base + diff;
    int right = base - diff;

    // ----死区+限幅----
    // 电机±1~11%扭矩不足，直接跳到±12；0保持0（停车）
    if (left  > 0 && left  < 12)  left  = 12;
    if (left  < 0 && left  > -12) left  = -12;
    if (right > 0 && right < 12)  right = 12;
    if (right < 0 && right > -12) right = -12;

    if (left  > motor_max_duty)  left  = motor_max_duty;
    if (left  < -motor_max_duty)  left  = -motor_max_duty;
    if (right > motor_max_duty)  right = motor_max_duty;
    if (right < -motor_max_duty)  right = -motor_max_duty;

    motor_set_both((int8_t)left, (int8_t)right); 
}

//控制总调用接口
void control_update(void)
{
    //---- |err|连续过渡: kp自动分离, kd/gyro平滑, motor速度跟随 ----
    servo_update();
    motor_update();
}


