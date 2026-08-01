#include "config.h"

//PID参数定义
float servo_kp1      = DEFAULT_SERVO_KP1;  // 基础P
float servo_kp2      = DEFAULT_SERVO_KP2;  // 二次P: err×|err|
float servo_kd       = DEFAULT_SERVO_KD;
float servo_center   = DEFAULT_SERVO_CENTER;
float servo_max_cha  = DEFAULT_SERVO_MAX_CHA;
float servo_dead     = DEFAULT_SERVO_DEAD;
float servo_max_add  = DEFAULT_SERVO_MAX_ADD;
int   servo_dir      = DEFAULT_SERVO_DIR;
int   motor_base_duty= DEFAULT_MOTOR_BASE;
int   motor_curve_duty = DEFAULT_MOTOR_CURVE_DUTY;  // 弯道占空比（is_straight判定为弯道时使用）
int   motor_max_duty = DEFAULT_MOTOR_MAX;
float gyro_kd        = DEFAULT_GYRO_KD;
float gyro_kd_curve   = DEFAULT_GYRO_KD_CURVE;
float motor_kp       = DEFAULT_MOTOR_KP;
float motor_kd       = DEFAULT_MOTOR_KD;
int   motor_diff_max = DEFAULT_MOTOR_DIFF_MAX;
bool  car_run        = false;    // 小车运行开关，菜单可切换

static float servo_last_err   = 0;       // 上一帧偏差，舵机D项用
static float servo_last_angle = 0;      // 上一帧舵机角度，步进限制用
static float motor_last_err   = 0;      // 上一帧偏差，电机D项用


void control_init(void)
{
    servo_last_angle = servo_center;     // 初始化舵机角度记录
    servo_set_angle(servo_center);       // 立即输出PWM，舵机进入工作状态
}

//舵机控制
static void servo_update(int straight)
{
    // ----死区处理----
    float e = err;                //err由vision算法计算
    if (e > -servo_dead && e < servo_dead) e = 0;
    if (servo_dir)             e = -e;

    // ----根据赛道形状切换参数（straight由control_update统一判定）----
    float kp, kd;
    if (straight)
    {
        /* 直道：小 P 稳速 + 大 D 抑抖 */
        kp = servo_kp1;
        kd = servo_kd * 1.5f;
    }
    else
    {
        /* 弯道：大 P 快响应 + 小 D 不拖转向 */
        float e_abs = (e > 0) ? e : -e;
        kp = servo_kp1 + servo_kp2 * e_abs;   // kp2×|e| 随偏差增大
        kd = servo_kd;
    }

    // ----位置式 PD ----
    float pd = kp * e + kd * (e - servo_last_err);

    // 陀螺仪前馈（ASC风格）：pd -= gyro × gyro_z
    //   "车已经在转了，视觉少打点"
    //   量化去噪 /20*20，系数可以安全开到 0.01-0.03 不抖
    {
        int gz = (int)mpu6050_gyro_transition(mpu6050_get_gyro_z() - mpu6050_gyro_z_offset);
        gz = (gz / 20) * 20;                       // ASC风格量化，±10°/s以下不触发
        float gyro_z = (float)gz;

        if (straight)
            pd -= gyro_kd * gyro_z;
        else
            pd -= gyro_kd_curve * gyro_z;
    }

    servo_last_err = e;

    // ----像素err转换为角度----
    float angle_rate = servo_max_cha / 35.0f;  // 映射斜率: max_cha/35, 入弯小err也有足够转角
    float angle_cha = pd * angle_rate;

    if (angle_cha >  servo_max_cha) angle_cha =  servo_max_cha;
    if (angle_cha < -servo_max_cha) angle_cha = -servo_max_cha;

    //---- Angle PID（偏航角稳定）----
    static float prev_yaw = 0.0f;
    float angle_out = angle_pid_set(prev_yaw, atti_yaw);
    prev_yaw = atti_yaw;

    //---- 舵机融合（两个偏差量融合，再映射到绝对角度）----
    float fused_cha = servo_fusion(angle_out, angle_cha);
    float target = servo_center + fused_cha;

    //----步进限制----
    float add = target - servo_last_angle;
    if (add >  servo_max_add) target = servo_last_angle + servo_max_add;
    if (add < -servo_max_add) target = servo_last_angle - servo_max_add;

    servo_set_angle(target);                       //舵机调整角度函数
    servo_last_angle = target;                     //存储当前目标角度
}


//电机控制
static void motor_update(int straight)
{
    // ----PD算出差速----
    float e=err;                                     // 偏差
    if (e > -servo_dead && e < servo_dead) e = 0;    // 死区（与舵机一致）
    if (servo_dir) e = -e;                           // 方向翻转（与舵机一致）
    float cha = e - motor_last_err;                  // 偏差变化

    float Cha = motor_kp * e + motor_kd * cha;
    motor_last_err = e;

    // ----直道/弯道基础占空比分开控制----
    // err>0偏右→右转: left=base+diff(快) right=base-diff(慢)
    // err<0偏左→左转: left=base-|diff|(慢) right=base+|diff|(快)
    // 直道: motor_base_duty（全速）
    // 弯道: motor_curve_duty（菜单可调，一般低于直道）
    int base;
    if (straight)
        base = motor_base_duty;
    else
        base = motor_curve_duty;
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
    static bool was_running = false;
    if (!was_running && car_run)
    {
        // 停车后重新出发：同步偏航基准，避免 prev_yaw 跳变
        atti_yaw_reset_ref();
    }
    was_running = car_run;

    int straight = is_straight();   // 一帧只判定一次，舵机/电机共用同一结果
    servo_update(straight);         //先更新舵机
    motor_update(straight);
}


