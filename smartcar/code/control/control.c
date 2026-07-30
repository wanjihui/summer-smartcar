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
int   motor_max_duty = DEFAULT_MOTOR_MAX;
float gyro_kd        = DEFAULT_GYRO_KD;
float gyro_kd_curve   = DEFAULT_GYRO_KD_CURVE;
float motor_bend_cut  = DEFAULT_MOTOR_BEND_CUT;
float motor_kp       = DEFAULT_MOTOR_KP;
float motor_kd       = DEFAULT_MOTOR_KD;
int   motor_diff_max = DEFAULT_MOTOR_DIFF_MAX;
bool  car_run        = false;    // 小车运行开关，菜单可切换

static float servo_last_err  = 0;       // 上一帧偏差，舵机D项用
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
    float e = err;                //err由vision算法计算
    if (e > -servo_dead && e < servo_dead) e = 0;
    if (servo_dir)             e = -e;

    // ----根据赛道形状切换参数----
    int straight = is_straight();   // 一次判断，多处复用
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

    // 陀螺仪阻尼：
    //   直道：gyro_kd × gyro_z，全量阻尼
    //   弯道：gyro_kd_curve × boost × gyro_z
    //         boost 只对偏航变号(摆动)放大，入弯单向加减速不误触发
    {
        static float gyro_prev = 0;
        float gyro_z = (float)mpu6050_get_gyro_z();

        if (straight)
        {
            pd += gyro_kd * gyro_z;
        }
        else
        {
            /* 变号检测：gyro_z 与上一帧异号 = 来回摆动 */
            float boost;
            if (gyro_z * gyro_prev < 0)
                boost = 1.5f;   // 摆动 → 强阻尼
            else
                boost = 0.15f;  // 稳态/入弯/出弯 → 轻阻尼，不拖转向

            gyro_prev = gyro_z;
            pd += gyro_kd_curve * boost * gyro_z;
        }
    }

    servo_last_err = e;

    // ----像素err转换为角度----
    float angle_rate = servo_max_cha / 50.0f;
    float angle_cha = pd * angle_rate;

    if (angle_cha >  servo_max_cha) angle_cha =  servo_max_cha;
    if (angle_cha < -servo_max_cha) angle_cha = -servo_max_cha;

    float target = servo_center + angle_cha;

    //----步进限制----
    float add = target - servo_last_angle;
    if (add >  servo_max_add) target = servo_last_angle + servo_max_add;
    if (add < -servo_max_add) target = servo_last_angle - servo_max_add;

    servo_set_angle(target);                       //舵机调整角度函数
    servo_last_angle = target;                     //存储当前目标角度
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

    // ----弯道减速 + 差速分配----
    // err>0偏右→右转: left=base+diff(快) right=base-diff(慢)
    // err<0偏左→左转: left=base-|diff|(慢) right=base+|diff|(快)
    float e_abs = (e > 0) ? e : -e;
    int bend_cut = (int)(e_abs * motor_bend_cut); // 弯越急降速越多
    int base = motor_base_duty - bend_cut;         // 弯道基础速度降低
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
    servo_update();         //先更新舵机
    motor_update();
}
