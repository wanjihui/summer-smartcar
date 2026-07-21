#include "config.h"

//PID参数定义
float servo_kp       = DEFAULT_SERVO_KP;
float servo_kd       = DEFAULT_SERVO_KD;
float servo_center   = DEFAULT_SERVO_CENTER;
float servo_max_cha  = DEFAULT_SERVO_MAX_CHA;
float servo_dead     = DEFAULT_SERVO_DEAD;
float servo_max_add  = DEFAULT_SERVO_MAX_ADD;
int   servo_dir      = DEFAULT_SERVO_DIR;
int   motor_base_duty= DEFAULT_MOTOR_BASE;
int   motor_max_duty = DEFAULT_MOTOR_MAX;
float motor_kp       = DEFAULT_MOTOR_KP;
float motor_kd       = DEFAULT_MOTOR_KD;

static pid_t pid_servo;                          //存舵机参数
static float servo_last_angle = 90.0f;           //上一帧角度记录,初始在center
static float motor_last_err = 0;                 // 上一帧电机err


void control_init(void)
{
    pid_init(&pid_servo, servo_kp, 0, servo_kd, 0,
             -servo_max_cha, servo_max_cha);//舵机pid输出限幅+_15度
    servo_last_angle = servo_center;//初始化上一帧的角度
}

//舵机控制
static void servo_update(void)
{
    // ----死区处理----
    float e = err;                //err由vision算法计算
    //即|e|<2 不调
    if (e > -servo_dead && e < servo_dead) e = 0;
    //servo_dir=1 翻转舵机方向
    if (servo_dir)              e = -e;

    // ----位置式PD计算----
    //kp×当前偏差,改变角度回正速度  kd×偏差变化,阻尼角度变化速度
    float pd = pid_servo.kp * e + pid_servo.kd * (e - pid_servo.last_err);
    pid_servo.last_err = e;   //存储当前err

    // ----像素err转换为角度----
    //角度差最大15° / 像素误差最大94px ≈ 0.16°/px
    // err 每偏离 1 像素，舵机偏离 0.16°
    float angle_rate = servo_max_cha / (float)pho_center_x;
    float angle_cha = pd * angle_rate;

    if (angle_cha >  servo_max_cha) angle_cha =  servo_max_cha;
    if (angle_cha < -servo_max_cha) angle_cha = -servo_max_cha;

    float target = servo_center + angle_cha;     //目标=中间+偏差

    //----步进限制----
    float add = target - servo_last_angle;        // 每帧预期改变度数
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
    float cha = e - motor_last_err;                  // 偏差变化

    float Cha = motor_kp * e + motor_kd * cha;
    motor_last_err = e;

    // ----差速分配----
    // err>0偏右 右转 左快右慢
    int left  = motor_base_duty + (int)Cha;
    int right = motor_base_duty - (int)Cha;

    // ----限幅----
    if (left  > motor_max_duty) left  = motor_max_duty;
    if (left  < 0)               left  = 0;
    if (right > motor_max_duty) right = motor_max_duty;
    if (right < 0)               right = 0;

    motor_set_both((int8_t)left, (int8_t)right); 
}

//控制总调用接口
void control_update(void)
{
    servo_update();         //先更新舵机
    motor_update();         
}


