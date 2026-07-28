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
float gyro_threshold  = DEFAULT_GYRO_THRESHOLD;
float motor_bend_cut  = DEFAULT_MOTOR_BEND_CUT;
float motor_kp       = DEFAULT_MOTOR_KP;
float motor_kd       = DEFAULT_MOTOR_KD;
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
    //即|e|<2 不调
    if (e > -servo_dead && e < servo_dead) e = 0;
    //servo_dir=1 翻转舵机方向
    if (servo_dir)              e = -e;

    // ----位置式PD ----
    // P = kp1 + kp2×|err|  弯道err大时自动增强
    // kd×偏差变化=阻尼
    float e_abs = (e > 0) ? e : -e;
    float pd = servo_kp1 * e + servo_kp2 * e * e_abs
             + servo_kd * (e - servo_last_err);

    // 直道陀螺仪阻尼：|err|<阈值时启用，陀螺仪直接感知车身旋转，比摄像头D项快20ms
    float e_abs_raw = (err > 0) ? err : -err;
    if (e_abs_raw < gyro_threshold)
        pd += gyro_kd * (float)mpu6050_get_gyro_z();

    servo_last_err = e;   //存储当前err

    // ----像素err转换为角度----
    // err物理上限≈50px（图像宽188，中心94，赛道最多偏离±50px）
    // max_cha/50: err=50px → 舵机打满，充分利用舵机量程
    // 旧公式 /94 导致err需94px才打满→舵机只用50%量程→弯道永远转不够
    float angle_rate = servo_max_cha / 50.0f;
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

    // 差速量：motor_kp低时(0.1~0.2)正常差速，高时急弯内轮可反转(pivot)
    int diff = (int)Cha;

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


