/*********************************************************************************************************************
* 文件名称          motor
* 适用平台          MM32F327X_G8P
* 描述              DRV8701E 双电机驱动模块实现
*********************************************************************************************************************/

#include "motor.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机初始化
// 功能描述     初始化双路电机的 DIR 引脚（GPIO 推挽输出）和 PWM 引脚（TIM5）
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    // 左电机：DIR 初始化 默认高电平
    gpio_init(MOTOR_DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR_PWM_L, MOTOR_PWM_FREQ, 0);

    // 右电机：DIR 初始化 默认高电平
    gpio_init(MOTOR_DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR_PWM_R, MOTOR_PWM_FREQ, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置左电机
// 参数说明     duty  占空比 -MOTOR_MAX_DUTY ~ +MOTOR_MAX_DUTY
//                     正值为正转 负值为反转 0 为停止
//-------------------------------------------------------------------------------------------------------------------
void motor_set_left(int8_t duty)
{
    // 限幅
    if(duty > MOTOR_MAX_DUTY)   duty = MOTOR_MAX_DUTY;
    if(duty < -MOTOR_MAX_DUTY)  duty = -MOTOR_MAX_DUTY;

    if(duty >= 0)
    {
        gpio_set_level(MOTOR_DIR_L, GPIO_HIGH);                                 // DIR 输出高电平 正转
        pwm_set_duty(MOTOR_PWM_L, ((uint32_t)duty) * (PWM_DUTY_MAX / 100));    // 计算占空比
    }
    else
    {
        gpio_set_level(MOTOR_DIR_L, GPIO_LOW);                                  // DIR 输出低电平 反转
        pwm_set_duty(MOTOR_PWM_L, ((uint32_t)(-duty)) * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置右电机
// 参数说明     duty  占空比 -MOTOR_MAX_DUTY ~ +MOTOR_MAX_DUTY
//                     正值为正转 负值为反转 0 为停止
//-------------------------------------------------------------------------------------------------------------------
void motor_set_right(int8_t duty)
{
    // 限幅
    if(duty > MOTOR_MAX_DUTY)   duty = MOTOR_MAX_DUTY;
    if(duty < -MOTOR_MAX_DUTY)  duty = -MOTOR_MAX_DUTY;

    if(duty >= 0)
    {
        gpio_set_level(MOTOR_DIR_R, GPIO_HIGH);                                 // DIR 输出高电平 正转
        pwm_set_duty(MOTOR_PWM_R, ((uint32_t)duty) * (PWM_DUTY_MAX / 100));    // 计算占空比
    }
    else
    {
        gpio_set_level(MOTOR_DIR_R, GPIO_LOW);                                  // DIR 输出低电平 反转
        pwm_set_duty(MOTOR_PWM_R, ((uint32_t)(-duty)) * (PWM_DUTY_MAX / 100)); // 计算占空比
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同时设置两个电机
// 参数说明     left_duty   左电机占空比 -MOTOR_MAX_DUTY ~ +MOTOR_MAX_DUTY
//              right_duty  右电机占空比 -MOTOR_MAX_DUTY ~ +MOTOR_MAX_DUTY
//-------------------------------------------------------------------------------------------------------------------
void motor_set_both(int8_t left_duty, int8_t right_duty)
{
    motor_set_left(left_duty);
    motor_set_right(right_duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止两个电机
//-------------------------------------------------------------------------------------------------------------------
void motor_stop(void)
{
    motor_set_left(0);
    motor_set_right(0);
}
