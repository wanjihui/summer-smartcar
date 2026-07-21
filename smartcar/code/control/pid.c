#include "config.h" 

//pid参数初始,存入pid_t的变量,pid三个参数,ilimit积分限幅,omin omax输出限幅
void pid_init(pid_t *p, float kp, float ki, float kd,
              float ilimit, float omin, float omax)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integral = 0; p->err = 0;
    p->last_err = 0; p->prev_err = 0;
    p->output = 0; p->i_limit = ilimit;
    p->omin = omin; p->omax = omax;
}

// 增量式：输出变化量，累加到上次输出
float pid_step(pid_t *p, float error)
{
    p->err = error;
    float delta = p->kp * (p->err - p->last_err)
                + p->ki * p->err
                + p->kd * (p->err - 2*p->last_err + p->prev_err);
    p->prev_err = p->last_err;
    p->last_err = p->err;
    p->output += delta;
    if (p->output > p->omax) p->output = p->omax;
    if (p->output < p->omin) p->output = p->omin;
    return p->output;
}

// 位置式：输出绝对值
float pid_pos(pid_t *p, float error)
{
    p->err = error;
    p->integral += p->err;
    if (p->integral >  p->i_limit) p->integral =  p->i_limit;
    if (p->integral < -p->i_limit) p->integral = -p->i_limit;
    p->output = p->kp * p->err
              + p->ki * p->integral
              + p->kd * (p->err - p->last_err);
    p->last_err = p->err;
    if (p->output > p->omax) p->output = p->omax;
    if (p->output < p->omin) p->output = p->omin;
    return p->output;
}