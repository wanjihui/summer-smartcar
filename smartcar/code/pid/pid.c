/*******************************************************************************
 * PID — 移植自 ASC-Summer26-Training P1/P2 速度环
 *
 * 增量式PID: delta = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2e1+e2)
 * 每次调用前需更新 Motor_PID.Target 和 Motor_PID.Actual
 *******************************************************************************/

#include "pid.h"
#include "motor.h"

/**********************************************************/
/*[S] 增量式PID [S]---------------------------------------*/
/**********************************************************/

void PID_INC_Init(PID_INC_t *p)
{
    p->Target = 0; p->Actual = 0; p->Out = 0;
    p->Error0 = 0; p->Error1 = 0; p->Error2 = 0;
}

void PID_INC_Update(PID_INC_t *p)
{
    /* 误差更新 */
    p->Error2 = p->Error1;
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    /* 增量式计算 */
    float delta = p->Kp * (p->Error0 - p->Error1)
                + p->Ki * p->Error0
                + p->Kd * (p->Error0 - 2.0f * p->Error1 + p->Error2);

    /* 单次增量限幅（防wheelspin）*/
    if (p->OutDeltaMax > 0.0f)
    {
        if (delta >  p->OutDeltaMax) delta =  p->OutDeltaMax;
        if (delta < -p->OutDeltaMax) delta = -p->OutDeltaMax;
    }

    /* 累加至输出 */
    p->Out += delta;

    /* 输出限幅 */
    if (p->Out > p->OutMax) p->Out = p->OutMax;
    if (p->Out < p->OutMin) p->Out = p->OutMin;
}

/**********************************************************/
/*[E] 增量式PID [E]---------------------------------------*/
/**********************************************************/

/**********************************************************/
/*[S] 速度环 [S]------------------------------------------*/
/**********************************************************/

/* 实例 */
PID_INC_t Motor_L_PID = {
    .OutMax =  MOTOR_MAX_DUTY,
    .OutMin = -MOTOR_MAX_DUTY,
    .OutDeltaMax = 6.0f,   /* 单次增量上限(duty/5ms)，200Hz */
};

PID_INC_t Motor_R_PID = {
    .OutMax =  MOTOR_MAX_DUTY,
    .OutMin = -MOTOR_MAX_DUTY,
    .OutDeltaMax = 6.0f,
};

volatile uint8_t Speed_PID_Enable = 0;

void Speed_PID_Init(void)
{
    PID_INC_Init(&Motor_L_PID);
    PID_INC_Init(&Motor_R_PID);
}

void Speed_PID_Crtl(void)
{
    if (!Speed_PID_Enable) return;

    PID_INC_Update(&Motor_L_PID);
    PID_INC_Update(&Motor_R_PID);

    motor_set_left ((int8_t)Motor_L_PID.Out);
    motor_set_right((int8_t)Motor_R_PID.Out);
}

/**********************************************************/
/*[E] 速度环 [E]------------------------------------------*/
/**********************************************************/
