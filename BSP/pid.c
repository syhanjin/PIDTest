//
// Created by hanjin on 24-10-14.
//

#include "pid.h"

/**
 * @brief PID初始化
 * @attention 由于输出在 -1 ~ 1 之间，所以参数需要 / MAX_INPUT 来对齐阶数 < 只是猜测，具体需要实践
 * @param hpid PID对象
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param target 输入值稳定目标
 * @param output_abs_max 输出值的绝对值最大值，用于限速&抗饱和
 * @param dfilter
 * @param ideadzone
 */
void PID_Init(PID_t* hpid,
              const float Kp, const float Ki, const float Kd,
              const float target, const float output_abs_max,
              const float dfilter, const float ufilter)
{
    hpid->Kp = Kp;
    hpid->Ki = Ki;
    hpid->Kd = Kd;
    hpid->target = target;
    hpid->output_abs_max = output_abs_max;
    hpid->dfilter = dfilter;
    hpid->ufilter = ufilter;
}


/**
 * @brief 计算pid输出
 * @note 理论上各种地方都能用不是么
 * @param hpid PID对象
 * @param input 输入值
 * @return -1 ~ 1
 */
float PID_Calculate(PID_t* hpid, const float input)
{
    // 计算误差值
    // hpid->error = input - hpid->target;
    hpid->error = hpid->target - input;
    // 计算增量
    /* 比例部分 */
    float p = hpid->Kp * (hpid->error - hpid->error_last1);

    /* 积分部分 */
    float i = hpid->Ki * hpid->error;

    /* 微分部分 */
    float d = hpid->Kd * (hpid->error - 2 * hpid->error_last1 + hpid->error_last2);
    // 对微分部分进行低通滤波
    hpid->filtered_d = hpid->filtered_d * hpid->dfilter + d * (1 - hpid->dfilter);

    float du = p + i + hpid->filtered_d;

    hpid->output += du;

    /* 抗饱和：限制范围，防止跑飞，同时限制速度 */
    if (hpid->output > hpid->output_abs_max)
        hpid->output = hpid->output_abs_max;
    else if (hpid->output < -hpid->output_abs_max)
        hpid->output = -hpid->output_abs_max;

    // 更新误差值
    hpid->error_last2 = hpid->error_last1;
    hpid->error_last1 = hpid->error;
    return hpid->output;
}
