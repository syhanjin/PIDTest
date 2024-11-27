//
// Created by hanjin on 24-10-14.
//

#include "pid.h"

/**
 * @brief PID初始化
 * @attention 由于输出在 -1 ~ 1 之间，所以参数需要 / MAX_INPUT 来对齐阶数 < 只是猜测，具体需要实践
 * @param hpid PID对讲指针
 * @param KpS 初始比例系数
 * @param KiS 初始积分系数
 * @param KdS 初始微分系数
 * @param ufilterS 初始输出滤波系数
 * @param dfilterS 初始微分部分滤波系数
 * @param KpE 末尾比例系数
 * @param KiE 末尾积分系数
 * @param KdE 末尾微分系数
 * @param ufilterE 末尾输出滤波系数
 * @param dfilterE 末尾微分部分滤波系数
 * @param ideadzone 积分部分死区大小
 * @param output_abs_max 输出绝对值最大限制
 * @param cutoff 初始与末尾分隔量 abs(error) < cutoff 视为末尾状态
 * @param target PID控制目标
 */
void PID_Init(PID_t* hpid,
              float KpS, float KiS, float KdS, float ufilterS, float dfilterS,
              float KpE, float KiE, float KdE, float ufilterE, float dfilterE, float ideadzone,
              float output_abs_max, float cutoff, float target
)
{
    hpid->target = target;
    hpid->output_abs_max = output_abs_max;
    hpid->cutoff = cutoff;

    hpid->KpS = KpS, hpid->KiS = KiS, hpid->KdS = KdS, hpid->ufilterS = ufilterS, hpid->dfilterS = dfilterS;
    hpid->KpE = KpE, hpid->KiE = KiE, hpid->KdE = KdE, hpid->ufilterE = ufilterE, hpid->dfilterE = dfilterE, hpid->ideadzone = ideadzone;
}


/**
 * @brief 计算pid输出
 * @note 理论上各种地方都能用不是么
 * @param hpid PID对象
 * @param feedback 输入值
 * @return -1 ~ 1
 */
float PID_Calculate(PID_t* hpid, const float feedback)
{
    // 计算误差值
    // hpid->error = input - hpid->target;
    hpid->error = hpid->target - feedback;
    // 计算增量
    float Kp, Ki, Kd, ufilter, dfilter;
    if (hpid->error < hpid->cutoff && hpid->error > -hpid->cutoff)
        Kp = hpid->KpE, Ki = hpid->KiE, Kd = hpid->KdE, ufilter = hpid->ufilterE, dfilter = hpid->dfilterE;
    else
        Kp = hpid->KpS, Ki = hpid->KiS, Kd = hpid->KdS, ufilter = hpid->ufilterS, dfilter = hpid->dfilterS;
    /* 比例部分 */
    float p = Kp * (hpid->error - hpid->error_last1);

    /* 积分部分 */
    float i = 0;
    if (hpid->error > hpid->ideadzone || hpid->error < -hpid->ideadzone) i = Ki * hpid->error;

    /* 微分部分 */
    float d = Kd * (hpid->error - 2 * hpid->error_last1 + hpid->error_last2);
    // 对微分部分进行低通滤波
    hpid->filtered_d = hpid->filtered_d * dfilter + d * (1 - dfilter);

    float du = p + i + hpid->filtered_d;

    hpid->output += du * (1 - ufilter);

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
