//
// Created by hanjin on 24-10-14.
//

/**@file pid.h
 * pid控制v2
 * @note 相较于v1对微分部分增加低筒滤波
 */

#ifndef PID_H
#define PID_H

#define PID_VERSION "0.3.1"

#include "main.h"


// 增量式PID
typedef struct
{
    uint8_t enable;
    float target; ///< PID控制要求输入值的目标
    float cutoff; ///< 分界值
    /* v3.0 将pid控制分段 */
    // error < 5 时的参数
    float KpE, KiE, KdE; ///< 比例系数 积分系数 微分系数
    float dfilterE; ///< 微分部分低通滤波系数，越大越平滑
    float ufilterE; ///< 输出值滤波
    /* v3.1 增加死区大小，用于位置环稳定 */
    float ideadzone; ///< 积分部分死区大小
    // error >=5 时的参数
    float KpS, KiS, KdS;
    float ufilterS, dfilterS;
    // 运行过程中间量
    float error_last1, error_last2, error; ///< 前一次误差 前前次误差 本次误差
    float output; ///< PID控制输出值
    float output_abs_max; ///< PID输出绝对值最大值，防止跑飞
    float filtered_d; ///< 微分部分经过滤波后的值
} PID_t;

/**
 * @brief 设置PID控制的目标
 * @param __PID_HANDLE__ PID对象的指针
 * @param __TARGET__ 目标
 */
#define __PID_SET_TARGET(__PID_HANDLE__, __TARGET__) ((__PID_HANDLE__)->target = (__TARGET__))

void PID_Init(PID_t* hpid,
              float KpS, float KiS, float KdS, float ufilterS, float dfilterS,
              float KpE, float KiE, float KdE, float ufilterE, float dfilterE, float ideadzone,
              float output_abs_max, float cutoff, float target
);
float PID_Calculate(PID_t* hpid, const float feedback);
#endif
