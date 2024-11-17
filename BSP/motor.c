//
// Created by hanjin on 24-10-14.
//

#include "motor.h"
#include <stdio.h>

/**
 * @brief 应用方向变更，也可以用于启动
 * @note 为啥是inline呢，因为我只是不想写两遍罢了
 * @param hmotor
 * @retval None
 */
inline void _Perform_SetDirection(const Motor_t* hmotor)
{
    if (hmotor->direction == 0) // 正向旋转
    {
        HAL_GPIO_WritePin(hmotor->in1.GPIOx, hmotor->in1.GPIO_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hmotor->in2.GPIOx, hmotor->in2.GPIO_Pin, GPIO_PIN_SET);
    } else
    {
        HAL_GPIO_WritePin(hmotor->in1.GPIOx, hmotor->in1.GPIO_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(hmotor->in2.GPIOx, hmotor->in2.GPIO_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief 使能驱动芯片
 * @note 打算不使用STBY引脚，想直接给他接3v3
 */
// void Motor_Enable()
// {
//     HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
// }

/**
 * @brief 失能驱动芯片
 */
// void Motor_Disable()
// {
//     HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
// }

/**
 * @brief 启动电机
 * @attention 本函数不会设置速度，所以会按照原本速度运行
 * @param hmotor 电机
 * @retval None
 */
void Motor_Start(const Motor_t* hmotor)
{
    _Perform_SetDirection(hmotor);
    __HAL_TIM_SET_COMPARE(hmotor->pwm.htim, hmotor->pwm.channel, __SPEED2CCR(hmotor->speed)); // 根据速度设置占空比
    HAL_TIM_PWM_Start(hmotor->pwm.htim, hmotor->pwm.channel);
}

/**
 * @brief 停止电机
 * @param hmotor
 */
void Motor_Stop(const Motor_t* hmotor)
{
    HAL_TIM_PWM_Stop(hmotor->pwm.htim, hmotor->pwm.channel);
    HAL_GPIO_WritePin(hmotor->in1.GPIOx, hmotor->in1.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hmotor->in2.GPIOx, hmotor->in2.GPIO_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 设置速度
 * @param hmotor
 * @param speed -1<= speed <= 1
 */
inline void Motor_SetSpeed(Motor_t* hmotor, float speed)
{
    if (speed > 0 && hmotor->direction == 1)
        Motor_SetDirection(hmotor, 0);
    else if (speed < 0 && hmotor->direction == 0)
        Motor_SetDirection(hmotor, 1);
    if (speed < 0) speed = -speed;
    hmotor->speed = speed;
    __HAL_TIM_SET_COMPARE(hmotor->pwm.htim, hmotor->pwm.channel, __SPEED2CCR(speed));
}

/**
 * @brief 设置方向
 * @param hmotor
 * @param direction 0 正转  1 反转
 */
void Motor_SetDirection(Motor_t* hmotor, const uint8_t direction)
{
    hmotor->direction = direction;
    _Perform_SetDirection(hmotor);
}

/**
 * @brief 切换方向
 * @param hmotor
 */
inline void Motor_ToggleDirection(Motor_t* hmotor)
{
    Motor_SetDirection(hmotor, !hmotor->direction);
}

/**
 * @brief 启动编码器
 * @param hmotor
 */
inline void Encoder_Start(Motor_t* hmotor)
{
    HAL_TIM_Encoder_Start(hmotor->encoder, TIM_CHANNEL_ALL);
}

/**
 * @brief 关闭编码器
 * @param hmotor
 */
inline void Encoder_Stop(Motor_t* hmotor)
{
    HAL_TIM_Encoder_Stop(hmotor->encoder, TIM_CHANNEL_ALL);
}

/**
 * @brief 在定时器中处理编码器的数据
 * @note 双边沿计数，TI1 and TI2
 * @param hmotor
 */
void Encoder_Progress(Motor_t* hmotor)
{
    // 此处需要保证 counter < 32768 正常电机转不了那么快
    const int16_t counter = __HAL_TIM_GET_COUNTER(hmotor->encoder);
    // printf("%d", counter);
    hmotor->real_round += (float)counter / (ROTO_RADIO * REDUCTION_RADIO); // 转过的圈数
    hmotor->real_speed = (float)counter / (ROTO_RADIO * REDUCTION_RADIO) / SAMPLING_PERIOD * 60; // 实际转速(rpm)
    __HAL_TIM_SET_COUNTER(hmotor->encoder, 0);
}

/**
 * @brief 获取电机速度
 * @param hmotor
 * @return 电机速度(rpm)
 */
inline float Encoder_GetSpeed(const Motor_t* hmotor)
{
    return hmotor->real_speed;
}
