//
// Created by hanjin on 24-10-14.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"
#define MAX_COMPARE     (1000U - 1U)    ///< 其实就是自动重载值

#define ROTO_RADIO      (4 * 13)        ///< 倍频器*线数
#define REDUCTION_RADIO (30)            ///< 减速比 30:1
#define MAX_SPEED       (366)           ///< 最大转速(rpm)

#define SAMPLING_PERIOD (1e-2)          ///< 采样间隔(s)

/**
 * @def __SPEED2COMPARE
 * @brief 将速度转化为CCR
 * @attention 结果四舍五入
 * @param SPEED 速度 [0, 1]
 */
#define __SPEED2CCR(SPEED) (uint16_t)((SPEED) * (MAX_COMPARE) + 0.5)

typedef struct
{
    TIM_HandleTypeDef* const htim;
    const uint32_t channel;
} TIM_Channel_t;

typedef struct
{
    GPIO_TypeDef* const GPIOx;
    const uint16_t GPIO_Pin;
} GPIO_Out_t;

// TODO: 直接用speed正负来表示方向 < 但是也许从实用的角度不需要？
typedef struct
{
    const TIM_Channel_t pwm;
    // const TIM_Channel_t encoder1, encoder2;
    TIM_HandleTypeDef* const encoder;
    const GPIO_Out_t in1, in2;
    uint8_t direction; ///< 方向：0 正向 1 反向
    float speed; ///< 速度：0 <= speed <= 1

    float real_speed; ///< 由编码器测得的速度
    float real_round; ///< 由编码器测得的圈数
} Motor_t;

// void Motor_Enable();
// void Motor_Disable();

void Motor_Start(const Motor_t* hmotor);
void Motor_Stop(const Motor_t* hmotor);
void Motor_SetSpeed(Motor_t* hmotor, float speed);
void Motor_SetDirection(Motor_t* hmotor, uint8_t direction);
void Motor_ToggleDirection(Motor_t* hmotor);

void Encoder_Start(Motor_t* hmotor);
void Encoder_Stop(Motor_t* hmotor);
void Encoder_Progress(Motor_t* hmotor);
float Encoder_GetSpeed(const Motor_t* hmotor);
#endif
