#ifndef __TB6612_H
#define __TB6612_H

#include "stm32f4xx_hal.h"  // 引入 HAL 库头文件

// 电机控制函数声明
// 初始化TB6612驱动
void tb6612_init(void);

// 设置A电机的正反转，0停转，1正转，2反转，3急停
void set_A(uint16_t i);

// 设置B电机的正反转，0停转，1正转，2反转，3急停
void set_B(uint16_t i);

// 设置A电机的速度（范围：0-999），修改A电机的PWM占空比
void set_speed_A(int16_t ccr);

// 设置B电机的速度（范围：0-999），修改B电机的PWM占空比
void set_speed_B(int16_t ccr);

#endif /* __TB6612_H */
