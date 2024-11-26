#ifndef __ST188_H
#define __ST188_H

#include "stm32f4xx_hal.h"       // STM32 HAL库头文件

// 函数声明
void st188_init(void);                          // 初始化ST188传感器
void PID_Init(float kp, float ki, float kd);    // 初始化PID控制器
void PID_Init_Speed_left(float kp_speed, float ki_speed, float kd_speed); // 初始化左轮速度环PID控制器
void PID_Init_Speed_right(float kp_speed, float ki_speed, float kd_speed); // 初始化右轮速度环PID控制器
void PID_Update_Speed_left(int16_t error_speed_left); // 更新左轮速度环PID控制器
void PID_Update_Speed_right(int16_t error_speed_right); // 更新右轮速度环PID控制器
void PID_Update(int16_t error);                   // 更新通用PID控制器
void pid_speed_control_left(int8_t target_speed);  // 左轮PID控制速度
void pid_speed_control_right(int8_t target_speed); // 右轮PID控制速度
void PID_Compute(void);                          // PID计算函数（用于传感器数据处理）

#endif // __TB6612_H
