#ifndef __ENCODE_H
#define __ENCODE_H

#include "stm32f4xx_hal.h"



// 左轮编码器初始化函数
void encode_init_left(void);
// 右轮编码器初始化函数
void encode_init_right(void);

// 获取当前左轮编码器计数
int32_t get_encoder_count_left(void);

// 获取当前右轮编码器计数
int16_t get_encoder_count_right(void);

// 计算左轮的速度和累计距离
void get_left_wheel_speed_and_distance(void);

// 计算右轮的速度和累计距离
void get_right_wheel_speed_and_distance(void);



#endif // __ENCODE_H
