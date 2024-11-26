#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"
#include "encode.h"

// 静态变量，用于保存上次的编码器计数
static int32_t last_encoder_count_left = 0;
static int16_t last_encoder_count_right = 0;  // 用于右轮的上次计数

// 定义速度变量，存储计算得到的速度（单位：脉冲数/ms）
int32_t car_current_speed_left = 0;
int16_t car_current_speed_right = 0;  // 右轮速度

// 定义累计距离变量
int32_t car_distance_left = 0;
int16_t car_distance_right = 0;  // 右轮累计距离

// 左轮编码器初始化函数
void encode_init_left() {
    // 启动左轮编码器的计数功能，使用TIM2的通道1和通道2
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);

    // 初始化上次的左轮编码器计数
    last_encoder_count_left = __HAL_TIM_GET_COUNTER(&htim2);
}

// 右轮编码器初始化函数
void encode_init_right() {
    // 启动右轮编码器的计数功能，使用TIM4的通道1和通道2
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

    // 初始化上次的右轮编码器计数
    last_encoder_count_right = __HAL_TIM_GET_COUNTER(&htim4);
}

// 获取当前左轮编码器计数
int32_t get_encoder_count_left() {
    taskENTER_CRITICAL();

    // 获取当前左轮编码器的计数值
    int32_t count = __HAL_TIM_GET_COUNTER(&htim2) * 4;  // 假设每个脉冲有4个计数

    // 将定时器计数器清零，准备下一次计数
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    taskEXIT_CRITICAL();

    return count;
}

// 获取当前右轮编码器计数
int16_t get_encoder_count_right() {
    taskENTER_CRITICAL();

    // 获取当前右轮编码器的计数值
    int16_t count = __HAL_TIM_GET_COUNTER(&htim4) * 2;  // 假设每个脉冲有4个计数

    // 将定时器计数器清零，准备下一次计数
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    taskEXIT_CRITICAL();

    return count;
}

// 计算左轮的速度和累计距离
void get_left_wheel_speed_and_distance() {
    // 获取左轮编码器的增量
    int32_t delta_count_left = get_encoder_count_left();
    
    // 更新左轮的当前速度（单位：脉冲数/ms）
    car_current_speed_left = delta_count_left;
    
    // 累加增量到左轮的累计距离
    car_distance_left += car_current_speed_left;
}

// 计算右轮的速度和累计距离
void get_right_wheel_speed_and_distance() {
    // 获取右轮编码器的增量
    int16_t delta_count_right = get_encoder_count_right();
    
    // 更新右轮的当前速度（单位：脉冲数/ms）
    car_current_speed_right = delta_count_right;
    
    // 累加增量到右轮的累计距离
    car_distance_right += car_current_speed_right;
}


