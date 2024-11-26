#include "stm32f4xx_hal.h"       // STM32 HAL库头文件
#include "main.h"                 // 主程序头文件
#include "cmsis_os.h"             // CMSIS-RTOS库头文件
#include "tim.h"                  // 定时器相关头文件
#include "gpio.h"                 // GPIO相关头文件
#include "st188.h"                // ST188传感器驱动头文件
#include "tb6612.h"               // TB6612电机驱动控制头文件
#include "encode.h"               // 编码器相关头文件
#include <stdlib.h>               // 包含 abs 函数的头文件
// AB轮最高速度120

// PID控制器参数
float Kp = 0, Ki = 0, Kd = 0;         // PID参数：比例、积分、微分
float integral = 0.0;                  // 积分项
float previous_error = 0.0;            // 上一次误差，用于计算微分项
float output = 0.0;                    // PID输出
float error = 0;                       // 当前误差


uint16_t speed=50;                        // 速度，可以在主函数中设置
uint16_t speed_left_motor;             // 左轮速度
uint16_t speed_right_motor;            // 右轮速度
uint8_t sensor_value = 0;              // 传感器读取值，用于反馈控制

// 左轮PID控制参数
float Kp_speed_left = 0, Ki_speed_left = 0, Kd_speed_left = 0;
float previous_error_speed_left = 0;   // 上次的误差
float integral_speed_left = 0;         // 积分项
int16_t speed_left_pwm = 0;            // 左轮PWM输出
float speed_left_out = 0;              //右轮实际的改变量
int16_t error_speed_left = 0;         //右轮实际误差

// 右轮PID控制参数
float Kp_speed_right = 0, Ki_speed_right = 0, Kd_speed_right = 0;
float previous_error_speed_right = 0;  // 上次的误差
float integral_speed_right = 0;        // 积分项
int16_t speed_right_pwm = 0;           // 右轮PWM输出
float speed_right_out = 0;             //右轮实际的改变量
int16_t error_speed_right = 0;         //右轮实际误差

int16_t current_speed = 0;             // 当前速度（单位：脉冲数/ms）
int16_t error_speed = 0;               // 速度误差
extern int16_t car_current_speed_left;
extern int16_t car_current_speed_right; // 右轮速度

// 初始化函数：设置ST188传感器和GPIO
void st188_init() {
    // 在gpio里面已经定义了ST188传感器的初始化
}

// 初始化PID控制器
void PID_Init(float kp, float ki, float kd) {
    Kp = kp;  // 设置比例系数
    Ki = ki;  // 设置积分系数
    Kd = kd;  // 设置微分系数

}

// 初始化左轮速度环PID控制器
void PID_Init_Speed_left(float kp, float ki, float kd) {
    Kp_speed_left = kp;       // 设置左轮速度环比例系数
    Ki_speed_left = ki;       // 设置左轮速度环积分系数
    Kd_speed_left = kd;       // 设置左轮速度环微分系数
}

// 初始化右轮速度环PID控制器
void PID_Init_Speed_right(float kp, float ki, float kd) {
    Kp_speed_right = kp;      // 设置右轮速度环比例系数
    Ki_speed_right = ki;      // 设置右轮速度环积分系数
    Kd_speed_right = kd;      // 设置右轮速度环微分系数
}

// 更新PID控制器
void PID_Update(int16_t error) {
    // 计算误差的微分项
    float derivative = error - previous_error; 
    // 累加误差（积分项）
    integral += error; 
    // 计算PID输出
    output = Kp * error + Ki * integral + Kd * derivative; 
    // 更新前一次误差
    previous_error = error; 
}

// 更新左轮PID控制器
void PID_Update_Speed_left(int16_t error_speed_left) {
    // 计算左轮误差的微分项
    float derivative_speed_left = error_speed_left - previous_error_speed_left;
    // 累加左轮误差（积分项）
    integral_speed_left += error_speed_left;
    // 更新左轮前一次的误差
    previous_error_speed_left = error_speed_left;
    // 计算左轮PID输出
    speed_left_out = Kp_speed_left * error_speed_left + Ki_speed_left * integral_speed_left + Kd_speed_left * derivative_speed_left;

}

// 更新右轮PID控制器
void PID_Update_Speed_right(int16_t error_speed_right) {
    // 计算右轮误差的微分项
    float derivative_speed_right = error_speed_right - previous_error_speed_right;
    // 累加右轮误差（积分项）
    integral_speed_right += error_speed_right;
    // 更新右轮前一次的误差
    previous_error_speed_right = error_speed_right;
    // 计算右轮PID输出
    speed_right_out = Kp_speed_right * error_speed_right + Ki_speed_right * integral_speed_right + Kd_speed_right * derivative_speed_right;
  
}

// 控制左轮速度的PID函数
void pid_speed_control_left(int8_t target_speed) {
    // 获取当前实际速度
    get_left_wheel_speed_and_distance();

    // 计算目标速度与当前速度之间的误差
    error_speed_left = target_speed - car_current_speed_left;

    // 更新速度环PID控制器
    PID_Update_Speed_left(error_speed_left);

    // 限制PID输出的最大值，避免超过电机的最大速度
    speed_left_pwm =speed_left_pwm+ speed_left_out;  // 左轮的目标速度
    if (speed_left_pwm > 999) {
        speed_left_pwm = 999;
    } else if (speed_left_pwm < -999) {
        speed_left_pwm = -999;
    }

    // 调整电机速度
    set_speed_B(speed_left_pwm);
}

// 控制右轮速度的PID函数
void pid_speed_control_right(int8_t target_speed) {
    // 获取当前实际速度
    get_right_wheel_speed_and_distance(); // 只是左轮的

    // 计算目标速度与当前速度之间的误差
    error_speed_right = target_speed - car_current_speed_right;

    // 更新速度环PID控制器
    PID_Update_Speed_right(error_speed_right);

    // 限制PID输出的最大值，避免超过电机的最大速度
    speed_right_pwm += speed_right_out;  // 右轮的目标速度
     if (speed_right_pwm > 800) {
        speed_right_pwm = 800;
    } else if (speed_right_pwm < -800) {
        speed_right_pwm = -800;
    }
    set_speed_A(speed_right_pwm);
}

// PID计算函数 - 循迹模块角度环
void PID_Compute(void) {
    error = 0;  // 初始化误差

    // 读取ST188八路传感器输入信号
    sensor_value = 0;
    sensor_value |= HAL_GPIO_ReadPin(st188_1_GPIO_Port, st188_1_Pin) << 0; // S1 -> bit0
    sensor_value |= HAL_GPIO_ReadPin(st188_2_GPIO_Port, st188_2_Pin) << 1; // S2 -> bit1
    sensor_value |= HAL_GPIO_ReadPin(st188_3_GPIO_Port, st188_3_Pin) << 2; // S3 -> bit2
    sensor_value |= HAL_GPIO_ReadPin(st188_4_GPIO_Port, st188_4_Pin) << 3; // S4 -> bit3
    sensor_value |= HAL_GPIO_ReadPin(st188_5_GPIO_Port, st188_5_Pin) << 4; // S5 -> bit4
    sensor_value |= HAL_GPIO_ReadPin(st188_6_GPIO_Port, st188_6_Pin) << 5; // S6 -> bit5
    sensor_value |= HAL_GPIO_ReadPin(st188_7_GPIO_Port, st188_7_Pin) << 6; // S7 -> bit6
    sensor_value |= HAL_GPIO_ReadPin(st188_8_GPIO_Port, st188_8_Pin) << 7; // S8 -> bit7

    // 根据传感器值判断误差
    switch (sensor_value) {
        case 0x80: error = -4.0; break; // S1 -> 10000000
        case 0x40: error = -3.0; break; // S2 -> 01000000
        case 0x20: error = -2.0; break; // S3 -> 00100000
        case 0x10: error = -1.0; break; // S4 -> 00010000
        case 0x08: error = 1.0; break;  // S5 -> 00001000
        case 0x04: error = 2.0; break;  // S6 -> 00000100
        case 0x02: error = 3.0; break;  // S7 -> 00000010
        case 0x01: error = 4.0; break;  // S8 -> 00000001

        // 两个相邻传感器检测黑线
        case 0xC0: error = -3.5; break; // S1, S2 -> 11000000
        case 0x60: error = -2.5; break; // S2, S3 -> 01100000
        case 0x30: error = -1.5; break; // S3, S4 -> 00110000
        case 0x18: error = 0.0; break;  // S4, S5 -> 00011000
        case 0x0C: error = 1.5; break;  // S5, S6 -> 00001100
        case 0x06: error = 2.5; break;  // S6, S7 -> 00000110
        case 0x03: error = 3.5; break;  // S7, S8 -> 00000011

        // 多个传感器检测黑线（宽轨迹）
        case 0xE0: error = -3.0; break; // S1, S2, S3 -> 11100000
        case 0x70: error = -1.0; break; // S2, S3, S4 -> 01110000
        case 0x38: error = 0.0; break;  // S3, S4, S5 -> 00111000
        case 0x1C: error = 1.0; break;  // S4, S5, S6 -> 00011100
        case 0x0E: error = 3.0; break;  // S5, S6, S7 -> 00001110

        // 特殊情况：全黑线或无信号
        case 0xFF: error = 0.0; break;  // 全黑线，保持当前方向
        case 0x00: error = previous_error; break; // 无信号，可能偏离轨道

        // 默认处理
        default: error = 0.0; break;
    }

    // 更新PID控制器
    PID_Update(error);
    
    // 调整小车的速度（根据PID输出值）
    int16_t speed_right = speed + (int16_t)output;
    int16_t speed_left = speed - (int16_t)output;
    
//    // 设置电机方向（正转或反转）
//    int16_t speed_right_motor = abs(speed_right);
//    int16_t speed_left_motor = abs(speed_left);
//    
    // 控制左轮和右轮的方向（1正转，2反转）
//    set_A(speed_left > 0 ? 1 : 2); // 左轮方向
//    set_B(speed_right > 0 ? 1 : 2); // 右轮方向

    // 限制速度范围（最大速度设定为80）
    if (speed_right > 100) speed_right = 100;
    else if(speed_right <-100) speed_right = -100;
    
    if (speed_left > 100) speed_left = 100;
    else if (speed_left <-100) speed_left = -100;
    // 设置电机速度
    pid_speed_control_right(speed_right);
    pid_speed_control_left(speed_left);
}
