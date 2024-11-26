#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"
#include "tb6612.h"
void tb6612_init()
{
  //在stm32cubemx里面进行了pwm配置
  //PWMA PB14 1000hz TIM12 CH1
  //PWMB PB15 1000hz TIM12 CH2
  //AIN1 PD8 
  //AIN2 PD9
  //BIN1 PD10
  //BIN2 PD11
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);//启动TIM12的CH1的PWM输出
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);//启动TIM12的CH2的PWM输出
}

// 设置A电机正反转 0停转 1正 2反 3急停
// 设置A电机的正反转，0停转，1正转，2反转，3急停
void set_A(uint16_t i)//右轮
{
    switch (i)
    {
        case 0:  // 停转
            // 停止A电机：将AIN1和AIN2都置为低电平，电机不转动
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);  // AIN1 置低
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);  // AIN2 置低
            break;

        case 1:  // 正转
            // 正转：AIN1为高电平，AIN2为低电平，电机正转
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);    // AIN1 置高
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);  // AIN2 置低
            break;

        case 2:  // 反转
            // 反转：AIN1为低电平，AIN2为高电平，电机反转
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);  // AIN1 置低
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);    // AIN2 置高
            break;

        case 3:  // 急停
            // 急停：AIN1和AIN2都置为高电平，电机立即停止
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);    // AIN1 置高
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);    // AIN2 置高
            break;
    }
}



// 设置B电机的正反转，0停转，1正转，2反转，3急停
void set_B(uint16_t i)//左轮
{
    switch (i)
    {
        case 0:  // 停转
            // 停止B电机：将BIN1和BIN2都置为低电平，电机不转动
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);  // BIN1 置低
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);  // BIN2 置低
            break;

        case 1:  // 正转
            // 正转：BIN1为高电平，BIN2为低电平，电机正转
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);    // BIN1 置高
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);  // BIN2 置低
            break;

        case 2:  // 反转
            // 反转：BIN1为低电平，BIN2为高电平，电机反转
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);  // BIN1 置低
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);    // BIN2 置高
            break;

        case 3:  // 急停
            // 急停：BIN1和BIN2都置为高电平，电机立即停止
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);    // BIN1 置高
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);    // BIN2 置高
            break;
    }
}

// 直接给A电机设置速度（范围：0-999）
// 直接给A电机设置速度（范围：-999到999），负值表示反转，正值表示正转
void set_speed_A(int16_t ccr)  // 修改ccr为int16_t类型，支持负值
{
    // 限制ccr的范围，确保最大值为999，最小值为-999
    if (ccr > 999)
        ccr = 999;
    if (ccr < -999)
        ccr = -999;

    if (ccr >= 0)
    {
      // 正转控制A电机
       set_A(1);
        // 正转：设置PWM占空比
        __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, ccr);  // 在TIM12的通道1上设置新的占空比

    }
    else
    {
        // 反转控制A电机
       set_A(2);
        // 反转：ccr取负值并设置PWM占空比
        __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, -ccr);  // 在TIM12的通道1上设置新的占空比（负值取反）

      
    }
}

// 直接给B电机设置速度（范围：-999到999），负值表示反转，正值表示正转
void set_speed_B(int16_t ccr)  // 修改ccr为int16_t类型，支持负值
{
    // 限制ccr的范围，确保最大值为999，最小值为-999
    if (ccr > 999)
        ccr = 999;
    if (ccr < -999)
        ccr = -999;

    if (ccr >= 0)
    {
       // 正转控制B电机
        set_B(1);
        // 正转：设置PWM占空比
        __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, ccr);  // 在TIM12的通道2上设置新的占空比

       
    }
    else
    {
       // 反转控制B电机
        set_B(2);
        // 反转：ccr取负值并设置PWM占空比
        __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, -ccr);  // 在TIM12的通道2上设置新的占空比（负值取反）

       
    }
}
