/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "oled.h"
#include "bmp.h"
#include "tim.h"
#include "math.h"
#include "tb6612.h"
#include "st188.h"
#include "encode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for oled_Task */
osThreadId_t oled_TaskHandle;
const osThreadAttr_t oled_Task_attributes = {
  .name = "oled_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motor_control */
osThreadId_t motor_controlHandle;
const osThreadAttr_t motor_control_attributes = {
  .name = "motor_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for init_Task */
osThreadId_t init_TaskHandle;
const osThreadAttr_t init_Task_attributes = {
  .name = "init_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void OledTask(void *argument);
void motor_task(void *argument);
void init(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 0, &myBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of oled_Task */
  oled_TaskHandle = osThreadNew(OledTask, NULL, &oled_Task_attributes);

  /* creation of motor_control */
  motor_controlHandle = osThreadNew(motor_task, NULL, &motor_control_attributes);

  /* creation of init_Task */
  init_TaskHandle = osThreadNew(init, NULL, &init_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_OledTask */
/**
  * @brief  Function implementing the oled_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_OledTask */
void OledTask(void *argument)
{
  /* USER CODE BEGIN OledTask */
// 初始化OLED显示屏
  
  // 定义并引用外部变量，用于显示计算得到的速度和累计距离
  extern int32_t car_current_speed_left;      // 外部定义的车速变量（单位：脉冲数/ms）
  extern int32_t car_distance_left;   // 外部定义的累计距离变量（单位：脉冲数）
  
  extern int16_t car_current_speed_right;      // 外部定义的车速变量（单位：脉冲数/ms）
  extern int16_t car_distance_right;   // 外部定义的累计距离变量（单位：脉冲数）
  
  extern uint8_t sensor_value;   // 外部定义的传感器值（例如，传感器的状态或数据）
  
  extern int16_t error_speed_left; //左轮误差
  extern float speed_left_out;//左轮改变量
  extern int16_t speed_left_pwm;//左轮实际输出的pwm波
   
  extern int16_t error_speed_right; //左轮误差
  extern float speed_right_out;//左轮改变量
  extern int16_t speed_right_pwm;//左轮实际输出的pwm波
  extern float error;                       // 当前误差
  
  extern float output;                    // PID输出

  //车轮最高速度120

  /* Infinite loop */
  for(;;)
  { 
    //get_left_wheel_speed_and_distance();
   // pid_speed_control_right(-20);//测试右轮
    //pid_speed_control_left(20);//测试左轮
// 在OLED显示屏上显示传感器值的二进制表示（注释掉了）
    OLED_ShowBinary(0, 0, sensor_value, 8, 8, 1);  // 显示 sensor_value 的 8 位二进制
    // 显示累计的车轮行驶距离（单位：脉冲数，5 位数字，8 像素高，1 行）
    //OLED_ShowNumWithSign(0, 0, car_current_speed_right, 8, 8, 1);//左轮实时速度
    OLED_ShowNumWithSign(0, 16, speed_right_pwm, 8, 8, 1);//左轮输出的pwm
    OLED_ShowNumWithSign(0, 32, error, 8, 8, 1);//与实际的误差
    OLED_ShowNumWithSign(0, 48, output, 8, 8, 1);//实际的输出
    //OLED_ShowNum(0, 16, speed_left_pwm, 8, 8, 1);
    // 刷新OLED显示屏，更新显示内容
    OLED_Refresh();
//    PID_Compute();
    // 延时 1 毫秒

    osDelay(1);
  }
  /* USER CODE END OledTask */
}

/* USER CODE BEGIN Header_motor_task */
/**
* @brief Function implementing the motor_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_task */
void motor_task(void *argument)
{
  /* USER CODE BEGIN motor_task */

  /* Infinite loop */
  
     if (osSemaphoreAcquire(myBinarySem01Handle, osWaitForever) == osOK)
        {//这个二进制信号量我觉得很奇怪，为什么我释放一次，他才能进行一次motor任务呢
            // 获取到信号量后，执行任务
           while(1){
                // PID计算等控制操作
                PID_Compute();
                osDelay(10);  // 延时 10ms，执行控制任务
            }
      }
 
  /* USER CODE END motor_task */
}

/* USER CODE BEGIN Header_init */
/**
* @brief Function implementing the init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_init */
void init(void *argument)
{
  /* USER CODE BEGIN init */
  OLED_Init();
  OLED_Clear();

  tb6612_init();//tb6612初始化
  // 初始化编码器
  encode_init_left() ;
  encode_init_right();
  PID_Init_Speed_right(0.19,0.0001,0);
   PID_Init_Speed_left(0.19,0.0001,0);
   PID_Init(50, 0, 0);  // PID 初始化完成
  

  //PID_Init_Speed(0.19,0.0001,0);//初始化pid_速度函数
  set_B(1);
  set_speed_B(0);
  set_A(1);
  set_speed_A(0);
  // 清空OLED显示屏
  
  // 释放一次信号量
   osSemaphoreRelease(myBinarySem01Handle);  // 释放信号量，通知 motor_task 开始执行
   vTaskDelete(NULL);
  /* Infinite loop */
  /* USER CODE END init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

