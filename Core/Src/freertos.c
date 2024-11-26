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
// ��ʼ��OLED��ʾ��
  
  // ���岢�����ⲿ������������ʾ����õ����ٶȺ��ۼƾ���
  extern int32_t car_current_speed_left;      // �ⲿ����ĳ��ٱ�������λ��������/ms��
  extern int32_t car_distance_left;   // �ⲿ������ۼƾ����������λ����������
  
  extern int16_t car_current_speed_right;      // �ⲿ����ĳ��ٱ�������λ��������/ms��
  extern int16_t car_distance_right;   // �ⲿ������ۼƾ����������λ����������
  
  extern uint8_t sensor_value;   // �ⲿ����Ĵ�����ֵ�����磬��������״̬�����ݣ�
  
  extern int16_t error_speed_left; //�������
  extern float speed_left_out;//���ָı���
  extern int16_t speed_left_pwm;//����ʵ�������pwm��
   
  extern int16_t error_speed_right; //�������
  extern float speed_right_out;//���ָı���
  extern int16_t speed_right_pwm;//����ʵ�������pwm��
  extern float error;                       // ��ǰ���
  
  extern float output;                    // PID���

  //��������ٶ�120

  /* Infinite loop */
  for(;;)
  { 
    //get_left_wheel_speed_and_distance();
   // pid_speed_control_right(-20);//��������
    //pid_speed_control_left(20);//��������
// ��OLED��ʾ������ʾ������ֵ�Ķ����Ʊ�ʾ��ע�͵��ˣ�
    OLED_ShowBinary(0, 0, sensor_value, 8, 8, 1);  // ��ʾ sensor_value �� 8 λ������
    // ��ʾ�ۼƵĳ�����ʻ���루��λ����������5 λ���֣�8 ���ظߣ�1 �У�
    //OLED_ShowNumWithSign(0, 0, car_current_speed_right, 8, 8, 1);//����ʵʱ�ٶ�
    OLED_ShowNumWithSign(0, 16, speed_right_pwm, 8, 8, 1);//���������pwm
    OLED_ShowNumWithSign(0, 32, error, 8, 8, 1);//��ʵ�ʵ����
    OLED_ShowNumWithSign(0, 48, output, 8, 8, 1);//ʵ�ʵ����
    //OLED_ShowNum(0, 16, speed_left_pwm, 8, 8, 1);
    // ˢ��OLED��ʾ����������ʾ����
    OLED_Refresh();
//    PID_Compute();
    // ��ʱ 1 ����

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
        {//����������ź����Ҿ��ú���֣�Ϊʲô���ͷ�һ�Σ������ܽ���һ��motor������
            // ��ȡ���ź�����ִ������
           while(1){
                // PID����ȿ��Ʋ���
                PID_Compute();
                osDelay(10);  // ��ʱ 10ms��ִ�п�������
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

  tb6612_init();//tb6612��ʼ��
  // ��ʼ��������
  encode_init_left() ;
  encode_init_right();
  PID_Init_Speed_right(0.19,0.0001,0);
   PID_Init_Speed_left(0.19,0.0001,0);
   PID_Init(50, 0, 0);  // PID ��ʼ�����
  

  //PID_Init_Speed(0.19,0.0001,0);//��ʼ��pid_�ٶȺ���
  set_B(1);
  set_speed_B(0);
  set_A(1);
  set_speed_A(0);
  // ���OLED��ʾ��
  
  // �ͷ�һ���ź���
   osSemaphoreRelease(myBinarySem01Handle);  // �ͷ��ź�����֪ͨ motor_task ��ʼִ��
   vTaskDelete(NULL);
  /* Infinite loop */
  /* USER CODE END init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

