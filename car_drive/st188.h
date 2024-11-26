#ifndef __ST188_H
#define __ST188_H

#include "stm32f4xx_hal.h"       // STM32 HAL��ͷ�ļ�

// ��������
void st188_init(void);                          // ��ʼ��ST188������
void PID_Init(float kp, float ki, float kd);    // ��ʼ��PID������
void PID_Init_Speed_left(float kp_speed, float ki_speed, float kd_speed); // ��ʼ�������ٶȻ�PID������
void PID_Init_Speed_right(float kp_speed, float ki_speed, float kd_speed); // ��ʼ�������ٶȻ�PID������
void PID_Update_Speed_left(int16_t error_speed_left); // ���������ٶȻ�PID������
void PID_Update_Speed_right(int16_t error_speed_right); // ���������ٶȻ�PID������
void PID_Update(int16_t error);                   // ����ͨ��PID������
void pid_speed_control_left(int8_t target_speed);  // ����PID�����ٶ�
void pid_speed_control_right(int8_t target_speed); // ����PID�����ٶ�
void PID_Compute(void);                          // PID���㺯�������ڴ��������ݴ���

#endif // __TB6612_H
