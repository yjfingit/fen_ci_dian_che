#ifndef __TB6612_H
#define __TB6612_H

#include "stm32f4xx_hal.h"  // ���� HAL ��ͷ�ļ�

// ������ƺ�������
// ��ʼ��TB6612����
void tb6612_init(void);

// ����A���������ת��0ͣת��1��ת��2��ת��3��ͣ
void set_A(uint16_t i);

// ����B���������ת��0ͣת��1��ת��2��ת��3��ͣ
void set_B(uint16_t i);

// ����A������ٶȣ���Χ��0-999�����޸�A�����PWMռ�ձ�
void set_speed_A(int16_t ccr);

// ����B������ٶȣ���Χ��0-999�����޸�B�����PWMռ�ձ�
void set_speed_B(int16_t ccr);

#endif /* __TB6612_H */
