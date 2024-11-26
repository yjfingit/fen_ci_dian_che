#ifndef __ENCODE_H
#define __ENCODE_H

#include "stm32f4xx_hal.h"



// ���ֱ�������ʼ������
void encode_init_left(void);
// ���ֱ�������ʼ������
void encode_init_right(void);

// ��ȡ��ǰ���ֱ���������
int32_t get_encoder_count_left(void);

// ��ȡ��ǰ���ֱ���������
int16_t get_encoder_count_right(void);

// �������ֵ��ٶȺ��ۼƾ���
void get_left_wheel_speed_and_distance(void);

// �������ֵ��ٶȺ��ۼƾ���
void get_right_wheel_speed_and_distance(void);



#endif // __ENCODE_H
