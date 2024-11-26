#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"
#include "encode.h"

// ��̬���������ڱ����ϴεı���������
static int32_t last_encoder_count_left = 0;
static int16_t last_encoder_count_right = 0;  // �������ֵ��ϴμ���

// �����ٶȱ������洢����õ����ٶȣ���λ��������/ms��
int32_t car_current_speed_left = 0;
int16_t car_current_speed_right = 0;  // �����ٶ�

// �����ۼƾ������
int32_t car_distance_left = 0;
int16_t car_distance_right = 0;  // �����ۼƾ���

// ���ֱ�������ʼ������
void encode_init_left() {
    // �������ֱ������ļ������ܣ�ʹ��TIM2��ͨ��1��ͨ��2
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);

    // ��ʼ���ϴε����ֱ���������
    last_encoder_count_left = __HAL_TIM_GET_COUNTER(&htim2);
}

// ���ֱ�������ʼ������
void encode_init_right() {
    // �������ֱ������ļ������ܣ�ʹ��TIM4��ͨ��1��ͨ��2
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

    // ��ʼ���ϴε����ֱ���������
    last_encoder_count_right = __HAL_TIM_GET_COUNTER(&htim4);
}

// ��ȡ��ǰ���ֱ���������
int32_t get_encoder_count_left() {
    taskENTER_CRITICAL();

    // ��ȡ��ǰ���ֱ������ļ���ֵ
    int32_t count = __HAL_TIM_GET_COUNTER(&htim2) * 4;  // ����ÿ��������4������

    // ����ʱ�����������㣬׼����һ�μ���
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    taskEXIT_CRITICAL();

    return count;
}

// ��ȡ��ǰ���ֱ���������
int16_t get_encoder_count_right() {
    taskENTER_CRITICAL();

    // ��ȡ��ǰ���ֱ������ļ���ֵ
    int16_t count = __HAL_TIM_GET_COUNTER(&htim4) * 2;  // ����ÿ��������4������

    // ����ʱ�����������㣬׼����һ�μ���
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    taskEXIT_CRITICAL();

    return count;
}

// �������ֵ��ٶȺ��ۼƾ���
void get_left_wheel_speed_and_distance() {
    // ��ȡ���ֱ�����������
    int32_t delta_count_left = get_encoder_count_left();
    
    // �������ֵĵ�ǰ�ٶȣ���λ��������/ms��
    car_current_speed_left = delta_count_left;
    
    // �ۼ����������ֵ��ۼƾ���
    car_distance_left += car_current_speed_left;
}

// �������ֵ��ٶȺ��ۼƾ���
void get_right_wheel_speed_and_distance() {
    // ��ȡ���ֱ�����������
    int16_t delta_count_right = get_encoder_count_right();
    
    // �������ֵĵ�ǰ�ٶȣ���λ��������/ms��
    car_current_speed_right = delta_count_right;
    
    // �ۼ����������ֵ��ۼƾ���
    car_distance_right += car_current_speed_right;
}


