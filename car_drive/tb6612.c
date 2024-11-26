#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"
#include "tb6612.h"
void tb6612_init()
{
  //��stm32cubemx���������pwm����
  //PWMA PB14 1000hz TIM12 CH1
  //PWMB PB15 1000hz TIM12 CH2
  //AIN1 PD8 
  //AIN2 PD9
  //BIN1 PD10
  //BIN2 PD11
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);//����TIM12��CH1��PWM���
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);//����TIM12��CH2��PWM���
}

// ����A�������ת 0ͣת 1�� 2�� 3��ͣ
// ����A���������ת��0ͣת��1��ת��2��ת��3��ͣ
void set_A(uint16_t i)//����
{
    switch (i)
    {
        case 0:  // ͣת
            // ֹͣA�������AIN1��AIN2����Ϊ�͵�ƽ�������ת��
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);  // AIN1 �õ�
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);  // AIN2 �õ�
            break;

        case 1:  // ��ת
            // ��ת��AIN1Ϊ�ߵ�ƽ��AIN2Ϊ�͵�ƽ�������ת
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);    // AIN1 �ø�
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);  // AIN2 �õ�
            break;

        case 2:  // ��ת
            // ��ת��AIN1Ϊ�͵�ƽ��AIN2Ϊ�ߵ�ƽ�������ת
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);  // AIN1 �õ�
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);    // AIN2 �ø�
            break;

        case 3:  // ��ͣ
            // ��ͣ��AIN1��AIN2����Ϊ�ߵ�ƽ���������ֹͣ
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);    // AIN1 �ø�
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);    // AIN2 �ø�
            break;
    }
}



// ����B���������ת��0ͣת��1��ת��2��ת��3��ͣ
void set_B(uint16_t i)//����
{
    switch (i)
    {
        case 0:  // ͣת
            // ֹͣB�������BIN1��BIN2����Ϊ�͵�ƽ�������ת��
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);  // BIN1 �õ�
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);  // BIN2 �õ�
            break;

        case 1:  // ��ת
            // ��ת��BIN1Ϊ�ߵ�ƽ��BIN2Ϊ�͵�ƽ�������ת
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);    // BIN1 �ø�
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);  // BIN2 �õ�
            break;

        case 2:  // ��ת
            // ��ת��BIN1Ϊ�͵�ƽ��BIN2Ϊ�ߵ�ƽ�������ת
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);  // BIN1 �õ�
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);    // BIN2 �ø�
            break;

        case 3:  // ��ͣ
            // ��ͣ��BIN1��BIN2����Ϊ�ߵ�ƽ���������ֹͣ
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);    // BIN1 �ø�
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);    // BIN2 �ø�
            break;
    }
}

// ֱ�Ӹ�A��������ٶȣ���Χ��0-999��
// ֱ�Ӹ�A��������ٶȣ���Χ��-999��999������ֵ��ʾ��ת����ֵ��ʾ��ת
void set_speed_A(int16_t ccr)  // �޸�ccrΪint16_t���ͣ�֧�ָ�ֵ
{
    // ����ccr�ķ�Χ��ȷ�����ֵΪ999����СֵΪ-999
    if (ccr > 999)
        ccr = 999;
    if (ccr < -999)
        ccr = -999;

    if (ccr >= 0)
    {
      // ��ת����A���
       set_A(1);
        // ��ת������PWMռ�ձ�
        __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, ccr);  // ��TIM12��ͨ��1�������µ�ռ�ձ�

    }
    else
    {
        // ��ת����A���
       set_A(2);
        // ��ת��ccrȡ��ֵ������PWMռ�ձ�
        __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, -ccr);  // ��TIM12��ͨ��1�������µ�ռ�ձȣ���ֵȡ����

      
    }
}

// ֱ�Ӹ�B��������ٶȣ���Χ��-999��999������ֵ��ʾ��ת����ֵ��ʾ��ת
void set_speed_B(int16_t ccr)  // �޸�ccrΪint16_t���ͣ�֧�ָ�ֵ
{
    // ����ccr�ķ�Χ��ȷ�����ֵΪ999����СֵΪ-999
    if (ccr > 999)
        ccr = 999;
    if (ccr < -999)
        ccr = -999;

    if (ccr >= 0)
    {
       // ��ת����B���
        set_B(1);
        // ��ת������PWMռ�ձ�
        __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, ccr);  // ��TIM12��ͨ��2�������µ�ռ�ձ�

       
    }
    else
    {
       // ��ת����B���
        set_B(2);
        // ��ת��ccrȡ��ֵ������PWMռ�ձ�
        __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, -ccr);  // ��TIM12��ͨ��2�������µ�ռ�ձȣ���ֵȡ����

       
    }
}
