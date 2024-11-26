#include "stm32f4xx_hal.h"       // STM32 HAL��ͷ�ļ�
#include "main.h"                 // ������ͷ�ļ�
#include "cmsis_os.h"             // CMSIS-RTOS��ͷ�ļ�
#include "tim.h"                  // ��ʱ�����ͷ�ļ�
#include "gpio.h"                 // GPIO���ͷ�ļ�
#include "st188.h"                // ST188����������ͷ�ļ�
#include "tb6612.h"               // TB6612�����������ͷ�ļ�
#include "encode.h"               // ���������ͷ�ļ�
#include <stdlib.h>               // ���� abs ������ͷ�ļ�
// AB������ٶ�120

// PID����������
float Kp = 0, Ki = 0, Kd = 0;         // PID���������������֡�΢��
float integral = 0.0;                  // ������
float previous_error = 0.0;            // ��һ�������ڼ���΢����
float output = 0.0;                    // PID���
float error = 0;                       // ��ǰ���


uint16_t speed=50;                        // �ٶȣ�������������������
uint16_t speed_left_motor;             // �����ٶ�
uint16_t speed_right_motor;            // �����ٶ�
uint8_t sensor_value = 0;              // ��������ȡֵ�����ڷ�������

// ����PID���Ʋ���
float Kp_speed_left = 0, Ki_speed_left = 0, Kd_speed_left = 0;
float previous_error_speed_left = 0;   // �ϴε����
float integral_speed_left = 0;         // ������
int16_t speed_left_pwm = 0;            // ����PWM���
float speed_left_out = 0;              //����ʵ�ʵĸı���
int16_t error_speed_left = 0;         //����ʵ�����

// ����PID���Ʋ���
float Kp_speed_right = 0, Ki_speed_right = 0, Kd_speed_right = 0;
float previous_error_speed_right = 0;  // �ϴε����
float integral_speed_right = 0;        // ������
int16_t speed_right_pwm = 0;           // ����PWM���
float speed_right_out = 0;             //����ʵ�ʵĸı���
int16_t error_speed_right = 0;         //����ʵ�����

int16_t current_speed = 0;             // ��ǰ�ٶȣ���λ��������/ms��
int16_t error_speed = 0;               // �ٶ����
extern int16_t car_current_speed_left;
extern int16_t car_current_speed_right; // �����ٶ�

// ��ʼ������������ST188��������GPIO
void st188_init() {
    // ��gpio�����Ѿ�������ST188�������ĳ�ʼ��
}

// ��ʼ��PID������
void PID_Init(float kp, float ki, float kd) {
    Kp = kp;  // ���ñ���ϵ��
    Ki = ki;  // ���û���ϵ��
    Kd = kd;  // ����΢��ϵ��

}

// ��ʼ�������ٶȻ�PID������
void PID_Init_Speed_left(float kp, float ki, float kd) {
    Kp_speed_left = kp;       // ���������ٶȻ�����ϵ��
    Ki_speed_left = ki;       // ���������ٶȻ�����ϵ��
    Kd_speed_left = kd;       // ���������ٶȻ�΢��ϵ��
}

// ��ʼ�������ٶȻ�PID������
void PID_Init_Speed_right(float kp, float ki, float kd) {
    Kp_speed_right = kp;      // ���������ٶȻ�����ϵ��
    Ki_speed_right = ki;      // ���������ٶȻ�����ϵ��
    Kd_speed_right = kd;      // ���������ٶȻ�΢��ϵ��
}

// ����PID������
void PID_Update(int16_t error) {
    // ��������΢����
    float derivative = error - previous_error; 
    // �ۼ��������
    integral += error; 
    // ����PID���
    output = Kp * error + Ki * integral + Kd * derivative; 
    // ����ǰһ�����
    previous_error = error; 
}

// ��������PID������
void PID_Update_Speed_left(int16_t error_speed_left) {
    // ������������΢����
    float derivative_speed_left = error_speed_left - previous_error_speed_left;
    // �ۼ������������
    integral_speed_left += error_speed_left;
    // ��������ǰһ�ε����
    previous_error_speed_left = error_speed_left;
    // ��������PID���
    speed_left_out = Kp_speed_left * error_speed_left + Ki_speed_left * integral_speed_left + Kd_speed_left * derivative_speed_left;

}

// ��������PID������
void PID_Update_Speed_right(int16_t error_speed_right) {
    // ������������΢����
    float derivative_speed_right = error_speed_right - previous_error_speed_right;
    // �ۼ������������
    integral_speed_right += error_speed_right;
    // ��������ǰһ�ε����
    previous_error_speed_right = error_speed_right;
    // ��������PID���
    speed_right_out = Kp_speed_right * error_speed_right + Ki_speed_right * integral_speed_right + Kd_speed_right * derivative_speed_right;
  
}

// ���������ٶȵ�PID����
void pid_speed_control_left(int8_t target_speed) {
    // ��ȡ��ǰʵ���ٶ�
    get_left_wheel_speed_and_distance();

    // ����Ŀ���ٶ��뵱ǰ�ٶ�֮������
    error_speed_left = target_speed - car_current_speed_left;

    // �����ٶȻ�PID������
    PID_Update_Speed_left(error_speed_left);

    // ����PID��������ֵ�����ⳬ�����������ٶ�
    speed_left_pwm =speed_left_pwm+ speed_left_out;  // ���ֵ�Ŀ���ٶ�
    if (speed_left_pwm > 999) {
        speed_left_pwm = 999;
    } else if (speed_left_pwm < -999) {
        speed_left_pwm = -999;
    }

    // ��������ٶ�
    set_speed_B(speed_left_pwm);
}

// ���������ٶȵ�PID����
void pid_speed_control_right(int8_t target_speed) {
    // ��ȡ��ǰʵ���ٶ�
    get_right_wheel_speed_and_distance(); // ֻ�����ֵ�

    // ����Ŀ���ٶ��뵱ǰ�ٶ�֮������
    error_speed_right = target_speed - car_current_speed_right;

    // �����ٶȻ�PID������
    PID_Update_Speed_right(error_speed_right);

    // ����PID��������ֵ�����ⳬ�����������ٶ�
    speed_right_pwm += speed_right_out;  // ���ֵ�Ŀ���ٶ�
     if (speed_right_pwm > 800) {
        speed_right_pwm = 800;
    } else if (speed_right_pwm < -800) {
        speed_right_pwm = -800;
    }
    set_speed_A(speed_right_pwm);
}

// PID���㺯�� - ѭ��ģ��ǶȻ�
void PID_Compute(void) {
    error = 0;  // ��ʼ�����

    // ��ȡST188��·�����������ź�
    sensor_value = 0;
    sensor_value |= HAL_GPIO_ReadPin(st188_1_GPIO_Port, st188_1_Pin) << 0; // S1 -> bit0
    sensor_value |= HAL_GPIO_ReadPin(st188_2_GPIO_Port, st188_2_Pin) << 1; // S2 -> bit1
    sensor_value |= HAL_GPIO_ReadPin(st188_3_GPIO_Port, st188_3_Pin) << 2; // S3 -> bit2
    sensor_value |= HAL_GPIO_ReadPin(st188_4_GPIO_Port, st188_4_Pin) << 3; // S4 -> bit3
    sensor_value |= HAL_GPIO_ReadPin(st188_5_GPIO_Port, st188_5_Pin) << 4; // S5 -> bit4
    sensor_value |= HAL_GPIO_ReadPin(st188_6_GPIO_Port, st188_6_Pin) << 5; // S6 -> bit5
    sensor_value |= HAL_GPIO_ReadPin(st188_7_GPIO_Port, st188_7_Pin) << 6; // S7 -> bit6
    sensor_value |= HAL_GPIO_ReadPin(st188_8_GPIO_Port, st188_8_Pin) << 7; // S8 -> bit7

    // ���ݴ�����ֵ�ж����
    switch (sensor_value) {
        case 0x80: error = -4.0; break; // S1 -> 10000000
        case 0x40: error = -3.0; break; // S2 -> 01000000
        case 0x20: error = -2.0; break; // S3 -> 00100000
        case 0x10: error = -1.0; break; // S4 -> 00010000
        case 0x08: error = 1.0; break;  // S5 -> 00001000
        case 0x04: error = 2.0; break;  // S6 -> 00000100
        case 0x02: error = 3.0; break;  // S7 -> 00000010
        case 0x01: error = 4.0; break;  // S8 -> 00000001

        // �������ڴ�����������
        case 0xC0: error = -3.5; break; // S1, S2 -> 11000000
        case 0x60: error = -2.5; break; // S2, S3 -> 01100000
        case 0x30: error = -1.5; break; // S3, S4 -> 00110000
        case 0x18: error = 0.0; break;  // S4, S5 -> 00011000
        case 0x0C: error = 1.5; break;  // S5, S6 -> 00001100
        case 0x06: error = 2.5; break;  // S6, S7 -> 00000110
        case 0x03: error = 3.5; break;  // S7, S8 -> 00000011

        // ��������������ߣ���켣��
        case 0xE0: error = -3.0; break; // S1, S2, S3 -> 11100000
        case 0x70: error = -1.0; break; // S2, S3, S4 -> 01110000
        case 0x38: error = 0.0; break;  // S3, S4, S5 -> 00111000
        case 0x1C: error = 1.0; break;  // S4, S5, S6 -> 00011100
        case 0x0E: error = 3.0; break;  // S5, S6, S7 -> 00001110

        // ���������ȫ���߻����ź�
        case 0xFF: error = 0.0; break;  // ȫ���ߣ����ֵ�ǰ����
        case 0x00: error = previous_error; break; // ���źţ�����ƫ����

        // Ĭ�ϴ���
        default: error = 0.0; break;
    }

    // ����PID������
    PID_Update(error);
    
    // ����С�����ٶȣ�����PID���ֵ��
    int16_t speed_right = speed + (int16_t)output;
    int16_t speed_left = speed - (int16_t)output;
    
//    // ���õ��������ת��ת��
//    int16_t speed_right_motor = abs(speed_right);
//    int16_t speed_left_motor = abs(speed_left);
//    
    // �������ֺ����ֵķ���1��ת��2��ת��
//    set_A(speed_left > 0 ? 1 : 2); // ���ַ���
//    set_B(speed_right > 0 ? 1 : 2); // ���ַ���

    // �����ٶȷ�Χ������ٶ��趨Ϊ80��
    if (speed_right > 100) speed_right = 100;
    else if(speed_right <-100) speed_right = -100;
    
    if (speed_left > 100) speed_left = 100;
    else if (speed_left <-100) speed_left = -100;
    // ���õ���ٶ�
    pid_speed_control_right(speed_right);
    pid_speed_control_left(speed_left);
}
