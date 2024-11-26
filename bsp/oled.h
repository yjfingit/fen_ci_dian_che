#ifndef __OLED_H
#define __OLED_H 


#include "stdlib.h"	
#include "main.h"

#define Your_JingZhen_Pinlu 168  // �����Լ��ľ���Ƶ������


//   �ڱ�׼�ߴ��£����ֶ�Ӧ��λ��////////////////////////////////////////
#define BZ_HZ 	16

#define L_X1 	0
#define L_X2	BZ_HZ*1
#define L_X3 	BZ_HZ*2
#define L_X4 	BZ_HZ*3
#define L_X5 	BZ_HZ*4
#define L_X6 	BZ_HZ*5
#define L_X7 	BZ_HZ*6
#define L_X8 	BZ_HZ*7

#define H_Y1 	0
#define H_Y2	BZ_HZ*1
#define H_Y3 	BZ_HZ*2
#define H_Y4 	BZ_HZ*3
////////////////////////����Ϊֹ//////////////////////////////////////////
#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif
//-----------------OLED�˿ڶ���---------------- 

#define OLED_SCL_Clr()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
//SCL
#define OLED_SCL_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)

#define OLED_SDA_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)//DIN
#define OLED_SDA_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
//#define OLED_RES_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_4)//RES
//#define OLED_RES_Set() GPIO_SetBits(GPIOD,GPIO_Pin_4)


#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

void OLED_ClearPoint(u8 x,u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WaitAck(void);
void Send_Byte(u8 dat);
void OLED_WR_Byte(u8 dat,u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
void OLED_ShowNumWithSign(u8 x, u8 y, int num, u8 len, u8 size1, u8 mode);
void OLED_ShowBinary(u8 x, u8 y, u32 num, u8 len, u8 size1, u8 mode);
void OLED_ShowFloat(u8 x, u8 y, float num, u8 len, u8 size1, u8 mode);
u8 OLED_NumWidth(u32 num);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,const unsigned char BMP[],u8 mode);
void OLED_Init(void);

#endif

