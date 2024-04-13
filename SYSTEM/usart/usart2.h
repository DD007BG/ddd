#ifndef __USART2_H
#define __USART2_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
#define USART2_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART2_RX 			    1		//ʹ�ܣ�1��/��ֹ��0������1����



/*�洢���ת����Ϣ�ṹ��*/
struct Moto_speed_s
{
    int MOTO1;
    int MOTO2;
    int MOTO3;
    int MOTO4;
    int MOTO5;
    int MOTO6;
    int MOTO7;
    int MOTO8;
    int MOTO9;
    int MOTO10;
    int MOTO11;
    int MOTO12;
    int MOTO13;
    int MOTO14;
    int MOTO15;
    int MOTO16;
};

/*Flagxxx��ʾ�����mcu���͸�ת�ټ��mcu�ı�־λ����������������1����ö�Ӧ�������ֵ��
  ��0��ͬʱ��������浽��Ӧ�Ľṹ�������
  */
struct Moto_speed_CTRL_s
{
    uint8_t Flag_Set_Freq;
    int     Send_Freq;
    uint8_t Flag_Set_Moto_Num;
    uint8_t Moto_Num;
    uint8_t Flag_Detect_EN;
    uint8_t Detect_EN;
    uint8_t Flag_Detect_Motox;
    uint8_t Speed_MOtox;
};

enum
{
    Moto_Setting_ERR    = 2,            /*�ѷ��������Ӧfail*/
    Moto_Setting_ASKING = 1,            /*�ѷ�������ȴ���Ӧ��*/
    Moto_Setting_IDLE   = 0,            /*���յ���������ٴη���*/
    
};

enum
{
	Moto_Speed_Detect_Start,	/* ����ת�ټ�� */
	Moto_Speed_Detect_Stop	    /* ֹͣת�ټ�� */
};


	  	
extern char  USART2_RX_BUF[USART2_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART2_RX_STA;         		//����״̬���	
extern struct Moto_speed_s Moto_speed;


extern char * STR_Res_Set_Freq;
extern char * STR_Res_Set_Moto_Num;
extern char * STR_Res_Detect_En;
extern char * STR_Res_Detect_Moto;

extern char * STR_Fail_Set_Freq;
extern char * STR_Fail_Set_Moto_Num;
extern char * STR_Fail_Detect_En;
extern char * STR_Fail_Detect_Moto;


//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart2_init(u32 bound);
void Rec_Process(void);
void Rec_NOR_Process(char *USART2_RX_BUF_temp);
void Rec_FAIL_Process(char *USART2_RX_BUF_temp);
int Conv_String_to_int(char *USART2_RX_BUF_temp, uint8_t index);


#endif


