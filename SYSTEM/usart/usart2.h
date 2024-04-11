#ifndef __USART2_H
#define __USART2_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
#define USART2_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART2_RX 			    1		//使能（1）/禁止（0）串口1接收



/*存储电机转速信息结构体*/
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

/*Flagxxx表示主监测mcu发送给转速监测mcu的标志位，当发送命令后会置1，获得对应的命令返回值后
  置0，同时将结果保存到对应的结构体变量中
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
    Moto_Setting_ERR    = 2,            /*已发送命令，回应fail*/
    Moto_Setting_ASKING = 1,            /*已发送命令，等待回应中*/
    Moto_Setting_IDLE   = 0,            /*已收到命令，可以再次发送*/
    
};

enum
{
	Moto_Speed_Detect_Start,	/* 开启转速监测 */
	Moto_Speed_Detect_Stop	    /* 停止转速监测 */
};


	  	
extern char  USART2_RX_BUF[USART2_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART2_RX_STA;         		//接收状态标记	
extern struct Moto_speed_s Moto_speed;


extern char * STR_Res_Set_Freq;
extern char * STR_Res_Set_Moto_Num;
extern char * STR_Res_Detect_En;
extern char * STR_Res_Detect_Moto;

extern char * STR_Fail_Set_Freq;
extern char * STR_Fail_Set_Moto_Num;
extern char * STR_Fail_Detect_En;
extern char * STR_Fail_Detect_Moto;


//如果想串口中断接收，请不要注释以下宏定义
void uart2_init(u32 bound);
void Rec_Process(void);
void Rec_NOR_Process(char *USART2_RX_BUF_temp);
void Rec_FAIL_Process(char *USART2_RX_BUF_temp);
int Conv_String_to_int(char *USART2_RX_BUF_temp, uint8_t index);


#endif


