#include "sys.h"
#include "usart2.h"	  	 
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#define __DEBUG_U2__

#ifdef __DEBUG_U2__
    #define PRINT_U2(...)       printf2(__VA_ARGS__);
    #define PRINT_U2_ERR(...)   printf2(__VA_ARGS__);led_err();
#else
    #define PRINT_U2(...)       //printf2(__VA_ARGS__)
    #define PRINT_U2_ERR(...)   //printf2(__VA_ARGS__);
#endif

#define NUM_STR_Res_Moto_Speed          10

#define NUM_STR_Res_Set_Freq            13
#define NUM_STR_Res_Set_Moto_Num        17
#define NUM_STR_Res_Detect_En           14
#define NUM_STR_Res_Detect_Moto         16
#define NUM_STR_Fail_Set_Freq           13
#define NUM_STR_Fail_Set_Moto_Num       13
#define NUM_STR_Fail_Detect_En          14
#define NUM_STR_Fail_Detect_Moto        16

#define NUM_STR_SPEED_MOTO1             4+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO2             12+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO3             20+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO4             28+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO5             36+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO6             44+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO7             52+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO8             60+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO9             68+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO10            76+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO11            84+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO12            92+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO13           100+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO14           108+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO15           116+NUM_STR_Res_Moto_Speed
#define NUM_STR_SPEED_MOTO16           124+NUM_STR_Res_Moto_Speed

char * STR_Res_Moto_Speed     = "moto_speed:";

char * STR_Res_Set_Freq       = "res_set_freq=";
char * STR_Res_Set_Moto_Num   = "res_set_moto_num=";
char * STR_Res_Detect_En      = "res_detect_en=";
char * STR_Res_Detect_Moto    = "res_detect_moto=";

char * STR_Fail_Set_Freq      = "fail_set_freq";
char * STR_Fail_Set_Moto_Num  = "fail_set_moto";
char * STR_Fail_Detect_En     = "fail_en_detect";
char * STR_Fail_Detect_Moto   = "fail_detect_moto";



struct Moto_speed_s         Moto_speed;
struct Moto_speed_CTRL_s    Moto_speed_CTRL;


//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口2初始化		   
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
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
     
//定义_sys_exit()以避免使用半主机模式 

struct __FILE
{
	int handle;
 
};
FILE __stdout2;

_sys_exit2(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc2(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
    USART2->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART2_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
char USART2_RX_BUF[USART2_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART2_RX_STA=0;       //接收状态标记	  
  
void uart2_init(u32 bound){
  //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART2_TX   GPIOA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.2
   
    //USART2_RX	  GPIOA.3初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//复位串口2
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//停止复位
  
    //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART2, &USART_InitStructure); //初始化串口2

    //Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART2, ENABLE);                    //使能串口2
}

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
	    Res =USART_ReceiveData(USART2);	//读取接收到的数据
	
	    if((USART2_RX_STA&0x8000)==0)//接收未完成
		{
    		if(USART2_RX_STA&0x4000)//接收到了0x0d
    		{
    			if(Res!=0x0a)USART2_RX_STA=0;//接收错误,重新开始
    			else 
                {
                    USART2_RX_STA|=0x8000;  //接收完成了 
                    Rec_Process();
    			}
    		}
    		else //还没收到0X0D
    		{	
    			if(Res==0x0d)USART2_RX_STA|=0x4000;
    			else
    			{
    				USART2_RX_BUF[USART2_RX_STA&0X3FFF]=Res ;
    				USART2_RX_STA++;
    				if(USART2_RX_STA>(USART2_REC_LEN-1))USART2_RX_STA=0;//接收数据错误,重新开始接收	  
    			}		 
    		}
		}   		 
     } 
} 


void printf2(char * fmt,...)
{
    char buffer[100];
    uint16_t i = 0;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buffer, 100, fmt, arg_ptr);

    while(i < 100&& buffer[i])
    {
        while((USART2->SR & 0X40) == 0);
        USART_SendData(USART2, buffer[i]);
        i++;
    }
    va_end(arg_ptr);
}

/**********************************************************************************************************
*   函 数 名: Command_MotoSpeed_Set_Freq
*   功能说明: 设置转速发送频率/hz
*   形    参:  
*   返 回 值: 
**********************************************************************************************************/
void Command_MotoSpeed_Set_Freq(int Hz)
{
    printf2("set_freq=%3dHz\r\n", Hz);
    Moto_speed_CTRL.Flag_Set_Freq = Moto_Setting_ASKING;
}

/**********************************************************************************************************
*   函 数 名: Command_MotoSpeed_Set_Moto_Num
*   功能说明: 设置电机个数
*   形    参:  
*   返 回 值: 
**********************************************************************************************************/
void Command_MotoSpeed_Set_Moto_Num(uint8_t moto_num)
{
    printf2("set_moto_num=%2d\r\n", moto_num);
    Moto_speed_CTRL.Flag_Set_Moto_Num = Moto_Setting_ASKING;
}

/**********************************************************************************************************
*   函 数 名: Command_MotoSpeed_Detect_EN
*   功能说明: 开启、停止转速监测
*   形    参:  
*   返 回 值: 
**********************************************************************************************************/
void Command_MotoSpeed_Detect_EN(uint8_t status)
{
    if(status == Moto_Speed_Detect_Start)
    {
        printf2("detect_en=start\r\n");
        Moto_speed_CTRL.Flag_Detect_EN = Moto_Setting_ASKING;
    }
    else if(status == Moto_Speed_Detect_Stop)
    {
        printf2("detect_en=stop\r\n");
        Moto_speed_CTRL.Flag_Detect_EN = Moto_Setting_ASKING;
    }
}

/**********************************************************************************************************
*   函 数 名: Command_MotoSpeed_Detect_MotoX
*   功能说明: 返回某一电机转速值
*   形    参:  
*   返 回 值: 
**********************************************************************************************************/
void Command_MotoSpeed_Detect_MotoX(uint8_t moto_index)
{
    printf2("detect_moto=%2d\r\n", moto_index);
    Moto_speed_CTRL.Flag_Detect_Motox = Moto_Setting_ASKING;
}



void Rec_Process(void)
{
    int rec_len = USART2_RX_STA&0x3f;
    char USART2_RX_BUF_temp[USART2_REC_LEN];
    
    //printf2("rec_len=%d",rec_len);

    strncpy(USART2_RX_BUF_temp, USART2_RX_BUF, rec_len);
    memset(USART2_RX_BUF, 0, sizeof(USART2_RX_BUF));

    //错误处理
    Rec_FAIL_Process(USART2_RX_BUF_temp);

    //信息匹配
    Rec_NOR_Process(USART2_RX_BUF_temp);
    
    
    USART2_RX_STA=0;
}


//信息匹配
void Rec_NOR_Process(char *USART2_RX_BUF_temp)
{   
    int i;
    int temp;
    
    if(0 == strncmp(USART2_RX_BUF_temp, STR_Res_Set_Freq, NUM_STR_Res_Set_Freq))
    {
        temp = (USART2_RX_BUF_temp[NUM_STR_Res_Set_Freq]-'0')*100+(USART2_RX_BUF_temp[NUM_STR_Res_Set_Freq+1]-'0')*10+(USART2_RX_BUF_temp[NUM_STR_Res_Set_Freq+2]-'0');
        Moto_speed_CTRL.Send_Freq     = temp;
        Moto_speed_CTRL.Flag_Set_Freq = Moto_Setting_IDLE;
        printf2("Set_Freq=%d\r\n", temp);
    }else
        PRINT_U2("cmp fail Set_Freq\r\n");

    if(0 == strncmp(USART2_RX_BUF_temp, STR_Res_Set_Moto_Num, NUM_STR_Res_Set_Moto_Num))
    {
        temp = (USART2_RX_BUF_temp[NUM_STR_Res_Set_Moto_Num]-'0')*10+(USART2_RX_BUF_temp[NUM_STR_Res_Set_Moto_Num+1]-'0');
        Moto_speed_CTRL.Moto_Num          = temp;
        Moto_speed_CTRL.Flag_Set_Moto_Num = Moto_Setting_IDLE;
        printf2("Set_Moto_Num=%d\r\n", temp);
    }else
        PRINT_U2("cmp fail Set_Moto_Num\r\n");

    if(0 == strncmp(USART2_RX_BUF_temp, STR_Res_Detect_En, NUM_STR_Res_Detect_En))
    {
        if(0 == strncmp(USART2_RX_BUF_temp+NUM_STR_Res_Detect_En, "start", 5))
        {
             printf2("Detect_En=start\r\n");
             Moto_speed_CTRL.Detect_EN      = Moto_Speed_Detect_Start;
             Moto_speed_CTRL.Flag_Detect_EN = Moto_Setting_IDLE;
        }
        else if(0 == strncmp(USART2_RX_BUF_temp+NUM_STR_Res_Detect_En, "stop", 4))
        {
             printf2("Detect_En=stop\r\n");
             Moto_speed_CTRL.Detect_EN      = Moto_Speed_Detect_Stop;
             Moto_speed_CTRL.Flag_Detect_EN = Moto_Setting_IDLE;
        }
        else 
        {
            PRINT_U2_ERR("Detect_En none\r\n");
            Moto_speed_CTRL.Flag_Detect_EN = Moto_Setting_ERR;
        }
    }
    else 
        PRINT_U2("cmp fail Detect_En\r\n");

    if(0 == strncmp(USART2_RX_BUF_temp, STR_Res_Detect_Moto, NUM_STR_Res_Detect_Moto))
    {
        temp = (USART2_RX_BUF_temp[NUM_STR_Res_Detect_Moto]-'0')*10+(USART2_RX_BUF_temp[NUM_STR_Res_Detect_Moto+1]-'0');
        Moto_speed_CTRL.Speed_MOtox       = temp;
        Moto_speed_CTRL.Flag_Detect_Motox = Moto_Setting_IDLE;
        printf2("Detect_Moto=%d\r\n", temp);
    }else
        PRINT_U2("cmp fail Detect_Moto\r\n");
    
    //根据Moto_speed_CTRL.Moto_Num 的电机数，获得对应数量的电机转速数据，保存到Moto_speed结构体中
    if(0 == strncmp(USART2_RX_BUF_temp, STR_Res_Moto_Speed, NUM_STR_Res_Moto_Speed))
    {
        switch (Moto_speed_CTRL.Moto_Num)
        {
            case 16: Moto_speed.MOTO16 = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO16);
            case 15: Moto_speed.MOTO15 = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO15);
            case 14: Moto_speed.MOTO14 = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO14);
            case 13: Moto_speed.MOTO13 = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO13);
            case 12: Moto_speed.MOTO12 = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO12);
            case 11: Moto_speed.MOTO11 = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO11);
            case 10: Moto_speed.MOTO10 = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO10);
            case 9 : Moto_speed.MOTO9  = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO9);
            case 8 : Moto_speed.MOTO8  = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO8);
            case 7 : Moto_speed.MOTO7  = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO7);
            case 6 : Moto_speed.MOTO6  = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO6);
            case 5 : Moto_speed.MOTO5  = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO5);
            case 4 : Moto_speed.MOTO4  = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO4);
            case 3 : Moto_speed.MOTO3  = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO3);
            case 2 : Moto_speed.MOTO2  = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO2);
            case 1 : Moto_speed.MOTO1  = Conv_String_to_int(USART2_RX_BUF_temp, NUM_STR_SPEED_MOTO1);
            default: break;
        }
        //printf2("moto1 = %d  moto2 = %d  moto3 = %d  moto4 = %d  moto5 = %d  \r\n", Moto_speed.MOTO1, \
        //                        Moto_speed.MOTO2, Moto_speed.MOTO3, Moto_speed.MOTO4, Moto_speed.MOTO5);
    }

}


//错误处理
void Rec_FAIL_Process(char *USART2_RX_BUF_temp)
{
    int temp;
    
    if(0 == strncmp(USART2_RX_BUF_temp, STR_Fail_Set_Freq, NUM_STR_Fail_Set_Freq))
    {
        Moto_speed_CTRL.Flag_Set_Freq = Moto_Setting_ERR;
        PRINT_U2_ERR("%s\r\n", STR_Fail_Set_Freq);
    }

    if(0 == strncmp(USART2_RX_BUF_temp, STR_Fail_Set_Moto_Num, NUM_STR_Fail_Set_Moto_Num))
    {
        Moto_speed_CTRL.Flag_Set_Moto_Num = Moto_Setting_ERR;
        PRINT_U2_ERR("%s\r\n", STR_Fail_Set_Moto_Num);
    }

    if(0 == strncmp(USART2_RX_BUF_temp, STR_Fail_Detect_En, NUM_STR_Fail_Detect_En))
    {
        Moto_speed_CTRL.Flag_Detect_EN = Moto_Setting_ERR;
        PRINT_U2_ERR("%s\r\n", STR_Fail_Detect_En);
    }

    if(0 == strncmp(USART2_RX_BUF_temp, STR_Fail_Detect_Moto, NUM_STR_Fail_Detect_Moto))
    {
        Moto_speed_CTRL.Flag_Detect_Motox = Moto_Setting_ERR;
        temp = (USART2_RX_BUF_temp[NUM_STR_Fail_Detect_Moto]-'0')*10+(USART2_RX_BUF_temp[NUM_STR_Fail_Detect_Moto+1]-'0');
        PRINT_U2_ERR("%s%2d\r\n", STR_Fail_Detect_Moto,temp);
    }
}


int Conv_String_to_int(char *USART2_RX_BUF_temp, uint8_t index)
{
    int temp = (USART2_RX_BUF_temp[index]-'0')*1000+(USART2_RX_BUF_temp[index+1]-'0')*100+(USART2_RX_BUF_temp[index+2]-'0')*10+(USART2_RX_BUF_temp[index+3]-'0');;
    //printf2("Conv_String_to_int  index=%d, 4:%d 3:%d 2:%d 1:%d\r\n",index,(USART2_RX_BUF_temp[index]-'0'),(USART2_RX_BUF_temp[index+1]-'0'),(USART2_RX_BUF_temp[index+2]-'0'),(USART2_RX_BUF_temp[index+3]-'0'));
    //printf2("Conv_String_to_int temp = %d \r\n", temp);
    return temp;
}

#endif	

