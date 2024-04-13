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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����2��ʼ��		   
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
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
     
//����_sys_exit()�Ա���ʹ�ð�����ģʽ 

struct __FILE
{
	int handle;
 
};
FILE __stdout2;

_sys_exit2(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc2(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
    USART2->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
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
 
#if EN_USART2_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
char USART2_RX_BUF[USART2_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART2_RX_STA=0;       //����״̬���	  
  
void uart2_init(u32 bound){
  //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART2_TX   GPIOA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.2
   
    //USART2_RX	  GPIOA.3��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//��λ����2
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//ֹͣ��λ
  
    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART2, &USART_InitStructure); //��ʼ������2

    //Usart2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2
}

void USART2_IRQHandler(void)                	//����2�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
	    Res =USART_ReceiveData(USART2);	//��ȡ���յ�������
	
	    if((USART2_RX_STA&0x8000)==0)//����δ���
		{
    		if(USART2_RX_STA&0x4000)//���յ���0x0d
    		{
    			if(Res!=0x0a)USART2_RX_STA=0;//���մ���,���¿�ʼ
    			else 
                {
                    USART2_RX_STA|=0x8000;  //��������� 
                    Rec_Process();
    			}
    		}
    		else //��û�յ�0X0D
    		{	
    			if(Res==0x0d)USART2_RX_STA|=0x4000;
    			else
    			{
    				USART2_RX_BUF[USART2_RX_STA&0X3FFF]=Res ;
    				USART2_RX_STA++;
    				if(USART2_RX_STA>(USART2_REC_LEN-1))USART2_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
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
*   �� �� ��: Command_MotoSpeed_Set_Freq
*   ����˵��: ����ת�ٷ���Ƶ��/hz
*   ��    ��:  
*   �� �� ֵ: 
**********************************************************************************************************/
void Command_MotoSpeed_Set_Freq(int Hz)
{
    printf2("set_freq=%3dHz\r\n", Hz);
    Moto_speed_CTRL.Flag_Set_Freq = Moto_Setting_ASKING;
}

/**********************************************************************************************************
*   �� �� ��: Command_MotoSpeed_Set_Moto_Num
*   ����˵��: ���õ������
*   ��    ��:  
*   �� �� ֵ: 
**********************************************************************************************************/
void Command_MotoSpeed_Set_Moto_Num(uint8_t moto_num)
{
    printf2("set_moto_num=%2d\r\n", moto_num);
    Moto_speed_CTRL.Flag_Set_Moto_Num = Moto_Setting_ASKING;
}

/**********************************************************************************************************
*   �� �� ��: Command_MotoSpeed_Detect_EN
*   ����˵��: ������ֹͣת�ټ��
*   ��    ��:  
*   �� �� ֵ: 
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
*   �� �� ��: Command_MotoSpeed_Detect_MotoX
*   ����˵��: ����ĳһ���ת��ֵ
*   ��    ��:  
*   �� �� ֵ: 
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

    //������
    Rec_FAIL_Process(USART2_RX_BUF_temp);

    //��Ϣƥ��
    Rec_NOR_Process(USART2_RX_BUF_temp);
    
    
    USART2_RX_STA=0;
}


//��Ϣƥ��
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
    
    //����Moto_speed_CTRL.Moto_Num �ĵ��������ö�Ӧ�����ĵ��ת�����ݣ����浽Moto_speed�ṹ����
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


//������
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

