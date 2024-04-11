#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"	
#include "timer.h"
#include "usart2.h"
#include "mlx90614.h" 
#include "bsp_tm7705.h"
#include "bsp_ads1115.h"


/*************���ܿ����궨��**************/
//#define DETECT_CUR
//#define DETECT_VOL
#define DETECT_VOL_NEW
//#define DETECT_TEMP
//#define DETECT_SPEED
//#define DETECT_MESSAGE_TRANSFER_ON
/*****************************************/



#define NUM_COM         1                     //COM�Ŀ�������
#define NUM_DETECT_TEMP NUM_COM /*2*/         //��������¶�����
#define NUM_DETECT_VOL  NUM_COM
#define NUM_DETECT_CUR  NUM_COM


int main(void)
{
//#ifdef DETECT_CUR
//	float Current = 0;
//#endif

	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	uart2_init(115200);	 	//���ڳ�ʼ��Ϊ115200
 	LED_Init();			     //LED�˿ڳ�ʼ��
	TIM3_Int_Init(99,7199);//10Khz�ļ���Ƶ�ʣ�������100Ϊ10ms  
 	
#ifdef DETECT_TEMP
	SMBus_Init();           //SMBus���߳�ʼ��
#endif

#if defined(DETECT_VOL) || defined(DETECT_CUR)
	TM7705_Init(NUM_COM);		    // ��ʼ������TM7705
#endif

#ifdef DETECT_VOL_NEW
    ads1115_Init();
#endif
    
	//SMBus_EditSlaveAddre(MLX90614_SA2);   //�趨�¶ȴ������Ĵӻ���ַ

    delay_ms(2000);


	while(1)
	{
#ifdef DETECT_TEMP
		Get_Temp(NUM_DETECT_TEMP);          //��ò���ӡ�¶�ֵ
#endif

#ifdef DETECT_VOL
		Get_Vol(NUM_DETECT_VOL);            //��ò���ӡ��ѹֵ
#endif        

#ifdef DETECT_CUR
        Get_Cur(NUM_DETECT_CUR);            //��ò���ӡ��ѹֵ
#endif

#ifdef DETECT_MESSAGE_TRANSFER_ON           //ͨ������1�����Ϣ���ɿ�
       //
#endif
        //Get_Vol_ads1115(0);
        //Get_Vol_ads1115(1);
        //Get_Vol_ads1115(2);
        //Get_Vol_ads1115(3);

        //delay_ms(500);

		//printf("halo\r\n");
		//printf2("usart2 printf2\r\n");
		led_err();                     //����ָʾ��
	}
    
}


//void Detect_Message_Transfer(int)

