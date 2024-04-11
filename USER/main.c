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


/*************功能开启宏定义**************/
//#define DETECT_CUR
//#define DETECT_VOL
#define DETECT_VOL_NEW
//#define DETECT_TEMP
//#define DETECT_SPEED
//#define DETECT_MESSAGE_TRANSFER_ON
/*****************************************/



#define NUM_COM         1                     //COM的开启数量
#define NUM_DETECT_TEMP NUM_COM /*2*/         //监测电机的温度数量
#define NUM_DETECT_VOL  NUM_COM
#define NUM_DETECT_CUR  NUM_COM


int main(void)
{
//#ifdef DETECT_CUR
//	float Current = 0;
//#endif

	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
	uart2_init(115200);	 	//串口初始化为115200
 	LED_Init();			     //LED端口初始化
	TIM3_Int_Init(99,7199);//10Khz的计数频率，计数到100为10ms  
 	
#ifdef DETECT_TEMP
	SMBus_Init();           //SMBus总线初始化
#endif

#if defined(DETECT_VOL) || defined(DETECT_CUR)
	TM7705_Init(NUM_COM);		    // 初始化配置TM7705
#endif

#ifdef DETECT_VOL_NEW
    ads1115_Init();
#endif
    
	//SMBus_EditSlaveAddre(MLX90614_SA2);   //设定温度传感器的从机地址

    delay_ms(2000);


	while(1)
	{
#ifdef DETECT_TEMP
		Get_Temp(NUM_DETECT_TEMP);          //获得并打印温度值
#endif

#ifdef DETECT_VOL
		Get_Vol(NUM_DETECT_VOL);            //获得并打印电压值
#endif        

#ifdef DETECT_CUR
        Get_Cur(NUM_DETECT_CUR);            //获得并打印电压值
#endif

#ifdef DETECT_MESSAGE_TRANSFER_ON           //通过串口1输出信息到飞控
       //
#endif
        //Get_Vol_ads1115(0);
        //Get_Vol_ads1115(1);
        //Get_Vol_ads1115(2);
        //Get_Vol_ads1115(3);

        //delay_ms(500);

		//printf("halo\r\n");
		//printf2("usart2 printf2\r\n");
		led_err();                     //运行指示灯
	}
    
}


//void Detect_Message_Transfer(int)

