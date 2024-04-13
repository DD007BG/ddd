#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "sys.h"
#include "bsp_ads1110.h"
#include "usart.h"
#include "timer.h"


int main(void)
{
	delay_init();									// 延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);								// 串口初始化为115200
	ads1110_I2C_INIT();
	TIM3_Int_Init(99, 7199);						// 10Khz的计数频率，计数到100为10ms
	
	while(1){
		
		Get_Vol_ads1110();
		
	}
}
