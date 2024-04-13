#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "sys.h"
#include "bsp_ads1110.h"
#include "usart.h"
#include "timer.h"


int main(void)
{
	delay_init();									// ��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // �����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);								// ���ڳ�ʼ��Ϊ115200
	ads1110_I2C_INIT();
	TIM3_Int_Init(99, 7199);						// 10Khz�ļ���Ƶ�ʣ�������100Ϊ10ms
	
	while(1){
		
		Get_Vol_ads1110();
		
	}
}
