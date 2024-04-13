#ifndef __MLX90614_H
#define __MLX90614_H
#include "sys.h"
#include "delay.h"
#include "usart.h"

#define ACK	 0
#define	NACK 1  //不应答或否定的应答

/************************************温度传感器从机地址定义*******************************/
#define MLX_90614_SAx_BASE          0xBE00  //修改传感器地址时用到，作为高8位
#define SA				0x20 //从机地址，单个MLX90614时地址为0x00,多个时地址默认为0x5a
#define MLX90614_SA1				0x21
#define MLX90614_SA2				0x22
#define MLX90614_SA3				0x23
#define MLX90614_SA4				0x24
#define MLX90614_SA5				0x25
#define MLX90614_SA6				0x26
#define MLX90614_SA7				0x27
#define MLX90614_SA8				0x28
#define MLX90614_SA9			    0x29
#define MLX90614_SA10			    0x2A
#define MLX90614_SA11	            0x2B
#define MLX90614_SA12		        0x2C
#define MLX90614_SA13	            0x2D
#define MLX90614_SA14		        0x2E
#define MLX90614_SA15		        0x2F
#define MLX90614_SA16			    0x30
/*****************************************************************************************/



#define RAM_ACCESS		0x00 //RAM access command
#define EEPROM_ACCESS	0x20 //EEPROM access command
#define RAM_TOBJ1		0x07 //To1 address in the eeprom

#define SMBUS_PORT	    GPIOB
#define SMBUS_SCK		GPIO_Pin_6
#define SMBUS_SDA		GPIO_Pin_7

#define RCC_APB2Periph_SMBUS_PORT		RCC_APB2Periph_GPIOB

#define SMBUS_SCK_H()	    SMBUS_PORT->BSRR = SMBUS_SCK
#define SMBUS_SCK_L()	    SMBUS_PORT->BRR = SMBUS_SCK
#define SMBUS_SDA_H()	    SMBUS_PORT->BSRR = SMBUS_SDA
#define SMBUS_SDA_L()	    SMBUS_PORT->BRR = SMBUS_SDA

#define SMBUS_SDA_PIN()	    SMBUS_PORT->IDR & SMBUS_SDA //读取引脚电平

/*存储电机温度信息结构体*/
struct MLX90614_Temp_s
{
    float MOTO1_temp;
    float MOTO2_temp;
    float MOTO3_temp;
    float MOTO4_temp;
    float MOTO5_temp;
    float MOTO6_temp;
    float MOTO7_temp;
    float MOTO8_temp;
    float MOTO9_temp;
    float MOTO10_temp;
    float MOTO11_temp;
    float MOTO12_temp;
    float MOTO13_temp;
    float MOTO14_temp;
    float MOTO15_temp;
    float MOTO16_temp;
    
};


extern struct MLX90614_Temp_s MOTO_Temp;


void SMBus_StartBit(void);
void SMBus_StopBit(void);
void SMBus_SendBit(u8);
u8 SMBus_SendByte(u8);
u8 SMBus_ReceiveBit(void);
u8 SMBus_ReceiveByte(u8);
void SMBus_Delay(u16);
void SMBus_Init(void);
u16 SMBus_ReadMemory(u8, u8);
u8 PEC_Calculation(u8*);
float SMBus_ReadTemp(u8 slaveAddress);    //获取温度值
void Get_Temp(uint8_t moto_num);
void print_Temp(uint8_t moto_num);

#endif
