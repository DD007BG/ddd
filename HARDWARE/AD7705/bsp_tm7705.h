/*********************************************************************************************************
*
*	模块名称 : TM7705 驱动模块(2通道带PGA的16位ADC) 
*	文件名称 : drv_tm7705.h
*	功能： 实现模块对stm32f103的移植
*	作者：肖春明  
*
*********************************************************************************************************
*/

#ifndef _DRV_TM7705_H
#define _DRV_TM7705_H

#include "sys.h"
/*
      SCK   ------  SPI2_SCK
      DOUT  ------  SPI2_MISO
      DIN   ------  SPI2_MOSI   SPI部分的引脚定义再drv_spi.c中

      RST   ------  暂时悬空，后续看情况时候全部模块的rst接一起

      第一路
      CS_1    ------  PB11
      RD_1    ------  PB10

      第二路
      CS_2    ------  PB1
      RD_2    ------  PB0
                           
*/
#define __CONCAT(HEAD,index) HEAD##index
#define __CONCAT2(HEAD,mode,index) HEAD##mode##index
                                     


enum
{
	TM7705_SW_CUR,	/* 选择电流测量 */
	TM7705_SW_VOL	/* 选择电压测量 */
};

enum
{
    TM7705_CH_ALL   = 0,    /* 测量两通道*/
	TM7705_CH_FIRST = 1,	/* 测量通道1 */
	TM7705_CH_SECOND= 2 	/* 测量通道2 */
};



struct TM7705_S
{
    GPIO_TypeDef*       CS_GPIO_Port;
    uint16_t            CS_GPIO_Pin;
    GPIOMode_TypeDef    CS_GPIO_Mode;

    GPIO_TypeDef*       RD_GPIO_Port;
    uint16_t            RD_GPIO_Pin;
    GPIOMode_TypeDef    RD_GPIO_Mode;
};


/*******************************定义GPIO端口**********************************/
/******************************** COM1 ********************************/
#define TM7705_CS_PORT_C1  	    GPIOC
#define TM7705_CS_PIN_C1	    GPIO_Pin_14
#define TM7705_CS_MODE_C1  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_C1	    GPIOC
#define TM7705_RD_PIN_C1 	    GPIO_Pin_15
#define TM7705_RD_MODE_C1 	    GPIO_Mode_IN_FLOATING


#define TM7705_CS_PORT_V1  	    GPIOC
#define TM7705_CS_PIN_V1	    GPIO_Pin_0
#define TM7705_CS_MODE_V1  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_V1 	    GPIOC
#define TM7705_RD_PIN_V1 	    GPIO_Pin_1
#define TM7705_RD_MODE_V1 	    GPIO_Mode_IN_FLOATING

//#define TM7705_RST_1 		    PAout(7)    //RST使能复位宏定义，低电平有效 //暂时不接
#define TM7705_CS_C1 		    PCout(14)   //CS片选宏定义，低电平有效
#define TM7705_RD_C1            PCin(15)    //RD,低电平时表示TM7705可读
#define TM7705_CS_V1 		    PCout(0) 
#define TM7705_RD_V1            PCin(1)  


/******************************** COM2 ********************************/
#define TM7705_CS_PORT_C2  	    GPIOA
#define TM7705_CS_PIN_C2	    GPIO_Pin_1
#define TM7705_CS_MODE_C2  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_C2	    GPIOA
#define TM7705_RD_PIN_C2 	    GPIO_Pin_0
#define TM7705_RD_MODE_C2 	    GPIO_Mode_IN_FLOATING


#define TM7705_CS_PORT_V2  	    GPIOC
#define TM7705_CS_PIN_V2	    GPIO_Pin_3
#define TM7705_CS_MODE_V2  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_V2 	    GPIOC
#define TM7705_RD_PIN_V2 	    GPIO_Pin_2
#define TM7705_RD_MODE_V2 	    GPIO_Mode_IN_FLOATING

#define TM7705_CS_C2 		    PAout(1)   
#define TM7705_RD_C2            PAin(0)   
#define TM7705_CS_V2 		    PCout(3) 
#define TM7705_RD_V2            PCin(2) 


/******************************** COM3 ********************************/
#define TM7705_CS_PORT_C3  	    GPIOA
#define TM7705_CS_PIN_C3	    GPIO_Pin_7
#define TM7705_CS_MODE_C3  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_C3	    GPIOC
#define TM7705_RD_PIN_C3 	    GPIO_Pin_4
#define TM7705_RD_MODE_C3 	    GPIO_Mode_IN_FLOATING


#define TM7705_CS_PORT_V3  	    GPIOC
#define TM7705_CS_PIN_V3	    GPIO_Pin_5
#define TM7705_CS_MODE_V3  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_V3 	    GPIOB
#define TM7705_RD_PIN_V3 	    GPIO_Pin_0
#define TM7705_RD_MODE_V3 	    GPIO_Mode_IN_FLOATING

#define TM7705_CS_C3 		    PAout(7)   
#define TM7705_RD_C3            PCin(4)   
#define TM7705_CS_V3 		    PCout(5) 
#define TM7705_RD_V3            PBin(0)



/******************************** COM4 ********************************/
#define TM7705_CS_PORT_C4  	    GPIOB
#define TM7705_CS_PIN_C4	    GPIO_Pin_1
#define TM7705_CS_MODE_C4  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_C4	    GPIOB
#define TM7705_RD_PIN_C4 	    GPIO_Pin_2
#define TM7705_RD_MODE_C4 	    GPIO_Mode_IN_FLOATING


#define TM7705_CS_PORT_V4  	    GPIOB
#define TM7705_CS_PIN_V4	    GPIO_Pin_10
#define TM7705_CS_MODE_V4  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_V4 	    GPIOB
#define TM7705_RD_PIN_V4 	    GPIO_Pin_11
#define TM7705_RD_MODE_V4 	    GPIO_Mode_IN_FLOATING

#define TM7705_CS_C4 		    PBout(1)   
#define TM7705_RD_C4            PBin(2)   
#define TM7705_CS_V4 		    PBout(10) 
#define TM7705_RD_V4            PBin(11)


/******************************** COM5 ********************************/
#define TM7705_CS_PORT_C5  	    GPIOB
#define TM7705_CS_PIN_C5	    GPIO_Pin_12
#define TM7705_CS_MODE_C5  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_C5	    GPIOC
#define TM7705_RD_PIN_C5 	    GPIO_Pin_6
#define TM7705_RD_MODE_C5 	    GPIO_Mode_IN_FLOATING


#define TM7705_CS_PORT_V5  	    GPIOC
#define TM7705_CS_PIN_V5	    GPIO_Pin_7
#define TM7705_CS_MODE_V5  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_V5 	    GPIOC
#define TM7705_RD_PIN_V5 	    GPIO_Pin_8
#define TM7705_RD_MODE_V5 	    GPIO_Mode_IN_FLOATING

#define TM7705_CS_C5 		    PBout(12)   
#define TM7705_RD_C5            PCin(6)   
#define TM7705_CS_V5 		    PCout(7) 
#define TM7705_RD_V5            PCin(8)


/******************************** COM6 ********************************/
#define TM7705_CS_PORT_C6  	    GPIOA
#define TM7705_CS_PIN_C6	    GPIO_Pin_8
#define TM7705_CS_MODE_C6  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_C6	    GPIOC
#define TM7705_RD_PIN_C6 	    GPIO_Pin_9
#define TM7705_RD_MODE_C6 	    GPIO_Mode_IN_FLOATING


#define TM7705_CS_PORT_V6  	    GPIOA
#define TM7705_CS_PIN_V6	    GPIO_Pin_11
#define TM7705_CS_MODE_V6  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_V6 	    GPIOA
#define TM7705_RD_PIN_V6 	    GPIO_Pin_12
#define TM7705_RD_MODE_V6 	    GPIO_Mode_IN_FLOATING

#define TM7705_CS_C6 		    PAout(8)   
#define TM7705_RD_C6            PCin(9)   
#define TM7705_CS_V6 		    PAout(11) 
#define TM7705_RD_V6            PAin(12)


/******************************** COM7********************************/
#define TM7705_CS_PORT_C7  	    GPIOA
#define TM7705_CS_PIN_C7	    GPIO_Pin_15
#define TM7705_CS_MODE_C7  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_C7	    GPIOC
#define TM7705_RD_PIN_C7 	    GPIO_Pin_10
#define TM7705_RD_MODE_C7 	    GPIO_Mode_IN_FLOATING


#define TM7705_CS_PORT_V7  	    GPIOC
#define TM7705_CS_PIN_V7	    GPIO_Pin_11
#define TM7705_CS_MODE_V7  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_V7 	    GPIOC
#define TM7705_RD_PIN_V7 	    GPIO_Pin_12
#define TM7705_RD_MODE_V7 	    GPIO_Mode_IN_FLOATING

#define TM7705_CS_C7 		    PAout(15)   
#define TM7705_RD_C7            PCin(10)   
#define TM7705_CS_V7 		    PCout(11) 
#define TM7705_RD_V7            PCin(12)


/******************************** COM8********************************/
#define TM7705_CS_PORT_C8  	    GPIOB
#define TM7705_CS_PIN_C8	    GPIO_Pin_5
#define TM7705_CS_MODE_C8  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_C8	    GPIOB
#define TM7705_RD_PIN_C8 	    GPIO_Pin_4
#define TM7705_RD_MODE_C8 	    GPIO_Mode_IN_FLOATING


#define TM7705_CS_PORT_V8  	    GPIOB
#define TM7705_CS_PIN_V8	    GPIO_Pin_3
#define TM7705_CS_MODE_V8  	    GPIO_Mode_Out_PP

#define TM7705_RD_PORT_V8 	    GPIOD
#define TM7705_RD_PIN_V8 	    GPIO_Pin_2
#define TM7705_RD_MODE_V8 	    GPIO_Mode_IN_FLOATING

#define TM7705_CS_C8 		    PBout(5)   
#define TM7705_RD_C8            PBin(4)   
#define TM7705_CS_V8 		    PBout(3) 
#define TM7705_RD_V8            PDin(2)
/*******************************定义GPIO端口      END******************************/



extern struct TM7705_S TM7705_C1,TM7705_V1;
extern struct TM7705_S TM7705_C2,TM7705_V2;
extern struct TM7705_S TM7705_C3,TM7705_V3;
extern struct TM7705_S TM7705_C4,TM7705_V4;
extern struct TM7705_S TM7705_C5,TM7705_V5;
extern struct TM7705_S TM7705_C6,TM7705_V6;
extern struct TM7705_S TM7705_C7,TM7705_V7;
extern struct TM7705_S TM7705_C8,TM7705_V8;




void TM7705_CalibSelf(uint8_t Chip_index,uint8_t SW,uint8_t _ch); //启动自校准
void TM7705_Init(int TM7705_NUM_SUM); //初始化函数
uint16_t TM7705_ReadAdc(uint8_t COM_index, uint8_t SW, uint8_t ch);
void Conv_True_Vol(uint8_t COM_index, uint8_t SW, uint8_t ch);
void Get_Vol(uint8_t CHANNEL_NUM);
void Get_Cur(uint8_t CHANNEL_NUM);



#endif
