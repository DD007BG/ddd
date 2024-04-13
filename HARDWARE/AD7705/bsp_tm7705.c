/*===================================================
=	模块名称：TM7705 驱动模块(2通道带PGA的16位ADC)
=	说明：使用芯片自带的SPI2驱动
=	作者：xcm
=	详情： 2019-8-28  版本：V1.0
=======================================================*/

#include <bsp_tm7705.h>
#include <drv_spi.h>
#include <delay.h>
#include "usart.h"


/*================下面为芯片的驱动数据定义============================*/
/* 通信寄存器bit定义 */
enum
{
	/* 寄存器选择  RS2 RS1 RS0  */
	REG_COMM	= 0x00,	/* 通信寄存器 */
	REG_SETUP	= 0x10,	/* 设置寄存器 */
	REG_CLOCK	= 0x20,	/* 时钟寄存器 */
	REG_DATA	= 0x30,	/* 数据寄存器 */
	REG_ZERO_CH1	= 0x60,	/* CH1 偏移寄存器 */
	REG_FULL_CH1	= 0x70,	/* CH1 满量程寄存器 */
	REG_ZERO_CH2	= 0x61,	/* CH2 偏移寄存器 */
	REG_FULL_CH2	= 0x71,	/* CH2 满量程寄存器 */

	/* 读写操作 */
	WRITE 		= 0x00,	/* 写操作 */
	READ 		= 0x08,	/* 读操作 */

	/* 通道 */
	CH_1		= 0,	/* AIN1+  AIN1- */
	CH_2		= 1,	/* AIN2+  AIN2- */
	CH_3		= 2,	/* AIN1-  AIN1- */
	CH_4		= 3		/* AIN1-  AIN2- */
};

/* 设置寄存器bit定义 */
enum
{
	MD_NORMAL		= (0 << 6),	/* 正常模式 */
	MD_CAL_SELF		= (1 << 6),	/* 自校准模式 */
	MD_CAL_ZERO		= (2 << 6),	/* 校准0刻度模式 */
	MD_CAL_FULL		= (3 << 6),	/* 校准满刻度模式 */

	GAIN_1			= (0 << 3),	/* 增益 */
	GAIN_2			= (1 << 3),	/* 增益 */
	GAIN_4			= (2 << 3),	/* 增益 */
	GAIN_8			= (3 << 3),	/* 增益 */
	GAIN_16			= (4 << 3),	/* 增益 */
	GAIN_32			= (5 << 3),	/* 增益 */
	GAIN_64			= (6 << 3),	/* 增益 */
	GAIN_128		= (7 << 3),	/* 增益 */

	/* 无论双极性还是单极性都不改变任何输入信号的状态，它只改变输出数据的代码和转换函数上的校准点 */
	BIPOLAR			= (0 << 2),	/* 双极性输入 */
	UNIPOLAR		= (1 << 2),	/* 单极性输入 */

	BUF_NO			= (0 << 1),	/* 输入无缓冲（内部缓冲器不启用) */
	BUF_EN			= (1 << 1),	/* 输入有缓冲 (启用内部缓冲器) 可处理高阻抗源 */

	FSYNC_0			= 0,
	FSYNC_1			= 1		/* 不启用 */
};

/* 时钟寄存器bit定义 */
enum
{
	CLKDIS_0	= 0x00,		/* 时钟输出使能 （当外接晶振时，必须使能才能振荡） */
	CLKDIS_1	= 0x10,		/* 时钟禁止 （当外部提供时钟时，设置该位可以禁止MCK_OUT引脚输出时钟以省电 */

	/*
		2.4576MHz（CLKDIV=0 ）或为 4.9152MHz （CLKDIV=1 ），CLK 应置 “0”。
		1MHz （CLKDIV=0 ）或 2MHz   （CLKDIV=1 ），CLK 该位应置  “1”
	*/
	CLK_4_9152M = 0x08,
	CLK_2_4576M = 0x00,
	CLK_1M 		= 0x04,
	CLK_2M 		= 0x0C,
	
/*输出更新速率设置*/
	FS_20HZ     = 0X00,
	FS_25HZ     = 0x01,
	FS_100HZ    = 0x20,
	FS_200HZ    = 0x03,
	FS_50HZ		= 0x04,
	FS_60HZ		= 0x05,
	FS_250HZ	= 0x06,
	FS_500HZ	= 0x07,

	/*
		四十九、电子秤应用中提高TM7705 精度的方法
			当使用主时钟为 2.4576MHz 时，强烈建议将时钟寄存器设为 84H,此时数据输出更新率为10Hz,即每0.1S 输出一个新数据。
			当使用主时钟为 1MHz 时，强烈建议将时钟寄存器设为80H, 此时数据输出更新率为4Hz, 即每0.25S 输出一个新数据
	*/
	ZERO_0		= 0x00,
	ZERO_1		= 0x80
};

struct TM7705_S TM7705_C1,TM7705_V1;
struct TM7705_S TM7705_C2,TM7705_V2;
struct TM7705_S TM7705_C3,TM7705_V3;
struct TM7705_S TM7705_C4,TM7705_V4;
struct TM7705_S TM7705_C5,TM7705_V5;
struct TM7705_S TM7705_C6,TM7705_V6;
struct TM7705_S TM7705_C7,TM7705_V7;
struct TM7705_S TM7705_C8,TM7705_V8;


static void TM7705_GPIO_Init(uint8_t TM7705_NUM_SUM);
static void TM7705_ResetHard(void);		//硬件复位
static void TM7705_SyncSPI(struct TM7705_S* TM7705_tmp_p);
static uint8_t TM7705_ReadByte(uint8_t COM_index, uint8_t SW);
static uint16_t TM7705_Read2Byte(uint8_t COM_index, uint8_t SW);
static uint16_t TM7705_Read3Byte(uint8_t COM_index, uint8_t SW);
static void TM7705_Chip_Select(uint8_t COM_index, uint8_t SW);
static void TM7705_Init_SubFunc(struct TM7705_S* TM7705_tmp_p, uint8_t COM_index, uint8_t SW);
struct TM7705_S* TM7705_Get_Assign(uint8_t COM_index, uint8_t SW);



/*====================芯片的驱动数据定义到此结束==========================*/
/*=====================================================================
=功能说明：片选到指定COM的电压或电流监测上的7705
=参    数：指定COM SW
=====================================================================*/
static void TM7705_Chip_Select(uint8_t COM_index, uint8_t SW)
{
    struct TM7705_S* TM7705_TMP_p;
    TM7705_TMP_p = (struct TM7705_S *)TM7705_Get_Assign(COM_index, SW);    //TM7705_xy  
    //失能全部7705
    TM7705_CS_C1 = 1;
    TM7705_CS_V1 = 1;
    TM7705_CS_C2 = 1;
    TM7705_CS_V2 = 1;
    TM7705_CS_C3 = 1;
    TM7705_CS_V3 = 1;
    TM7705_CS_C4 = 1;
    TM7705_CS_V4 = 1;
    TM7705_CS_C5 = 1;
    TM7705_CS_V5 = 1;
    TM7705_CS_C6 = 1;
    TM7705_CS_V6 = 1;
    TM7705_CS_C7 = 1;
    TM7705_CS_V7 = 1;
    TM7705_CS_C8 = 1;
    TM7705_CS_V8 = 1;

    //片选指定7705          TM7705_CS_xy.
    //GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin); 
    GPIO_ResetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin); 
}


/*=====================================================================
=功能说明：根据COM_index SW获得指定的结构体变量
=参    数：指定COM SW
=====================================================================*/
struct TM7705_S* TM7705_Get_Assign(uint8_t COM_index, uint8_t SW)
{
    if(SW == TM7705_SW_CUR)
    {
        switch (COM_index)
        {
            case 8: return (struct TM7705_S *)&TM7705_C8;
            case 7: return (struct TM7705_S *)&TM7705_C7;
            case 6: return (struct TM7705_S *)&TM7705_C6;
            case 5: return (struct TM7705_S *)&TM7705_C5;
            case 4: return (struct TM7705_S *)&TM7705_C4;
            case 3: return (struct TM7705_S *)&TM7705_C3;
            case 2: return (struct TM7705_S *)&TM7705_C2;
            case 1: return (struct TM7705_S *)&TM7705_C1;
            default: break;
        }
    }else if(SW == TM7705_SW_VOL)
    {
        switch (COM_index)
        {
            case 8: return (struct TM7705_S *)&TM7705_V8;
            case 7: return (struct TM7705_S *)&TM7705_V7;
            case 6: return (struct TM7705_S *)&TM7705_V6;
            case 5: return (struct TM7705_S *)&TM7705_V5;
            case 4: return (struct TM7705_S *)&TM7705_V4;
            case 3: return (struct TM7705_S *)&TM7705_V3;
            case 2: return (struct TM7705_S *)&TM7705_V2;
            case 1: return (struct TM7705_S *)&TM7705_V1;
            default: break;
        }
    }else
    {
        printf("error: TM7705_Get_Assign input val is wrong\r\n");
    }
		return (struct TM7705_S*)-1;
}


static void TM7705_GPIO_CONCAT(uint8_t COM_index)
{
    switch(COM_index)
    {

        case 1:
        {           
            TM7705_C1.CS_GPIO_Port = TM7705_CS_PORT_C1;
            TM7705_C1.CS_GPIO_Pin  = TM7705_CS_PIN_C1;
            TM7705_C1.CS_GPIO_Mode = TM7705_CS_MODE_C1;
            TM7705_C1.RD_GPIO_Port = TM7705_RD_PORT_C1;
            TM7705_C1.RD_GPIO_Pin  = TM7705_RD_PIN_C1;
            TM7705_C1.RD_GPIO_Mode = TM7705_RD_MODE_C1;
            
            TM7705_V1.CS_GPIO_Port = TM7705_CS_PORT_V1;
            TM7705_V1.CS_GPIO_Pin  = TM7705_CS_PIN_V1;
            TM7705_V1.CS_GPIO_Mode = TM7705_CS_MODE_V1;
            TM7705_V1.RD_GPIO_Port = TM7705_RD_PORT_V1;
            TM7705_V1.RD_GPIO_Pin  = TM7705_RD_PIN_V1;
            TM7705_V1.RD_GPIO_Mode = TM7705_RD_MODE_V1;

            break;
        }
        case 2:
        {
            TM7705_C2.CS_GPIO_Port = TM7705_CS_PORT_C2;
            TM7705_C2.CS_GPIO_Pin  = TM7705_CS_PIN_C2;
            TM7705_C2.CS_GPIO_Mode = TM7705_CS_MODE_C2;
            TM7705_C2.RD_GPIO_Port = TM7705_RD_PORT_C2;
            TM7705_C2.RD_GPIO_Pin  = TM7705_RD_PIN_C2;
            TM7705_C2.RD_GPIO_Mode = TM7705_RD_MODE_C2;
            
            TM7705_V2.CS_GPIO_Port = TM7705_CS_PORT_V2;
            TM7705_V2.CS_GPIO_Pin  = TM7705_CS_PIN_V2;
            TM7705_V2.CS_GPIO_Mode = TM7705_CS_MODE_V2;
            TM7705_V2.RD_GPIO_Port = TM7705_RD_PORT_V2;
            TM7705_V2.RD_GPIO_Pin  = TM7705_RD_PIN_V2;
            TM7705_V2.RD_GPIO_Mode = TM7705_RD_MODE_V2;

            break;
        }
        case 3:
        {
            TM7705_C3.CS_GPIO_Port = TM7705_CS_PORT_C3;
            TM7705_C3.CS_GPIO_Pin  = TM7705_CS_PIN_C3;
            TM7705_C3.CS_GPIO_Mode = TM7705_CS_MODE_C3;
            TM7705_C3.RD_GPIO_Port = TM7705_RD_PORT_C3;
            TM7705_C3.RD_GPIO_Pin  = TM7705_RD_PIN_C3;
            TM7705_C3.RD_GPIO_Mode = TM7705_RD_MODE_C3;
            
            TM7705_V3.CS_GPIO_Port = TM7705_CS_PORT_V3;
            TM7705_V3.CS_GPIO_Pin  = TM7705_CS_PIN_V3;
            TM7705_V3.CS_GPIO_Mode = TM7705_CS_MODE_V3;
            TM7705_V3.RD_GPIO_Port = TM7705_RD_PORT_V3;
            TM7705_V3.RD_GPIO_Pin  = TM7705_RD_PIN_V3;
            TM7705_V3.RD_GPIO_Mode = TM7705_RD_MODE_V3;

            break;
        }
        case 4:
        {
            TM7705_C4.CS_GPIO_Port = TM7705_CS_PORT_C4;
            TM7705_C4.CS_GPIO_Pin  = TM7705_CS_PIN_C4;
            TM7705_C4.CS_GPIO_Mode = TM7705_CS_MODE_C4;
            TM7705_C4.RD_GPIO_Port = TM7705_RD_PORT_C4;
            TM7705_C4.RD_GPIO_Pin  = TM7705_RD_PIN_C4;
            TM7705_C4.RD_GPIO_Mode = TM7705_RD_MODE_C4;
            
            TM7705_V4.CS_GPIO_Port = TM7705_CS_PORT_V4;
            TM7705_V4.CS_GPIO_Pin  = TM7705_CS_PIN_V4;
            TM7705_V4.CS_GPIO_Mode = TM7705_CS_MODE_V4;
            TM7705_V4.RD_GPIO_Port = TM7705_RD_PORT_V4;
            TM7705_V4.RD_GPIO_Pin  = TM7705_RD_PIN_V4;
            TM7705_V4.RD_GPIO_Mode = TM7705_RD_MODE_V4;

            break;
        }
        case 5:
        {
            TM7705_C5.CS_GPIO_Port = TM7705_CS_PORT_C5;
            TM7705_C5.CS_GPIO_Pin  = TM7705_CS_PIN_C5;
            TM7705_C5.CS_GPIO_Mode = TM7705_CS_MODE_C5;
            TM7705_C5.RD_GPIO_Port = TM7705_RD_PORT_C5;
            TM7705_C5.RD_GPIO_Pin  = TM7705_RD_PIN_C5;
            TM7705_C5.RD_GPIO_Mode = TM7705_RD_MODE_C5;
            
            TM7705_V5.CS_GPIO_Port = TM7705_CS_PORT_V5;
            TM7705_V5.CS_GPIO_Pin  = TM7705_CS_PIN_V5;
            TM7705_V5.CS_GPIO_Mode = TM7705_CS_MODE_V5;
            TM7705_V5.RD_GPIO_Port = TM7705_RD_PORT_V5;
            TM7705_V5.RD_GPIO_Pin  = TM7705_RD_PIN_V5;
            TM7705_V5.RD_GPIO_Mode = TM7705_RD_MODE_V5;

            break;
        }
        case 6:
        {
            TM7705_C6.CS_GPIO_Port = TM7705_CS_PORT_C6;
            TM7705_C6.CS_GPIO_Pin  = TM7705_CS_PIN_C6;
            TM7705_C6.CS_GPIO_Mode = TM7705_CS_MODE_C6;
            TM7705_C6.RD_GPIO_Port = TM7705_RD_PORT_C6;
            TM7705_C6.RD_GPIO_Pin  = TM7705_RD_PIN_C6;
            TM7705_C6.RD_GPIO_Mode = TM7705_RD_MODE_C6;
            
            TM7705_V6.CS_GPIO_Port = TM7705_CS_PORT_V6;
            TM7705_V6.CS_GPIO_Pin  = TM7705_CS_PIN_V6;
            TM7705_V6.CS_GPIO_Mode = TM7705_CS_MODE_V6;
            TM7705_V6.RD_GPIO_Port = TM7705_RD_PORT_V6;
            TM7705_V6.RD_GPIO_Pin  = TM7705_RD_PIN_V6;
            TM7705_V6.RD_GPIO_Mode = TM7705_RD_MODE_V6;

            break;
        }
        case 7:
        {
            TM7705_C7.CS_GPIO_Port = TM7705_CS_PORT_C7;
            TM7705_C7.CS_GPIO_Pin  = TM7705_CS_PIN_C7;
            TM7705_C7.CS_GPIO_Mode = TM7705_CS_MODE_C7;
            TM7705_C7.RD_GPIO_Port = TM7705_RD_PORT_C7;
            TM7705_C7.RD_GPIO_Pin  = TM7705_RD_PIN_C7;
            TM7705_C7.RD_GPIO_Mode = TM7705_RD_MODE_C7;
            
            TM7705_V7.CS_GPIO_Port = TM7705_CS_PORT_V7;
            TM7705_V7.CS_GPIO_Pin  = TM7705_CS_PIN_V7;
            TM7705_V7.CS_GPIO_Mode = TM7705_CS_MODE_V7;
            TM7705_V7.RD_GPIO_Port = TM7705_RD_PORT_V7;
            TM7705_V7.RD_GPIO_Pin  = TM7705_RD_PIN_V7;
            TM7705_V7.RD_GPIO_Mode = TM7705_RD_MODE_V7;

            break;
        }
        case 8:
        {
            TM7705_C8.CS_GPIO_Port = TM7705_CS_PORT_C8;
            TM7705_C8.CS_GPIO_Pin  = TM7705_CS_PIN_C8;
            TM7705_C8.CS_GPIO_Mode = TM7705_CS_MODE_C8;
            TM7705_C8.RD_GPIO_Port = TM7705_RD_PORT_C8;
            TM7705_C8.RD_GPIO_Pin  = TM7705_RD_PIN_C8;
            TM7705_C8.RD_GPIO_Mode = TM7705_RD_MODE_C8;
            
            TM7705_V8.CS_GPIO_Port = TM7705_CS_PORT_V8;
            TM7705_V8.CS_GPIO_Pin  = TM7705_CS_PIN_V8;
            TM7705_V8.CS_GPIO_Mode = TM7705_CS_MODE_V8;
            TM7705_V8.RD_GPIO_Port = TM7705_RD_PORT_V8;
            TM7705_V8.RD_GPIO_Pin  = TM7705_RD_PIN_V8;
            TM7705_V8.RD_GPIO_Mode = TM7705_RD_MODE_V8;

            break;
        }
        default:
        {
            printf("err:   TM7705_GPIO_CONCAT input num is error\r\n");
            break;
        }
    }
}
   

static void TM7705_GPIO_Init(uint8_t TM7705_NUM_SUM)
{
    uint8_t COM_index;
    struct TM7705_S* TM7705_TMP_p;
    /*==信号引脚初始化==*/
	GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 //使能端口时钟	

    for(COM_index = 1; COM_index <= TM7705_NUM_SUM; COM_index++)
    {
        TM7705_GPIO_CONCAT(COM_index);

        TM7705_TMP_p = TM7705_Get_Assign(COM_index, TM7705_SW_CUR);       //TM7705_Cx
        GPIO_InitStructure.GPIO_Pin  = TM7705_TMP_p->CS_GPIO_Pin;	        //推挽 	 CS 
        GPIO_InitStructure.GPIO_Mode = TM7705_TMP_p->CS_GPIO_Mode;
        GPIO_Init(TM7705_TMP_p->CS_GPIO_Port, &GPIO_InitStructure);
        GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);	        

        GPIO_InitStructure.GPIO_Pin  = TM7705_TMP_p->RD_GPIO_Pin;         //输入  DRAY
    	GPIO_InitStructure.GPIO_Mode =  TM7705_TMP_p->RD_GPIO_Mode;
    	GPIO_Init(TM7705_TMP_p->RD_GPIO_Port, &GPIO_InitStructure);
    	GPIO_ResetBits(TM7705_TMP_p->RD_GPIO_Port, TM7705_TMP_p->RD_GPIO_Pin);

        TM7705_TMP_p = TM7705_Get_Assign(COM_index, TM7705_SW_VOL);       //TM7705_Vx
        GPIO_InitStructure.GPIO_Pin  = TM7705_TMP_p->CS_GPIO_Pin;	        //推挽 	 CS 
        GPIO_InitStructure.GPIO_Mode = TM7705_TMP_p->CS_GPIO_Mode;
        GPIO_Init(TM7705_TMP_p->CS_GPIO_Port, &GPIO_InitStructure);
        GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);	        

        GPIO_InitStructure.GPIO_Pin  = TM7705_TMP_p->RD_GPIO_Pin;         //输入  DRAY
    	GPIO_InitStructure.GPIO_Mode =  TM7705_TMP_p->RD_GPIO_Mode;
    	GPIO_Init(TM7705_TMP_p->RD_GPIO_Port, &GPIO_InitStructure);
    	GPIO_ResetBits(TM7705_TMP_p->RD_GPIO_Port, TM7705_TMP_p->RD_GPIO_Pin);
    }   
}


/*******************************************************************************
 * 函数名: TM7705_Init
 * 功能: 根据传入参数，初始化指定数量的7705
 * Input          : TM7705_NUM
 * Return         : void
 * 
*******************************************************************************/
void TM7705_Init(int TM7705_NUM_SUM) //初始化函数
{
    uint8_t COM_index;
    struct TM7705_S* TM7705_TMP_p;     
    
    TM7705_GPIO_Init(TM7705_NUM_SUM);
		
	SPI2_Init();			//初始化SPI2外设，在drv_spi.h中实现
	SPI2_SetSpeed(SPI_BaudRatePrescaler_2);//设置为18M时钟,高速模式

    for(COM_index = 1; COM_index <= TM7705_NUM_SUM; COM_index++)
    {

        /******************初始化电流测量的7705*****************/
        TM7705_TMP_p = TM7705_Get_Assign(COM_index, TM7705_SW_CUR);    
        TM7705_Init_SubFunc(TM7705_TMP_p, COM_index, TM7705_SW_CUR);
        /******************初始化电压测量的7705*****************/
        TM7705_TMP_p = TM7705_Get_Assign(COM_index, TM7705_SW_VOL);    
        TM7705_Init_SubFunc(TM7705_TMP_p, COM_index, TM7705_SW_VOL);
    }
}	

static void TM7705_Init_SubFunc(struct TM7705_S* TM7705_tmp_p, uint8_t COM_index, uint8_t SW)
{
    //TM7705_CS = 1; 			//TM7705初始化不选中
	//TM7705_RST = 0; 
    GPIO_SetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin);  //TM7705_CS = 1; 	//TM7705初始化不选中

	delay_ms(10);
	TM7705_ResetHard();		//硬件复位
	delay_ms(5);
	
	TM7705_SyncSPI(TM7705_tmp_p);      //同步SPI接口时序
	delay_ms(10);
	
	/*配置时钟寄存器*/
	GPIO_ResetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin); //TM7705_CS = 0; 
	SPI2_ReadWriteByte(REG_CLOCK | WRITE |CH_1); //先写通信寄存器，下一步写时钟寄存器
	delay_ms(10);
	SPI2_ReadWriteByte(CLKDIS_0 | CLK_4_9152M | FS_500HZ ); //刷新速率250HZ
	delay_ms(10);
	SPI2_ReadWriteByte(REG_CLOCK | WRITE |CH_2); //先写通信寄存器，下一步写时钟寄存器
	delay_ms(10);
	SPI2_ReadWriteByte(CLKDIS_0 | CLK_4_9152M | FS_500HZ ); //刷新速率250HZ
	delay_ms(10);
	GPIO_SetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin); //TM7705_CS = 1;

	/*每次上电进行一次自校准*/
	TM7705_CalibSelf(COM_index, SW, 1);      //内部自校准
	TM7705_CalibSelf(COM_index, SW, 2);      //内部自校准
	delay_ms(5);
}


/*===硬件复位Tm7705芯片，无出入参数===*/
static void TM7705_ResetHard(void)	//硬件复位
{
	/*TM7705_RST = 1;
	delay_ms(1);
	TM7705_RST = 0;
	delay_ms(2);
	TM7705_RST = 1;
	delay_ms(1);*/
}

/*=============================================
= 功能：同步TM7705芯片SPI接口时序
= 说明：连续发送32个1即可，不同步会发生数据错位   
==============================================*/
static void TM7705_SyncSPI(struct TM7705_S* TM7705_tmp_p)      //同步SPI接口时序
{
	GPIO_ResetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin); //TM7705_CS = 0; 
	SPI2_ReadWriteByte(0xFF);//32个1
	SPI2_ReadWriteByte(0xFF);
	SPI2_ReadWriteByte(0xFF);
	SPI2_ReadWriteByte(0xFF);
    
	GPIO_SetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin); //TM7705_CS = 1;
}

/*====================================================================
=功能说明： 等待内部操作完成，自校准时间较长，需要等待
=参	   数： 无
======================================================================*/
static void TM7705_WaitDRDY(uint8_t COM_index, uint8_t SW)
{
	uint32_t i;
    struct TM7705_S* TM7705_TMP_p = TM7705_Get_Assign(COM_index, SW);    //TM7705_xy  
    
	for(i = 0; i<400000; i++)
	{
		if(GPIO_ReadInputDataBit(TM7705_TMP_p->RD_GPIO_Port,TM7705_TMP_p->RD_GPIO_Pin) == 0)  //7705_RD == 0?
		{
			break;
		}
	}
	if(i >=400000 )
	{
        printf("TM7705 WAIT TIME OUT !!!\r\n");
	}
}



/*====================================================================
= 功能说明: 启动自校准. 内部自动短接AIN+ AIN-校准0位，内部短接到Vref 
			校准满位。此函数执行过程较长，实测约 180ms	
= 形    参:  _ch : ADC通道，1或2
=====================================================================*/

void TM7705_CalibSelf(uint8_t Chip_index,uint8_t SW,uint8_t _ch)
{
    TM7705_Chip_Select(Chip_index, SW);
	if (_ch == 1)
	{
		/* 自校准CH1 */
		SPI2_ReadWriteByte(REG_SETUP | WRITE | CH_1);	/* 写通信寄存器，下一步是写设置寄存器，通道1 */		
		SPI2_ReadWriteByte(MD_CAL_SELF | GAIN_1 | UNIPOLAR	|BUF_EN |FSYNC_0);/* 启动自校准 *///写设置寄存器，单极性、有缓冲、增益为1、滤波器工作、自校准
		//SPI2_ReadWriteByte(0x4C);
		TM7705_WaitDRDY(Chip_index, SW);	/* 等待内部操作完成 --- 时间较长，约180ms */
	}
	else if (_ch == 2)
	{
		/* 自校准CH2 */
		SPI2_ReadWriteByte(REG_SETUP | WRITE | CH_2);	/* 写通信寄存器，下一步是写设置寄存器，通道2 */
		SPI2_ReadWriteByte(MD_CAL_SELF | GAIN_1 | UNIPOLAR |BUF_EN |FSYNC_0);	/* 启动自校准 */
		//SPI2_ReadWriteByte(0x44);
		TM7705_WaitDRDY(Chip_index, SW);	/* 等待内部操作完成  --- 时间较长，约180ms */
	}


}

/*=====================================================================
=功能说明：读到一个8位数据
=参    数：返回读到的值
=====================================================================*/
static uint8_t TM7705_ReadByte(uint8_t COM_index, uint8_t SW)
{
	uint8_t read = 0;
    struct TM7705_S* TM7705_TMP_p = TM7705_Get_Assign(COM_index, SW);    //TM7705_xy  
    
	GPIO_ResetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 0;
	read = SPI2_ReadWriteByte(0xFF);
	GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 1;
	
	return read;
}

/*=====================================================================
=功能说明：读到一个16位数据（半字）
=参    数：返回读到16位数据的值
=====================================================================*/
static uint16_t TM7705_Read2Byte(uint8_t COM_index, uint8_t SW)
{
	uint16_t read = 0;
    struct TM7705_S* TM7705_TMP_p = TM7705_Get_Assign(COM_index, SW);    //TM7705_xy  
	
	GPIO_ResetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 0;
	read = SPI2_ReadWriteByte(0xFF);
	read <<=8;
	read += SPI2_ReadWriteByte(0xFF);
	GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 1;

	return read;
}

/*=====================================================================
=功能说明：读到一个24位数据（3字节）
=参    数：返回读到24位数据的值
=====================================================================*/
static uint16_t TM7705_Read3Byte(uint8_t COM_index, uint8_t SW)
{
	uint32_t read = 0;
    struct TM7705_S* TM7705_TMP_p = TM7705_Get_Assign(COM_index, SW);    //TM7705_xy   
    
	GPIO_ResetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 0;
	read = SPI2_ReadWriteByte(0xFF);
	read <<=8;
	read += SPI2_ReadWriteByte(0xFF);
	read <<=8;
	read += SPI2_ReadWriteByte(0xFF);
	GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 1;
	
	return read;
}

/*=====================================================================
=功能说明：读取采样电压值
=参    数：返回采样值
=====================================================================*/
uint16_t TM7705_ReadAdc(uint8_t COM_index, uint8_t SW, uint8_t ch)
{
	uint8_t i;
	uint16_t read = 0;
    struct TM7705_S* TM7705_TMP_p = TM7705_Get_Assign(COM_index, SW);    //TM7705_xy 
    TM7705_Chip_Select(COM_index, SW);                      //片选指定7705
	/* 为了避免通道切换造成读数失效，读2次 */
	for (i = 0; i < 2; i++)
	{
		TM7705_WaitDRDY(COM_index, SW);		/* 等待DRDY口线为0 */		

		if (ch == 1)
		{
			GPIO_ResetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 0;
			SPI2_ReadWriteByte(0x38);
			GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 1;
		}
		else if (ch == 2)
		{
			GPIO_ResetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 0;
			SPI2_ReadWriteByte(0x39);
			GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);  //TM7705_CS = 1;
		}
		//TM7705_WaitDRDY();	/* 等待内部操作完成 --- 时间较长，约180ms */
		read = TM7705_Read2Byte(COM_index, SW);
	}
	return read;	
}


 /*====================================================================
 = 功能说明: 读取指定COM，指定电压或者电流，指定通道的7705采样值。
 = 形    参: COM_index :  指定COMx,1-8
            SW        :  指定获取电压或者电流的采样值
            ch        :  ADC通道，0表示全部获得，1或2则指定通道1或2
 =====================================================================*/

void Conv_True_Vol(uint8_t COM_index, uint8_t SW, uint8_t ch)
{
    uint16_t vol_adc1 = 0, vol_adc2 = 0;
    int      vol_val1 = 0, vol_val2 = 0;
	
    if(1 == ch || 2 == ch)
	    vol_adc1 = TM7705_ReadAdc(COM_index, SW, ch);	/* 执行时间 80ms */		
    else if(0 == ch)/* 双通道切换采样，执行一轮实际那约 160ms */		
    {
        vol_adc1 = TM7705_ReadAdc(COM_index, SW, 1);	/* 执行时间 80ms */	
        vol_adc2 = TM7705_ReadAdc(COM_index, SW, 2);	/* 执行时间 80ms */
   
}else           /*错误的ch值输入*/
    {
        vol_adc1 = 0;
        vol_adc2 = 0;
    }

   if(1 == ch || 2 == ch)
   {
       	vol_val1 = (vol_adc1 * 5000) / 65535;/* 计算实际电压值（近似估算的），如需准确，请进行校准 */
      	/* 打印ADC采样结果 */	
    	printf("SW=%d Chip_index=%d CH%d=%5d (%5dmV) \r\n", SW, COM_index, ch, vol_adc1, vol_val1);
   }
   else if(0 == ch){
        /* 计算实际电压值（近似估算的），如需准确，请进行校准 */
        vol_val1 = (vol_adc1 * 5000) / 65535;
    	vol_val2 = (vol_adc2 * 5000) / 65535;
    	/* 打印ADC采样结果 */	
    	printf("SW=%d Chip_index=%d CH1=%5d (%5dmV) CH2=%5d (%5dmV)\r\n", SW, COM_index, vol_adc1, vol_val1, vol_adc2, vol_val2);
   }
   else{
        printf("err: Get_Vol input val wrong \r\n");
   }
}


/*====================================================================
= 功能说明: 根据输入的需要测量的电压路数CHANNEL_NUM，调用Conv_True_Vol。
            注意，默认从第1路开始顺序测量，即当CHANNEL_NUM=3时，默认输
            出1-3路的电压测量结果
= 形    参: CHANNEL_NUM :  需要测量的电压路数，1-16.
           
=====================================================================*/

void Get_Vol(uint8_t CHANNEL_NUM)
{
    int COM_index;
    for(COM_index = 1; COM_index <= CHANNEL_NUM; COM_index++)
    {
        delay_ms(1000);
        Conv_True_Vol(COM_index, TM7705_SW_VOL, TM7705_CH_ALL);       //获得并打印电压值
    }
}



/*====================================================================
= 功能说明: 根据输入的需要测量的电流路数CHANNEL_NUM，调用Conv_True_Vol。
            注意，默认从第1路开始顺序测量，即当CHANNEL_NUM=3时，默认输
            出1-3路的电流测量结果
= 形    参: CHANNEL_NUM :  需要测量的电流路数，1-16.
           
=====================================================================*/

void Get_Cur(uint8_t CHANNEL_NUM)
{
   int COM_index;
   for(COM_index = 1; COM_index <= CHANNEL_NUM; COM_index++)
   {
        delay_ms(1000);
        Conv_True_Vol(COM_index, TM7705_SW_CUR, TM7705_CH_ALL);       //获得并打印电流值
   }
}

 

