/*===================================================
=	ģ�����ƣ�TM7705 ����ģ��(2ͨ����PGA��16λADC)
=	˵����ʹ��оƬ�Դ���SPI2����
=	���ߣ�xcm
=	���飺 2019-8-28  �汾��V1.0
=======================================================*/

#include <bsp_tm7705.h>
#include <drv_spi.h>
#include <delay.h>
#include "usart.h"


/*================����ΪоƬ���������ݶ���============================*/
/* ͨ�żĴ���bit���� */
enum
{
	/* �Ĵ���ѡ��  RS2 RS1 RS0  */
	REG_COMM	= 0x00,	/* ͨ�żĴ��� */
	REG_SETUP	= 0x10,	/* ���üĴ��� */
	REG_CLOCK	= 0x20,	/* ʱ�ӼĴ��� */
	REG_DATA	= 0x30,	/* ���ݼĴ��� */
	REG_ZERO_CH1	= 0x60,	/* CH1 ƫ�ƼĴ��� */
	REG_FULL_CH1	= 0x70,	/* CH1 �����̼Ĵ��� */
	REG_ZERO_CH2	= 0x61,	/* CH2 ƫ�ƼĴ��� */
	REG_FULL_CH2	= 0x71,	/* CH2 �����̼Ĵ��� */

	/* ��д���� */
	WRITE 		= 0x00,	/* д���� */
	READ 		= 0x08,	/* ������ */

	/* ͨ�� */
	CH_1		= 0,	/* AIN1+  AIN1- */
	CH_2		= 1,	/* AIN2+  AIN2- */
	CH_3		= 2,	/* AIN1-  AIN1- */
	CH_4		= 3		/* AIN1-  AIN2- */
};

/* ���üĴ���bit���� */
enum
{
	MD_NORMAL		= (0 << 6),	/* ����ģʽ */
	MD_CAL_SELF		= (1 << 6),	/* ��У׼ģʽ */
	MD_CAL_ZERO		= (2 << 6),	/* У׼0�̶�ģʽ */
	MD_CAL_FULL		= (3 << 6),	/* У׼���̶�ģʽ */

	GAIN_1			= (0 << 3),	/* ���� */
	GAIN_2			= (1 << 3),	/* ���� */
	GAIN_4			= (2 << 3),	/* ���� */
	GAIN_8			= (3 << 3),	/* ���� */
	GAIN_16			= (4 << 3),	/* ���� */
	GAIN_32			= (5 << 3),	/* ���� */
	GAIN_64			= (6 << 3),	/* ���� */
	GAIN_128		= (7 << 3),	/* ���� */

	/* ����˫���Ի��ǵ����Զ����ı��κ������źŵ�״̬����ֻ�ı�������ݵĴ����ת�������ϵ�У׼�� */
	BIPOLAR			= (0 << 2),	/* ˫�������� */
	UNIPOLAR		= (1 << 2),	/* ���������� */

	BUF_NO			= (0 << 1),	/* �����޻��壨�ڲ�������������) */
	BUF_EN			= (1 << 1),	/* �����л��� (�����ڲ�������) �ɴ�����迹Դ */

	FSYNC_0			= 0,
	FSYNC_1			= 1		/* ������ */
};

/* ʱ�ӼĴ���bit���� */
enum
{
	CLKDIS_0	= 0x00,		/* ʱ�����ʹ�� ������Ӿ���ʱ������ʹ�ܲ����񵴣� */
	CLKDIS_1	= 0x10,		/* ʱ�ӽ�ֹ �����ⲿ�ṩʱ��ʱ�����ø�λ���Խ�ֹMCK_OUT�������ʱ����ʡ�� */

	/*
		2.4576MHz��CLKDIV=0 ����Ϊ 4.9152MHz ��CLKDIV=1 ����CLK Ӧ�� ��0����
		1MHz ��CLKDIV=0 ���� 2MHz   ��CLKDIV=1 ����CLK ��λӦ��  ��1��
	*/
	CLK_4_9152M = 0x08,
	CLK_2_4576M = 0x00,
	CLK_1M 		= 0x04,
	CLK_2M 		= 0x0C,
	
/*���������������*/
	FS_20HZ     = 0X00,
	FS_25HZ     = 0x01,
	FS_100HZ    = 0x20,
	FS_200HZ    = 0x03,
	FS_50HZ		= 0x04,
	FS_60HZ		= 0x05,
	FS_250HZ	= 0x06,
	FS_500HZ	= 0x07,

	/*
		��ʮ�š����ӳ�Ӧ�������TM7705 ���ȵķ���
			��ʹ����ʱ��Ϊ 2.4576MHz ʱ��ǿ�ҽ��齫ʱ�ӼĴ�����Ϊ 84H,��ʱ�������������Ϊ10Hz,��ÿ0.1S ���һ�������ݡ�
			��ʹ����ʱ��Ϊ 1MHz ʱ��ǿ�ҽ��齫ʱ�ӼĴ�����Ϊ80H, ��ʱ�������������Ϊ4Hz, ��ÿ0.25S ���һ��������
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
static void TM7705_ResetHard(void);		//Ӳ����λ
static void TM7705_SyncSPI(struct TM7705_S* TM7705_tmp_p);
static uint8_t TM7705_ReadByte(uint8_t COM_index, uint8_t SW);
static uint16_t TM7705_Read2Byte(uint8_t COM_index, uint8_t SW);
static uint16_t TM7705_Read3Byte(uint8_t COM_index, uint8_t SW);
static void TM7705_Chip_Select(uint8_t COM_index, uint8_t SW);
static void TM7705_Init_SubFunc(struct TM7705_S* TM7705_tmp_p, uint8_t COM_index, uint8_t SW);
struct TM7705_S* TM7705_Get_Assign(uint8_t COM_index, uint8_t SW);



/*====================оƬ���������ݶ��嵽�˽���==========================*/
/*=====================================================================
=����˵����Ƭѡ��ָ��COM�ĵ�ѹ���������ϵ�7705
=��    ����ָ��COM SW
=====================================================================*/
static void TM7705_Chip_Select(uint8_t COM_index, uint8_t SW)
{
    struct TM7705_S* TM7705_TMP_p;
    TM7705_TMP_p = (struct TM7705_S *)TM7705_Get_Assign(COM_index, SW);    //TM7705_xy  
    //ʧ��ȫ��7705
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

    //Ƭѡָ��7705          TM7705_CS_xy.
    //GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin); 
    GPIO_ResetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin); 
}


/*=====================================================================
=����˵��������COM_index SW���ָ���Ľṹ�����
=��    ����ָ��COM SW
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
    /*==�ź����ų�ʼ��==*/
	GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ�ܶ˿�ʱ��	

    for(COM_index = 1; COM_index <= TM7705_NUM_SUM; COM_index++)
    {
        TM7705_GPIO_CONCAT(COM_index);

        TM7705_TMP_p = TM7705_Get_Assign(COM_index, TM7705_SW_CUR);       //TM7705_Cx
        GPIO_InitStructure.GPIO_Pin  = TM7705_TMP_p->CS_GPIO_Pin;	        //���� 	 CS 
        GPIO_InitStructure.GPIO_Mode = TM7705_TMP_p->CS_GPIO_Mode;
        GPIO_Init(TM7705_TMP_p->CS_GPIO_Port, &GPIO_InitStructure);
        GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);	        

        GPIO_InitStructure.GPIO_Pin  = TM7705_TMP_p->RD_GPIO_Pin;         //����  DRAY
    	GPIO_InitStructure.GPIO_Mode =  TM7705_TMP_p->RD_GPIO_Mode;
    	GPIO_Init(TM7705_TMP_p->RD_GPIO_Port, &GPIO_InitStructure);
    	GPIO_ResetBits(TM7705_TMP_p->RD_GPIO_Port, TM7705_TMP_p->RD_GPIO_Pin);

        TM7705_TMP_p = TM7705_Get_Assign(COM_index, TM7705_SW_VOL);       //TM7705_Vx
        GPIO_InitStructure.GPIO_Pin  = TM7705_TMP_p->CS_GPIO_Pin;	        //���� 	 CS 
        GPIO_InitStructure.GPIO_Mode = TM7705_TMP_p->CS_GPIO_Mode;
        GPIO_Init(TM7705_TMP_p->CS_GPIO_Port, &GPIO_InitStructure);
        GPIO_SetBits(TM7705_TMP_p->CS_GPIO_Port, TM7705_TMP_p->CS_GPIO_Pin);	        

        GPIO_InitStructure.GPIO_Pin  = TM7705_TMP_p->RD_GPIO_Pin;         //����  DRAY
    	GPIO_InitStructure.GPIO_Mode =  TM7705_TMP_p->RD_GPIO_Mode;
    	GPIO_Init(TM7705_TMP_p->RD_GPIO_Port, &GPIO_InitStructure);
    	GPIO_ResetBits(TM7705_TMP_p->RD_GPIO_Port, TM7705_TMP_p->RD_GPIO_Pin);
    }   
}


/*******************************************************************************
 * ������: TM7705_Init
 * ����: ���ݴ����������ʼ��ָ��������7705
 * Input          : TM7705_NUM
 * Return         : void
 * 
*******************************************************************************/
void TM7705_Init(int TM7705_NUM_SUM) //��ʼ������
{
    uint8_t COM_index;
    struct TM7705_S* TM7705_TMP_p;     
    
    TM7705_GPIO_Init(TM7705_NUM_SUM);
		
	SPI2_Init();			//��ʼ��SPI2���裬��drv_spi.h��ʵ��
	SPI2_SetSpeed(SPI_BaudRatePrescaler_2);//����Ϊ18Mʱ��,����ģʽ

    for(COM_index = 1; COM_index <= TM7705_NUM_SUM; COM_index++)
    {

        /******************��ʼ������������7705*****************/
        TM7705_TMP_p = TM7705_Get_Assign(COM_index, TM7705_SW_CUR);    
        TM7705_Init_SubFunc(TM7705_TMP_p, COM_index, TM7705_SW_CUR);
        /******************��ʼ����ѹ������7705*****************/
        TM7705_TMP_p = TM7705_Get_Assign(COM_index, TM7705_SW_VOL);    
        TM7705_Init_SubFunc(TM7705_TMP_p, COM_index, TM7705_SW_VOL);
    }
}	

static void TM7705_Init_SubFunc(struct TM7705_S* TM7705_tmp_p, uint8_t COM_index, uint8_t SW)
{
    //TM7705_CS = 1; 			//TM7705��ʼ����ѡ��
	//TM7705_RST = 0; 
    GPIO_SetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin);  //TM7705_CS = 1; 	//TM7705��ʼ����ѡ��

	delay_ms(10);
	TM7705_ResetHard();		//Ӳ����λ
	delay_ms(5);
	
	TM7705_SyncSPI(TM7705_tmp_p);      //ͬ��SPI�ӿ�ʱ��
	delay_ms(10);
	
	/*����ʱ�ӼĴ���*/
	GPIO_ResetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin); //TM7705_CS = 0; 
	SPI2_ReadWriteByte(REG_CLOCK | WRITE |CH_1); //��дͨ�żĴ�������һ��дʱ�ӼĴ���
	delay_ms(10);
	SPI2_ReadWriteByte(CLKDIS_0 | CLK_4_9152M | FS_500HZ ); //ˢ������250HZ
	delay_ms(10);
	SPI2_ReadWriteByte(REG_CLOCK | WRITE |CH_2); //��дͨ�żĴ�������һ��дʱ�ӼĴ���
	delay_ms(10);
	SPI2_ReadWriteByte(CLKDIS_0 | CLK_4_9152M | FS_500HZ ); //ˢ������250HZ
	delay_ms(10);
	GPIO_SetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin); //TM7705_CS = 1;

	/*ÿ���ϵ����һ����У׼*/
	TM7705_CalibSelf(COM_index, SW, 1);      //�ڲ���У׼
	TM7705_CalibSelf(COM_index, SW, 2);      //�ڲ���У׼
	delay_ms(5);
}


/*===Ӳ����λTm7705оƬ���޳������===*/
static void TM7705_ResetHard(void)	//Ӳ����λ
{
	/*TM7705_RST = 1;
	delay_ms(1);
	TM7705_RST = 0;
	delay_ms(2);
	TM7705_RST = 1;
	delay_ms(1);*/
}

/*=============================================
= ���ܣ�ͬ��TM7705оƬSPI�ӿ�ʱ��
= ˵������������32��1���ɣ���ͬ���ᷢ�����ݴ�λ   
==============================================*/
static void TM7705_SyncSPI(struct TM7705_S* TM7705_tmp_p)      //ͬ��SPI�ӿ�ʱ��
{
	GPIO_ResetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin); //TM7705_CS = 0; 
	SPI2_ReadWriteByte(0xFF);//32��1
	SPI2_ReadWriteByte(0xFF);
	SPI2_ReadWriteByte(0xFF);
	SPI2_ReadWriteByte(0xFF);
    
	GPIO_SetBits(TM7705_tmp_p->CS_GPIO_Port, TM7705_tmp_p->CS_GPIO_Pin); //TM7705_CS = 1;
}

/*====================================================================
=����˵���� �ȴ��ڲ�������ɣ���У׼ʱ��ϳ�����Ҫ�ȴ�
=��	   ���� ��
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
= ����˵��: ������У׼. �ڲ��Զ��̽�AIN+ AIN-У׼0λ���ڲ��̽ӵ�Vref 
			У׼��λ���˺���ִ�й��̽ϳ���ʵ��Լ 180ms	
= ��    ��:  _ch : ADCͨ����1��2
=====================================================================*/

void TM7705_CalibSelf(uint8_t Chip_index,uint8_t SW,uint8_t _ch)
{
    TM7705_Chip_Select(Chip_index, SW);
	if (_ch == 1)
	{
		/* ��У׼CH1 */
		SPI2_ReadWriteByte(REG_SETUP | WRITE | CH_1);	/* дͨ�żĴ�������һ����д���üĴ�����ͨ��1 */		
		SPI2_ReadWriteByte(MD_CAL_SELF | GAIN_1 | UNIPOLAR	|BUF_EN |FSYNC_0);/* ������У׼ *///д���üĴ����������ԡ��л��塢����Ϊ1���˲�����������У׼
		//SPI2_ReadWriteByte(0x4C);
		TM7705_WaitDRDY(Chip_index, SW);	/* �ȴ��ڲ�������� --- ʱ��ϳ���Լ180ms */
	}
	else if (_ch == 2)
	{
		/* ��У׼CH2 */
		SPI2_ReadWriteByte(REG_SETUP | WRITE | CH_2);	/* дͨ�żĴ�������һ����д���üĴ�����ͨ��2 */
		SPI2_ReadWriteByte(MD_CAL_SELF | GAIN_1 | UNIPOLAR |BUF_EN |FSYNC_0);	/* ������У׼ */
		//SPI2_ReadWriteByte(0x44);
		TM7705_WaitDRDY(Chip_index, SW);	/* �ȴ��ڲ��������  --- ʱ��ϳ���Լ180ms */
	}


}

/*=====================================================================
=����˵��������һ��8λ����
=��    �������ض�����ֵ
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
=����˵��������һ��16λ���ݣ����֣�
=��    �������ض���16λ���ݵ�ֵ
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
=����˵��������һ��24λ���ݣ�3�ֽڣ�
=��    �������ض���24λ���ݵ�ֵ
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
=����˵������ȡ������ѹֵ
=��    �������ز���ֵ
=====================================================================*/
uint16_t TM7705_ReadAdc(uint8_t COM_index, uint8_t SW, uint8_t ch)
{
	uint8_t i;
	uint16_t read = 0;
    struct TM7705_S* TM7705_TMP_p = TM7705_Get_Assign(COM_index, SW);    //TM7705_xy 
    TM7705_Chip_Select(COM_index, SW);                      //Ƭѡָ��7705
	/* Ϊ�˱���ͨ���л���ɶ���ʧЧ����2�� */
	for (i = 0; i < 2; i++)
	{
		TM7705_WaitDRDY(COM_index, SW);		/* �ȴ�DRDY����Ϊ0 */		

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
		//TM7705_WaitDRDY();	/* �ȴ��ڲ�������� --- ʱ��ϳ���Լ180ms */
		read = TM7705_Read2Byte(COM_index, SW);
	}
	return read;	
}


 /*====================================================================
 = ����˵��: ��ȡָ��COM��ָ����ѹ���ߵ�����ָ��ͨ����7705����ֵ��
 = ��    ��: COM_index :  ָ��COMx,1-8
            SW        :  ָ����ȡ��ѹ���ߵ����Ĳ���ֵ
            ch        :  ADCͨ����0��ʾȫ����ã�1��2��ָ��ͨ��1��2
 =====================================================================*/

void Conv_True_Vol(uint8_t COM_index, uint8_t SW, uint8_t ch)
{
    uint16_t vol_adc1 = 0, vol_adc2 = 0;
    int      vol_val1 = 0, vol_val2 = 0;
	
    if(1 == ch || 2 == ch)
	    vol_adc1 = TM7705_ReadAdc(COM_index, SW, ch);	/* ִ��ʱ�� 80ms */		
    else if(0 == ch)/* ˫ͨ���л�������ִ��һ��ʵ����Լ 160ms */		
    {
        vol_adc1 = TM7705_ReadAdc(COM_index, SW, 1);	/* ִ��ʱ�� 80ms */	
        vol_adc2 = TM7705_ReadAdc(COM_index, SW, 2);	/* ִ��ʱ�� 80ms */
   
}else           /*�����chֵ����*/
    {
        vol_adc1 = 0;
        vol_adc2 = 0;
    }

   if(1 == ch || 2 == ch)
   {
       	vol_val1 = (vol_adc1 * 5000) / 65535;/* ����ʵ�ʵ�ѹֵ�����ƹ���ģ�������׼ȷ�������У׼ */
      	/* ��ӡADC������� */	
    	printf("SW=%d Chip_index=%d CH%d=%5d (%5dmV) \r\n", SW, COM_index, ch, vol_adc1, vol_val1);
   }
   else if(0 == ch){
        /* ����ʵ�ʵ�ѹֵ�����ƹ���ģ�������׼ȷ�������У׼ */
        vol_val1 = (vol_adc1 * 5000) / 65535;
    	vol_val2 = (vol_adc2 * 5000) / 65535;
    	/* ��ӡADC������� */	
    	printf("SW=%d Chip_index=%d CH1=%5d (%5dmV) CH2=%5d (%5dmV)\r\n", SW, COM_index, vol_adc1, vol_val1, vol_adc2, vol_val2);
   }
   else{
        printf("err: Get_Vol input val wrong \r\n");
   }
}


/*====================================================================
= ����˵��: �����������Ҫ�����ĵ�ѹ·��CHANNEL_NUM������Conv_True_Vol��
            ע�⣬Ĭ�ϴӵ�1·��ʼ˳�����������CHANNEL_NUM=3ʱ��Ĭ����
            ��1-3·�ĵ�ѹ�������
= ��    ��: CHANNEL_NUM :  ��Ҫ�����ĵ�ѹ·����1-16.
           
=====================================================================*/

void Get_Vol(uint8_t CHANNEL_NUM)
{
    int COM_index;
    for(COM_index = 1; COM_index <= CHANNEL_NUM; COM_index++)
    {
        delay_ms(1000);
        Conv_True_Vol(COM_index, TM7705_SW_VOL, TM7705_CH_ALL);       //��ò���ӡ��ѹֵ
    }
}



/*====================================================================
= ����˵��: �����������Ҫ�����ĵ���·��CHANNEL_NUM������Conv_True_Vol��
            ע�⣬Ĭ�ϴӵ�1·��ʼ˳�����������CHANNEL_NUM=3ʱ��Ĭ����
            ��1-3·�ĵ����������
= ��    ��: CHANNEL_NUM :  ��Ҫ�����ĵ���·����1-16.
           
=====================================================================*/

void Get_Cur(uint8_t CHANNEL_NUM)
{
   int COM_index;
   for(COM_index = 1; COM_index <= CHANNEL_NUM; COM_index++)
   {
        delay_ms(1000);
        Conv_True_Vol(COM_index, TM7705_SW_CUR, TM7705_CH_ALL);       //��ò���ӡ����ֵ
   }
}

 

