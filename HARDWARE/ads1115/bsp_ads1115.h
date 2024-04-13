
#ifndef _bsp_ads1115_H
#define _bsp_ads1115_H
#include "stm32f10x.h"

/* -----ADS1115 Pin ------- */
<<<<<<< HEAD
#define ads1115_I2C_SCL GPIO_Pin_1	  
#define ads1115_I2C_SDA GPIO_Pin_0	  
#define GPIOX_ads1115_I2C GPIOC
#define RCC_GPIOX  RCC_APB2Periph_GPIOC
=======
#define ADS1115_I2C_COM1_SCL            GPIO_Pin_1	  
#define ADS1115_I2C_COM1_SDA            GPIO_Pin_0	  
#define ADS1115_I2C_COM1_SCL_GPIOX      GPIOC
#define ADS1115_I2C_COM1_SDA_GPIOX      GPIOC
//#define ADS1115_I2C_COM1_RCC_GPIOX      RCC_APB2Periph_GPIOC

#define ADS1115_I2C_COM2_SCL            GPIO_Pin_2  
#define ADS1115_I2C_COM2_SDA            GPIO_Pin_3
#define ADS1115_I2C_COM2_SCL_GPIOX      GPIOC
#define ADS1115_I2C_COM2_SDA_GPIOX      GPIOC   
//#define ADS1115_I2C_COM2_RCC_GPIOX      RCC_APB2Periph_GPIOC

#define ADS1115_I2C_COM3_SCL            GPIO_Pin_0 	  
#define ADS1115_I2C_COM3_SDA            GPIO_Pin_5 
#define ADS1115_I2C_COM3_SCL_GPIOX      GPIOB
#define ADS1115_I2C_COM3_SDA_GPIOX      GPIOC
//#define ADS1115_I2C_COM3_RCC_GPIOX      

#define ADS1115_I2C_COM4_SCL            GPIO_Pin_11
#define ADS1115_I2C_COM4_SDA            GPIO_Pin_10
#define ADS1115_I2C_COM4_SCL_GPIOX      GPIOB
#define ADS1115_I2C_COM4_SDA_GPIOX      GPIOB
//#define ADS1115_I2C_COM4_RCC_GPIOX   

#define ADS1115_I2C_COM5_SCL            GPIO_Pin_8   
#define ADS1115_I2C_COM5_SDA            GPIO_Pin_7 
#define ADS1115_I2C_COM5_SCL_GPIOX      GPIOC        
#define ADS1115_I2C_COM5_SDA_GPIOX      GPIOC
//#define ADS1115_I2C_COM5_RCC_GPIOX   

#define ADS1115_I2C_COM6_SCL            GPIO_Pin_12
#define ADS1115_I2C_COM6_SDA            GPIO_Pin_11
#define ADS1115_I2C_COM6_SCL_GPIOX      GPIOA
#define ADS1115_I2C_COM6_SDA_GPIOX      GPIOA
//#define ADS1115_I2C_COM6_RCC_GPIOX   

#define ADS1115_I2C_COM7_SCL            GPIO_Pin_12  
#define ADS1115_I2C_COM7_SDA            GPIO_Pin_11
#define ADS1115_I2C_COM7_SCL_GPIOX      GPIOC
#define ADS1115_I2C_COM7_SDA_GPIOX      GPIOC 
//#define ADS1115_I2C_COM7_RCC_GPIOX   

#define ADS1115_I2C_COM8_SCL            GPIO_Pin_2
#define ADS1115_I2C_COM8_SDA            GPIO_Pin_3
#define ADS1115_I2C_COM8_SCL_GPIOX      GPIOD
#define ADS1115_I2C_COM8_SDA_GPIOX      GPIOB
//#define ADS1115_I2C_COM8_RCC_GPIOX 

/* --------ads1115_I2C --------*/

#define ADS1115_I2C_COM1_SCL_H GPIO_SetBits(ADS1115_I2C_COM1_SCL_GPIOX,ADS1115_I2C_COM1_SCL)
#define ADS1115_I2C_COM1_SCL_L GPIO_ResetBits(ADS1115_I2C_COM1_SCL_GPIOX,ADS1115_I2C_COM1_SCL)
#define ADS1115_I2C_COM1_SDA_H GPIO_SetBits(ADS1115_I2C_COM1_SDA_GPIOX,ADS1115_I2C_COM1_SDA)
#define ADS1115_I2C_COM1_SDA_L GPIO_ResetBits(ADS1115_I2C_COM1_SDA_GPIOX,ADS1115_I2C_COM1_SDA)

#define ADS1115_I2C_COM2_SCL_H GPIO_SetBits(ADS1115_I2C_COM2_SCL_GPIOX,ADS1115_I2C_COM2_SCL)
#define ADS1115_I2C_COM2_SCL_L GPIO_ResetBits(ADS1115_I2C_COM2_SCL_GPIOX,ADS1115_I2C_COM2_SCL)
#define ADS1115_I2C_COM2_SDA_H GPIO_SetBits(ADS1115_I2C_COM2_SDA_GPIOX,ADS1115_I2C_COM2_SDA)
#define ADS1115_I2C_COM2_SDA_L GPIO_ResetBits(ADS1115_I2C_COM2_SDA_GPIOX,ADS1115_I2C_COM2_SDA)

#define ADS1115_I2C_COM3_SCL_H GPIO_SetBits(ADS1115_I2C_COM3_SCL_GPIOX,ADS1115_I2C_COM3_SCL)
#define ADS1115_I2C_COM3_SCL_L GPIO_ResetBits(ADS1115_I2C_COM3_SCL_GPIOX,ADS1115_I2C_COM3_SCL)
#define ADS1115_I2C_COM3_SDA_H GPIO_SetBits(ADS1115_I2C_COM3_SDA_GPIOX,ADS1115_I2C_COM3_SDA)
#define ADS1115_I2C_COM3_SDA_L GPIO_ResetBits(ADS1115_I2C_COM3_SDA_GPIOX,ADS1115_I2C_COM3_SDA)

#define ADS1115_I2C_COM4_SCL_H GPIO_SetBits(ADS1115_I2C_COM4_SCL_GPIOX,ADS1115_I2C_COM4_SCL)
#define ADS1115_I2C_COM4_SCL_L GPIO_ResetBits(ADS1115_I2C_COM4_SCL_GPIOX,ADS1115_I2C_COM4_SCL)
#define ADS1115_I2C_COM4_SDA_H GPIO_SetBits(ADS1115_I2C_COM4_SDA_GPIOX,ADS1115_I2C_COM4_SDA)
#define ADS1115_I2C_COM4_SDA_L GPIO_ResetBits(ADS1115_I2C_COM4_SDA_GPIOX,ADS1115_I2C_COM4_SDA)

#define ADS1115_I2C_COM5_SCL_H GPIO_SetBits(ADS1115_I2C_COM5_SCL_GPIOX,ADS1115_I2C_COM5_SCL)
#define ADS1115_I2C_COM5_SCL_L GPIO_ResetBits(ADS1115_I2C_COM5_SCL_GPIOX,ADS1115_I2C_COM5_SCL)
#define ADS1115_I2C_COM5_SDA_H GPIO_SetBits(ADS1115_I2C_COM5_SDA_GPIOX,ADS1115_I2C_COM5_SDA)
#define ADS1115_I2C_COM5_SDA_L GPIO_ResetBits(ADS1115_I2C_COM5_SDA_GPIOX,ADS1115_I2C_COM5_SDA)

#define ADS1115_I2C_COM6_SCL_H GPIO_SetBits(ADS1115_I2C_COM6_SCL_GPIOX,ADS1115_I2C_COM6_SCL)
#define ADS1115_I2C_COM6_SCL_L GPIO_ResetBits(ADS1115_I2C_COM6_SCL_GPIOX,ADS1115_I2C_COM6_SCL)
#define ADS1115_I2C_COM6_SDA_H GPIO_SetBits(ADS1115_I2C_COM6_SDA_GPIOX,ADS1115_I2C_COM6_SDA)
#define ADS1115_I2C_COM6_SDA_L GPIO_ResetBits(ADS1115_I2C_COM6_SDA_GPIOX,ADS1115_I2C_COM6_SDA)

#define ADS1115_I2C_COM7_SCL_H GPIO_SetBits(ADS1115_I2C_COM7_SCL_GPIOX,ADS1115_I2C_COM7_SCL)
#define ADS1115_I2C_COM7_SCL_L GPIO_ResetBits(ADS1115_I2C_COM7_SCL_GPIOX,ADS1115_I2C_COM7_SCL)
#define ADS1115_I2C_COM7_SDA_H GPIO_SetBits(ADS1115_I2C_COM7_SDA_GPIOX,ADS1115_I2C_COM7_SDA)
#define ADS1115_I2C_COM7_SDA_L GPIO_ResetBits(ADS1115_I2C_COM7_SDA_GPIOX,ADS1115_I2C_COM7_SDA)

#define ADS1115_I2C_COM8_SCL_H GPIO_SetBits(ADS1115_I2C_COM8_SCL_GPIOX,ADS1115_I2C_COM8_SCL)
#define ADS1115_I2C_COM8_SCL_L GPIO_ResetBits(ADS1115_I2C_COM8_SCL_GPIOX,ADS1115_I2C_COM8_SCL)
#define ADS1115_I2C_COM8_SDA_H GPIO_SetBits(ADS1115_I2C_COM8_SDA_GPIOX,ADS1115_I2C_COM8_SDA)
#define ADS1115_I2C_COM8_SDA_L GPIO_ResetBits(ADS1115_I2C_COM8_SDA_GPIOX,ADS1115_I2C_COM8_SDA)





>>>>>>> 6f2502c (update_v1)

/*=========================================================================
    ads1115_I2C channel enable
    -----------------------------------------------------------------------*/
    #define ADS1015_EN_CHANNEL_0    0
    #define ADS1015_EN_CHANNEL_1    1
    #define ADS1015_EN_CHANNEL_2    2
    #define ADS1015_EN_CHANNEL_3    3
/*=========================================================================*/


/*=========================================================================
    ads1115_I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADS1015_ADDRESS                 (0x90)    // 1001 000 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
    #define ADS1115_CONVERSIONDELAY         (8)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER -- ֻд
    -----------------------------------------------------------------------*/
    #define ADS1015_REG_POINTER_MASK        (0x03)
    #define ADS1015_REG_POINTER_CONVERT     (0x00)  // ת���Ĵ���
    #define ADS1015_REG_POINTER_CONFIG      (0x01)  // ���üĴ���
    #define ADS1015_REG_POINTER_LOWTHRESH   (0x02)  // LOWTHRESH�Ĵ���
    #define ADS1015_REG_POINTER_HITHRESH    (0x03)  // HITHRESH�Ĵ���
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER -- ��д
    -----------------------------------------------------------------------*/
		// ����״̬/����ת����ʼ
    #define ADS1015_REG_CONFIG_OS_MASK      (0x8000)  
    #define ADS1015_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: ����״̬/����ת����ʼ
    #define ADS1015_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 �豸Ŀǰ����ִ��ת��
    #define ADS1015_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 �豸Ŀǰ����ִ��ת��

		// �����·����������
    #define ADS1015_REG_CONFIG_MUX_MASK     (0x7000)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // AIN P = AIN0 �� AIN N = AIN1 (default)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // AIN P = AIN0 �� AIN N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // AIN P = AIN1 �� AIN N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // AIN P = AIN2 �� AIN N = AIN3
    #define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // AIN P = AIN0 �� AIN N = GND
    #define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // AIN P = AIN1 �� AIN N = GND
    #define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // AIN P = AIN2 �� AIN N = GND
    #define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // AIN P = AIN3 �� AIN N = GND

		// �ɱ������Ŵ�������
    #define ADS1015_REG_CONFIG_PGA_MASK     (0x0E00)
    #define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
    #define ADS1015_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
    #define ADS1015_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
    #define ADS1015_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
    #define ADS1015_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
    #define ADS1015_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

		// ����ģʽ
    #define ADS1015_REG_CONFIG_MODE_MASK    (0x0100)
    #define ADS1015_REG_CONFIG_MODE_CONTIN  (0x0000)  // ����ת��ģʽ
    #define ADS1015_REG_CONFIG_MODE_SINGLE  (0x0100)  // ���絥��ģʽ(default)

		// ��������
    #define ADS1015_REG_CONFIG_DR_MASK      (0x00E0)  
    #define ADS1015_REG_CONFIG_DR_128SPS    (0x0000)  // 128 samples per second
    #define ADS1015_REG_CONFIG_DR_250SPS    (0x0020)  // 250 samples per second
    #define ADS1015_REG_CONFIG_DR_490SPS    (0x0040)  // 490 samples per second
    #define ADS1015_REG_CONFIG_DR_920SPS    (0x0060)  // 920 samples per second
    #define ADS1015_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
    #define ADS1015_REG_CONFIG_DR_2400SPS   (0x00A0)  // 2400 samples per second
    #define ADS1015_REG_CONFIG_DR_3300SPS   (0x00C0)  // 3300 samples per second

		// �Ƚ���ģʽ
    #define ADS1015_REG_CONFIG_CMODE_MASK   (0x0010)
    #define ADS1015_REG_CONFIG_CMODE_TRAD   (0x0000)  // ���г��͵Ĵ�ͳ�Ƚ��� (default)
    #define ADS1015_REG_CONFIG_CMODE_WINDOW (0x0010)  // ���ڱȽ���

		// �Ƚ�������
    #define ADS1015_REG_CONFIG_CPOL_MASK    (0x0008)
    #define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY �͵�ƽ��Ч (default)
    #define ADS1015_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY �ߵ�ƽ��Ч

		// ����Ƚ���
    #define ADS1015_REG_CONFIG_CLAT_MASK    (0x0004)  // ��λ������ALERT/RDY һ�������Ƿ���λ
    #define ADS1015_REG_CONFIG_CLAT_NONLAT  (0x0000)  // ������Ƚ��� (default)
    #define ADS1015_REG_CONFIG_CLAT_LATCH   (0x0004)  // ����Ƚ���

		// �Ƚ������кͽ���
    #define ADS1015_REG_CONFIG_CQUE_MASK    (0x0003)
    #define ADS1015_REG_CONFIG_CQUE_1CONV   (0x0000)  // ALERT/RDY ��һ��ת��������
    #define ADS1015_REG_CONFIG_CQUE_2CONV   (0x0001)  // ALERT/RDY ��һ��ת��������
    #define ADS1015_REG_CONFIG_CQUE_4CONV   (0x0002)  // ALERT/RDY ��һ��ת��������
    #define ADS1015_REG_CONFIG_CQUE_NONE    (0x0003)  // ALERT/RDY ���ñȽ���(default)
/*=========================================================================*/
typedef enum
{
  GAIN_TWOTHIRDS    = ADS1015_REG_CONFIG_PGA_6_144V,  // 0.1875mV
  GAIN_ONE          = ADS1015_REG_CONFIG_PGA_4_096V,  // 0.125mV
  GAIN_TWO          = ADS1015_REG_CONFIG_PGA_2_048V,  // 0.0625mV
  GAIN_FOUR         = ADS1015_REG_CONFIG_PGA_1_024V,  // 0.03125mV
  GAIN_EIGHT        = ADS1015_REG_CONFIG_PGA_0_512V,  // 0.015625mV
  GAIN_SIXTEEN      = ADS1015_REG_CONFIG_PGA_0_256V  //  0.0078125mV
} adsGain_t;


<<<<<<< HEAD
#define CMD_ADS1115_ADDR        0x90   /* ADS1115���豸��ַ,��ǰ��д */
#define CMD_ADS1115_Write_REG   0x90   /* д ADS1115�豸*/
#define CMD_ADS1115_Read_REG    0x91   /* �� ADS1115�豸 */


/* public: */
void  ads1115_Init(void);
void  SetGain(adsGain_t gain);
adsGain_t GetGain(void);
uint16_t GetAds1115Values(void);
/* privata: */
u8  ConfigeRegister(unsigned char channel);
u8  PointRegister (uint8_t ads1115_I2CAddress, uint8_t reg);
u8 writeRegister(uint8_t ads1115_I2CAddress, uint8_t reg, uint16_t value);
uint16_t readConvertRegister(void);
=======
#define CMD_ADS1115_ADDR1        0x90   /* ADS1115���豸��ַ,��ǰ��д */
#define CMD_ADS1115_Write_REG1   0x90   /* д ADS1115�豸*/
#define CMD_ADS1115_Read_REG1    0x91   /* �� ADS1115�豸 */

#define CMD_ADS1115_ADDR2        0x92   /* ADS1115���豸��ַ,��ǰ��д */
#define CMD_ADS1115_Write_REG2   0x92   /* д ADS1115�豸*/
#define CMD_ADS1115_Read_REG2    0x93   /* �� ADS1115�豸 */



/* public: */
void  ads1115_Init(unsigned int COM_Address);
void  SetGain(adsGain_t gain);
adsGain_t GetGain(void);
uint16_t GetAds1115Values(unsigned int COM_Address,u16 Ads_Address);
/* privata: */
u8  ConfigeRegister(unsigned int COM_Address,u16 Ads_Address, unsigned char channel);
u8  PointRegister (unsigned int COM_Address,uint8_t ads1115_I2CAddress, uint8_t reg);
u8 writeRegister(unsigned int COM_Address,uint8_t ads1115_I2CAddress, uint8_t reg, uint16_t value);
uint16_t readConvertRegister(unsigned int COM_Address,u16 Ads_Adress);
>>>>>>> 6f2502c (update_v1)


/* --------ads1115_I2C --------*/

#define ads1115_I2C_SCL_H GPIO_SetBits(GPIOX_ads1115_I2C,ads1115_I2C_SCL)
#define ads1115_I2C_SCL_L GPIO_ResetBits(GPIOX_ads1115_I2C,ads1115_I2C_SCL)

#define ads1115_I2C_SDA_H GPIO_SetBits(GPIOX_ads1115_I2C,ads1115_I2C_SDA)
#define ads1115_I2C_SDA_L GPIO_ResetBits(GPIOX_ads1115_I2C,ads1115_I2C_SDA)

/* ����ȫ�ֺ��� */
<<<<<<< HEAD
void ads1115_I2C_INIT(void);
void ads1115_I2C_SDA_OUT(void);
void ads1115_I2C_SDA_IN(void);
void ads1115_I2C_Start(void);
void ads1115_I2C_Stop(void);
void ads1115_I2C_Ack(void);
void ads1115_I2C_NAck(void);
u8   ads1115_I2C_Wait_Ack(void);
void ads1115_I2C_Send_Byte(u8 txd);
u8   ads1115_I2C_Read_Byte(u8 ack);
=======
void ads1115_I2C_INIT(unsigned int COM_Address);
void ads1115_I2C_SDA_OUT(unsigned int COM_Address);
void ads1115_I2C_SDA_IN(unsigned int COM_Address);
void ads1115_I2C_Start(unsigned int COM_Address);
void ads1115_I2C_Stop(unsigned int COM_Address);
void ads1115_I2C_Ack(unsigned int COM_Address);
void ads1115_I2C_NAck(unsigned int COM_Address);
u8   ads1115_I2C_Wait_Ack(unsigned int COM_Address);
void ads1115_I2C_Send_Byte(unsigned int COM_Address,u8 txd);
u8   ads1115_I2C_Read_Byte(unsigned int COM_Address,u8 ack);
>>>>>>> 6f2502c (update_v1)

/* ----- delay -------*/
void ads1115_delay_us(u32 i);
void ads1115_delay_ms(u32 i);


#endif
