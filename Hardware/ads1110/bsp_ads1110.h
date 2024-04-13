
#ifndef _bsp_ads1110_H
#define _bsp_ads1110_H
#include "stm32f10x.h"

/* -----ADS1110 Pin ------- */
#define ads1110_I2C_SCL GPIO_Pin_6
#define ads1110_I2C_SDA GPIO_Pin_7
#define GPIOX_ads1110_I2C GPIOB
#define RCC_GPIOX RCC_APB2Periph_GPIOB



//ads1110_I2C ADDRESS/BITS
#define ADS1010_ADDRESS (0x90) // 1001 000 (ADDR = GND)
#define CMD_ADS1110_Write_REG 0x90 /* 写 ads1110设备*/
#define CMD_ADS1110_Read_REG 0x91  /* 读 ads1110设备 */


// 运行状态/单次转换开始
#define ADS1110_REG_CONFIG_Single_Conversion_Mode (0x90)			 //conversion start  +  Single Conversion Mode
#define ADS1110_REG_CONFIG_Continuous_Conversion_Mode  (0x80)  //conversion start  +  Continus Conversion Mode



// Config Register数据速率+可编程增益放大器配置
#define ADS1110_REG_CONFIG_15SPS_PGA8 (0x0F) // 15SPS+PGA8
#define ADS1110_REG_CONFIG_15SPS_PGA4 (0x0E) // 15SPS+PGA4
#define ADS1110_REG_CONFIG_15SPS_PGA2 (0x0D) // 15SPS+PGA2
#define ADS1110_REG_CONFIG_15SPS_PGA1 (0x0C) // 15SPS+PGA1
#define ADS1110_REG_CONFIG_30SPS_PGA8 (0x0B) // 15SPS+PGA8
#define ADS1110_REG_CONFIG_30SPS_PGA4 (0x0A) // 15SPS+PGA4
#define ADS1110_REG_CONFIG_30SPS_PGA2 (0x09) // 15SPS+PGA2
#define ADS1110_REG_CONFIG_30SPS_PGA1 (0x08) // 15SPS+PGA1
#define ADS1110_REG_CONFIG_60SPS_PGA8 (0x07) // 15SPS+PGA8
#define ADS1110_REG_CONFIG_60SPS_PGA4 (0x06) // 15SPS+PGA4
#define ADS1110_REG_CONFIG_60SPS_PGA2 (0x05) // 15SPS+PGA2
#define ADS1110_REG_CONFIG_60SPS_PGA1 (0x04) // 15SPS+PGA1
#define ADS1110_REG_CONFIG_240SPS_PGA8 (0x03) // 15SPS+PGA8
#define ADS1110_REG_CONFIG_240SPS_PGA4 (0x02) // 15SPS+PGA4
#define ADS1110_REG_CONFIG_240SPS_PGA2 (0x01) // 15SPS+PGA2
#define ADS1110_REG_CONFIG_240SPS_PGA1 (0x00) // 15SPS+PGA1


/* public: */
void ads1110_Init(void);

/* privata: */
uint16_t readConvertRegister(void);

/* --------ads1110_I2C --------*/

#define ads1110_I2C_SCL_H GPIO_SetBits(GPIOX_ads1110_I2C, ads1110_I2C_SCL)
#define ads1110_I2C_SCL_L GPIO_ResetBits(GPIOX_ads1110_I2C, ads1110_I2C_SCL)

#define ads1110_I2C_SDA_H GPIO_SetBits(GPIOX_ads1110_I2C, ads1110_I2C_SDA)
#define ads1110_I2C_SDA_L GPIO_ResetBits(GPIOX_ads1110_I2C, ads1110_I2C_SDA)

/* 声明全局函数 */
void ads1110_I2C_INIT(void);
void ads1110_I2C_SDA_OUT(void);
void ads1110_I2C_SDA_IN(void);
void ads1110_I2C_Start(void);
void ads1110_I2C_Stop(void);
void ads1110_I2C_Ack(void);
void ads1110_I2C_NAck(void);
u8 ads1110_I2C_Wait_Ack(void);
void ads1110_I2C_Send_Byte(u8 txd);
u8 ads1110_I2C_Read_Byte(u8 ack);

/* ----- delay -------*/
void ads1110_delay_us(u32 i);
void ads1110_delay_ms(u32 i);

#endif
