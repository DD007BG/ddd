#include "bsp_ads1110.h"



/**
 * @brief  设置配置寄存器
 * @param  None
 * @retval None:
 */
void ads1110_Init(void)
{
	u8 ads1110_Config_Register_Set = 0;
  ads1110_I2C_INIT(); // ads1110_I2C init
	ads1110_Config_Register_Set = ADS1110_REG_CONFIG_Single_Conversion_Mode|ADS1110_REG_CONFIG_15SPS_PGA8;
}

/**
 * @brief  写寄存器
 * @param  ads1110_I2CAddress : ADS1110地址和读写位 0bit控制读写
 * @param  reg        :  要操作的寄存器
 * @param  value      :  写入寄存器的值
 * @retval None
 */
u8 writeRegister(uint8_t ads1110_I2CAddress)
{

  ads1110_I2C_Start();
	
  /* Send slave address + write bit */
  ads1110_I2C_Send_Byte(CMD_ADS1110_Write_REG);
	
  /* ACK */
  if (ads1110_I2C_Wait_Ack())
  {
    return 1;
  }

  /* 寄存器config*/
  ads1110_I2C_Send_Byte(ADS1110_REG_CONFIG_Single_Conversion_Mode|ADS1110_REG_CONFIG_15SPS_PGA8); //0x9F
	
  /* ACK */
  if (ads1110_I2C_Wait_Ack())
  {
    return 1;
  }

  ads1110_I2C_Stop();

  /* normal return */
  return 0;
}

/**
 * @brief  读取Output寄存器
 * @param  None
 * @retval None
 */
uint16_t readOutputRegister(void)
{
  u16 ad = 0;
  u8 config = 0;
  u8 high = 0;
  u8 low = 0;
	
  /* ads1110_I2C start */
  ads1110_I2C_Start();
  /* addr and read reg */
  ads1110_I2C_Send_Byte((uint8_t)CMD_ADS1110_Read_REG);
  /* ACK */
  if (ads1110_I2C_Wait_Ack())
  {
    return 1;
  }

  high = ads1110_I2C_Read_Byte(1);
  low = ads1110_I2C_Read_Byte(1);
  config = ads1110_I2C_Read_Byte(1);
	
  ad = high << 8 | low;
  ad = 65536 - ad;
	
  /* ads1110_I2C stop */
  ads1110_I2C_Stop();

  return ad;
}


void Get_Vol_ads1110()
{
  u8 config = 0;
  double temp;
  double ad;
  int COM_index;
  float val = 0;
  ads1110_delay_ms(1000);
  writeRegister(CMD_ADS1110_Write_REG);
	
	
  ad = readOutputRegister();     // 获得并打印电压值 Output Code = -1 * MinCode * PGA * ((VIN+) - (VIN-))/2.048V
  ad = ad * 2048;                //2.048V --> 2048mV
  ad = ad / 8;							     //PGA = 8
  ad = (ad / 32768);				     //15SPS mincode = -32768
  temp = (ad + 0.34) / 0.0413;	 
	
  printf("ads1110 get temp = %f\r\n", temp);


}

/* ------------- ads1110_I2C drive ---------- */

/**
 * @brief  IIC初始化IO
 * @param  None
 * @retval None
 */
void ads1110_I2C_INIT()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_GPIOX, ENABLE);

  GPIO_InitStructure.GPIO_Pin = ads1110_I2C_SCL | ads1110_I2C_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOX_ads1110_I2C, &GPIO_InitStructure);

  ads1110_I2C_SCL_H;
  ads1110_I2C_SDA_H;
}

/**
 * @brief  SDA输出配置
 * @param  None
 * @retval None
 */
void ads1110_I2C_SDA_OUT()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = ads1110_I2C_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOX_ads1110_I2C, &GPIO_InitStructure);
}

/**
 * @brief  SDA输入配置
 * @param  None
 * @retval None
 */
void ads1110_I2C_SDA_IN(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = ads1110_I2C_SDA;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOX_ads1110_I2C, &GPIO_InitStructure);
}

/**
  * @brief  产生起始信号信号
  * @param  None
  * @retval None

SCL   -------|_____


SDA    ---|________

*/
void ads1110_I2C_Start(void)
{
  ads1110_I2C_SDA_OUT();

  ads1110_I2C_SDA_H;
  ads1110_I2C_SCL_H;
  ads1110_delay_us(2);
  ads1110_I2C_SDA_L;
  ads1110_delay_us(2);
  ads1110_I2C_SCL_L;
}

/**
  * @brief  产生停止信号
  * @param  None
  * @retval None

SCL   ------------

SDA   ____|-------

*/
void ads1110_I2C_Stop(void)
{
  ads1110_I2C_SDA_OUT();

  ads1110_I2C_SCL_L;
  ads1110_I2C_SDA_L;
  ads1110_I2C_SCL_H;
  ads1110_delay_us(2);
  ads1110_I2C_SDA_H;
  ads1110_delay_us(2);
}

/**
 * @brief  主机产生应答信号ACK
 * @param  None
 * @retval None
 */
void ads1110_I2C_Ack(void)
{
  ads1110_I2C_SCL_L;
  ads1110_I2C_SDA_OUT();
  ads1110_I2C_SDA_L;
  ads1110_delay_us(2);
  ads1110_I2C_SCL_H;
  ads1110_delay_us(2);
  ads1110_I2C_SCL_L;
}

/**
 * @brief  主机不产生应答信号ACK
 * @param  None
 * @retval None
 */
void ads1110_I2C_NAck(void)
{
  ads1110_I2C_SCL_L;
  ads1110_I2C_SDA_OUT();
  ads1110_I2C_SDA_H;
  ads1110_delay_us(2);
  ads1110_I2C_SCL_H;
  ads1110_delay_us(2);
  ads1110_I2C_SCL_L;
}

/**
  * @brief  等待从机应答信号
  * @param  txd 要发送的字符
  * @retval 返回值：1 接收应答失败 0 接收应答成功
有ACK:
SCL   -------

SDA   ---|____

没有ACK;

SCL   -------

SDA   -------
*/
u8 ads1110_I2C_Wait_Ack(void)
{
  u8 tempTime = 0;

  ads1110_I2C_SDA_IN();

  ads1110_I2C_SDA_H;
  ads1110_delay_us(2);
  ads1110_I2C_SCL_H;
  ads1110_delay_us(2);

  while (GPIO_ReadInputDataBit(GPIOX_ads1110_I2C, ads1110_I2C_SDA))
  {
    tempTime++;
    if (tempTime > 10)
    {
      // ads1110_I2C_Stop();
      return 1;
    }
  }

  ads1110_I2C_SCL_L;
  return 0;
}
// ads1110_I2C 发送一个字节

/**
  * @brief  ads1110_I2C 发送一个字节
  * @param  txd 要发送的字符
  * @retval None:
  *
 SCL __|----|___|-----|___

 SDA __|--------|_________

数据:  |-- 1----| ----0---|

*/
void ads1110_I2C_Send_Byte(u8 txd)
{
  u8 i = 0;

  ads1110_I2C_SDA_OUT();
  ads1110_I2C_SCL_L; // 拉低时钟开始数据传输

  for (i = 0; i < 8; i++)
  {
    if ((txd & 0x80) > 0) // 0x80  1000 0000
      ads1110_I2C_SDA_H;
    else
      ads1110_I2C_SDA_L;

    txd <<= 1;
    ads1110_I2C_SCL_H;
    ads1110_delay_us(2); // 发送数据
    ads1110_I2C_SCL_L;
    ads1110_delay_us(2);
  }
}

/**
 * @brief  ads1110_I2C 读取一个字节
 * @param  ask: 1 读取有应答,0 没有应答
 * @retval None:
 */
u8 ads1110_I2C_Read_Byte(u8 ack)
{
  u8 i = 0, receive = 0;

  ads1110_I2C_SDA_IN();
  for (i = 0; i < 8; i++)
  {
    ads1110_I2C_SCL_L;
    ads1110_delay_us(5);
    ads1110_I2C_SCL_H;
    receive <<= 1;
    if (GPIO_ReadInputDataBit(GPIOX_ads1110_I2C, ads1110_I2C_SDA))
      receive++;
    ads1110_delay_us(5);
  }

  if (ack == 0)
    ads1110_I2C_NAck();
  else
    ads1110_I2C_Ack();

  return receive;
}

/* -------------- delay --------------*/

/**
 * @brief  延时函数，延时us
 * @param  i(0..255)
 * @retval None:
 */
void ads1110_delay_us(u32 i)
{
  u32 temp;
  SysTick->LOAD = 9 * i; // 设置重装数值, 72MHZ时
  SysTick->CTRL = 0X01;  // 使能，减到零是无动作，采用外部时钟源
  SysTick->VAL = 0;      // 清零计数器
  do
  {
    temp = SysTick->CTRL;                           // 读取当前倒计数值
  } while ((temp & 0x01) && (!(temp & (1 << 16)))); // 等待时间到达
  SysTick->CTRL = 0;                                // 关闭计数器
  SysTick->VAL = 0;                                 // 清空计数器
}

/**
 * @brief  延时函数，延时ms
 * @param  i(0..255)
 * @retval None:
 */
void ads1110_delay_ms(u32 i)
{
  u32 temp;
  SysTick->LOAD = 9000 * i; // 设置重装数值, 72MHZ时
  SysTick->CTRL = 0X01;     // 使能，减到零是无动作，采用外部时钟源
  SysTick->VAL = 0;         // 清零计数器
  do
  {
    temp = SysTick->CTRL;                           // 读取当前倒计数值
  } while ((temp & 0x01) && (!(temp & (1 << 16)))); // 等待时间到达
  SysTick->CTRL = 0;                                // 关闭计数器
  SysTick->VAL = 0;                                 // 清空计数器
}
