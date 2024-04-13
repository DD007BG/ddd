#include "bsp_ads1110.h"



/**
 * @brief  �������üĴ���
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
 * @brief  д�Ĵ���
 * @param  ads1110_I2CAddress : ADS1110��ַ�Ͷ�дλ 0bit���ƶ�д
 * @param  reg        :  Ҫ�����ļĴ���
 * @param  value      :  д��Ĵ�����ֵ
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

  /* �Ĵ���config*/
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
 * @brief  ��ȡOutput�Ĵ���
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
	
	
  ad = readOutputRegister();     // ��ò���ӡ��ѹֵ Output Code = -1 * MinCode * PGA * ((VIN+) - (VIN-))/2.048V
  ad = ad * 2048;                //2.048V --> 2048mV
  ad = ad / 8;							     //PGA = 8
  ad = (ad / 32768);				     //15SPS mincode = -32768
  temp = (ad + 0.34) / 0.0413;	 
	
  printf("ads1110 get temp = %f\r\n", temp);


}

/* ------------- ads1110_I2C drive ---------- */

/**
 * @brief  IIC��ʼ��IO
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
 * @brief  SDA�������
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
 * @brief  SDA��������
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
  * @brief  ������ʼ�ź��ź�
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
  * @brief  ����ֹͣ�ź�
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
 * @brief  ��������Ӧ���ź�ACK
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
 * @brief  ����������Ӧ���ź�ACK
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
  * @brief  �ȴ��ӻ�Ӧ���ź�
  * @param  txd Ҫ���͵��ַ�
  * @retval ����ֵ��1 ����Ӧ��ʧ�� 0 ����Ӧ��ɹ�
��ACK:
SCL   -------

SDA   ---|____

û��ACK;

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
// ads1110_I2C ����һ���ֽ�

/**
  * @brief  ads1110_I2C ����һ���ֽ�
  * @param  txd Ҫ���͵��ַ�
  * @retval None:
  *
 SCL __|----|___|-----|___

 SDA __|--------|_________

����:  |-- 1----| ----0---|

*/
void ads1110_I2C_Send_Byte(u8 txd)
{
  u8 i = 0;

  ads1110_I2C_SDA_OUT();
  ads1110_I2C_SCL_L; // ����ʱ�ӿ�ʼ���ݴ���

  for (i = 0; i < 8; i++)
  {
    if ((txd & 0x80) > 0) // 0x80  1000 0000
      ads1110_I2C_SDA_H;
    else
      ads1110_I2C_SDA_L;

    txd <<= 1;
    ads1110_I2C_SCL_H;
    ads1110_delay_us(2); // ��������
    ads1110_I2C_SCL_L;
    ads1110_delay_us(2);
  }
}

/**
 * @brief  ads1110_I2C ��ȡһ���ֽ�
 * @param  ask: 1 ��ȡ��Ӧ��,0 û��Ӧ��
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
 * @brief  ��ʱ��������ʱus
 * @param  i(0..255)
 * @retval None:
 */
void ads1110_delay_us(u32 i)
{
  u32 temp;
  SysTick->LOAD = 9 * i; // ������װ��ֵ, 72MHZʱ
  SysTick->CTRL = 0X01;  // ʹ�ܣ����������޶����������ⲿʱ��Դ
  SysTick->VAL = 0;      // ���������
  do
  {
    temp = SysTick->CTRL;                           // ��ȡ��ǰ������ֵ
  } while ((temp & 0x01) && (!(temp & (1 << 16)))); // �ȴ�ʱ�䵽��
  SysTick->CTRL = 0;                                // �رռ�����
  SysTick->VAL = 0;                                 // ��ռ�����
}

/**
 * @brief  ��ʱ��������ʱms
 * @param  i(0..255)
 * @retval None:
 */
void ads1110_delay_ms(u32 i)
{
  u32 temp;
  SysTick->LOAD = 9000 * i; // ������װ��ֵ, 72MHZʱ
  SysTick->CTRL = 0X01;     // ʹ�ܣ����������޶����������ⲿʱ��Դ
  SysTick->VAL = 0;         // ���������
  do
  {
    temp = SysTick->CTRL;                           // ��ȡ��ǰ������ֵ
  } while ((temp & 0x01) && (!(temp & (1 << 16)))); // �ȴ�ʱ�䵽��
  SysTick->CTRL = 0;                                // �رռ�����
  SysTick->VAL = 0;                                 // ��ռ�����
}
