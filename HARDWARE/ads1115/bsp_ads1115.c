#include "bsp_ads1115.h"

<<<<<<< HEAD

/* 增益 */
adsGain_t m_gain;
/* 测试通道 */
//u8 m_channel = 0;


 
/**
  * @brief  设置配置寄存器
  * @param  None
  * @retval None:
  */
void ads1115_Init(void)
{
  

  ads1115_I2C_INIT();  // ads1115_I2C init

  /* 增益 */
  m_gain = GAIN_ONE;
  
  
  /* config the config reg */
  /*
  if (ConfigeRegister(m_channel))
  {
    // deal error 
    printf("init configreg error\r\n");
  }else
    printf("ads1115_Init:init configreg success\r\n");
  */
}

/**
  * @brief  设置配置寄存器
  * @param  channel : 通道X(0..3)
  * @retval None:
  */
 u8  ConfigeRegister(unsigned char channel)
{

      /* Set config reg valus */
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // 禁用比较器              (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // 非锁存比较器            (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // ALERT/RDY 低电平有效    (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // 具有迟滞的传统比较器     (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default val)
                    ADS1015_REG_CONFIG_MODE_CONTIN;   // Single-shot mode       
  
  /* Set PGA/voltage range */
  config |= m_gain;
  
  /* Set 'start single-conversion' bit */
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  /* Set single-ended input channel */
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }
  

   /* Write config register to the ADC */
   if (writeRegister(CMD_ADS1115_Write_REG, ADS1015_REG_POINTER_CONFIG, config))
   {
     
     // deal no ack error
     return 1;
   }
   
   /* normal return */
   return 0;

}

  

/**
  * @brief  指针寄存器
  * @param  ads1115_I2CAddress : ADS115地址和读写位 0bit控制读写
  * @param  reg        :  要操作的寄存器
  * @retval None:
  */
 u8  PointRegister (uint8_t ads1115_I2CAddress, uint8_t reg)
{  
    
  /* ads1115_I2C start */
    ads1115_I2C_Start(); 
  
  /* sent addr and write reg */
    ads1115_I2C_Send_Byte(ads1115_I2CAddress);
  /* ack */
   if (ads1115_I2C_Wait_Ack())
   {
      return 1;
   }
   
   /* Point reg is seted convert reg */
   ads1115_I2C_Send_Byte(reg);
   /* ack */
    if (ads1115_I2C_Wait_Ack())
   {
       return 1;
   }
   
   /* ads1115_I2C stop */
  ads1115_I2C_Stop();
   
  /* normal rerturn */
  return 0;
 }

/**
  * @brief  设置增益
  * @param  gain : 增益 
  * @retval None:
  */
 void  SetGain(adsGain_t gain)
 {
   m_gain = gain;
 }
 
/**
  * @brief  取得增益
  * @param  None
  * @retval None
  */
 adsGain_t GetGain(void)
 {
   return m_gain;
 }
 
 
/**
  * @brief  写寄存器
  * @param  ads1115_I2CAddress : ADS115地址和读写位 0bit控制读写
  * @param  reg        :  要操作的寄存器
  * @param  value      :  写入寄存器的值
  * @retval None
  */
  u8 writeRegister(uint8_t ads1115_I2CAddress, uint8_t reg, uint16_t value) 
 {
   
  ads1115_I2C_Start(); 
   /* 地址和读写地址 */
  ads1115_I2C_Send_Byte(ads1115_I2CAddress);
   /* ACK */
  if (ads1115_I2C_Wait_Ack())
   {
     return 1;
   }
   
   /* 寄存器地址 */
  ads1115_I2C_Send_Byte(reg);
    /* ACK */
   if (ads1115_I2C_Wait_Ack())
   {
     return 1;
   }
   
   /* Lo_thresh寄存器 */
  ads1115_I2C_Send_Byte((uint8_t)(value>>8));
   /* ACK */
   if (ads1115_I2C_Wait_Ack())
   {
     return 1;
   }
   
   /* Hi_thresh寄存器 */
  ads1115_I2C_Send_Byte((uint8_t)(value & 0xFF));
   /* ACK */
   if (ads1115_I2C_Wait_Ack())
   {
     return 1;
   }
  ads1115_I2C_Stop();
   
   /* normal return */
   return 0;
}
 


/**
  * @brief  读取convert寄存器
  * @param  None
  * @retval None
  */
 uint16_t readConvertRegister(void) 
{ 
  u8 high = 0;
  u8 low = 0;
  /* ads1115_I2C start */
  ads1115_I2C_Start(); 
  /* addr and read reg */
  ads1115_I2C_Send_Byte((uint8_t)CMD_ADS1115_Read_REG);
   /* ACK */
   if (ads1115_I2C_Wait_Ack())
   {
     return 1;
   }
   
   high = ads1115_I2C_Read_Byte(1);
   low = ads1115_I2C_Read_Byte(1);
   
   /* ads1115_I2C stop */
  ads1115_I2C_Stop();

  return (high<<8 | low);
}

/**
  * @brief  获得adc的值
  * @param  None
  * @retval None
  */
uint16_t GetAds1115Values(void)
{
	  /* point to convert reg .ready to read adc values */
  if (PointRegister(CMD_ADS1115_Write_REG,ADS1015_REG_POINTER_CONVERT))
  {
    // deal error 
    printf("init PointRegister error\r\n");
  }
  
   return readConvertRegister();
}


double ads1115_get_voltage_val(u8 channel)
{
    double val;
    int16_t ad_val;

    if (ConfigeRegister(channel))
    {
        // deal error 
        printf("init channel %d configreg error\r\n", channel);
    }

    ad_val=GetAds1115Values();
    if((ad_val==0x7FFF)|(ad_val==0X8000))//是否超量程了
    {
        ad_val=0;
        printf("over PGA\r\n");
    }

    switch(m_gain)//量程对应的分辨率
    {
        case(GAIN_TWOTHIRDS):
            val=(double)ad_val*187.5/1000000.0;//
        break;
        case(GAIN_ONE):
            val=(double)ad_val*125/1000000.0;
        break;
        case(GAIN_TWO):
            val=(double)ad_val*62.5/1000000.0;
        break;
        case(GAIN_FOUR):
            val=(double)ad_val*31.25/1000000.0;
        break;
        case(GAIN_EIGHT):
            val=(double)ad_val*15.625/1000000.0;
        break;
        case(GAIN_SIXTEEN):
            val=(double)ad_val*7.8125/1000000.0;
        break;
    }
    return val;

}


void Get_Vol_ads1115(uint8_t CHANNEL_NUM)
{
    int COM_index;
    float val;
    //for(COM_index = 0; COM_index <= CHANNEL_NUM; COM_index++)
    {
        delay_ms(1000);
        val = ads1115_get_voltage_val(CHANNEL_NUM);       //获得并打印电压值
        printf("ads1115 get vol   chaneel %d = %f\r\n", CHANNEL_NUM, val);
    }
}


/* ------------- ads1115_I2C drive ---------- */


/**
  * @brief  IIC初始化IO
  * @param  None
  * @retval None
  */
void ads1115_I2C_INIT()
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  
  RCC_APB2PeriphClockCmd(RCC_GPIOX,ENABLE);

  GPIO_InitStructure.GPIO_Pin=ads1115_I2C_SCL|ads1115_I2C_SDA;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_Init(GPIOX_ads1115_I2C,&GPIO_InitStructure);

  ads1115_I2C_SCL_H;
  ads1115_I2C_SDA_H;
}

/**
  * @brief  SDA输出配置
  * @param  None
  * @retval None
  */
void ads1115_I2C_SDA_OUT()
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  GPIO_InitStructure.GPIO_Pin=ads1115_I2C_SDA;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_Init(GPIOX_ads1115_I2C,&GPIO_InitStructure);
}

/**
  * @brief  SDA输入配置
  * @param  None
  * @retval None
  */
void ads1115_I2C_SDA_IN(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  
  GPIO_InitStructure.GPIO_Pin=ads1115_I2C_SDA;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(GPIOX_ads1115_I2C,&GPIO_InitStructure);
=======
/* 增益 */
adsGain_t m_gain;
/* 测试通道 */
// u8 m_channel = 0;

/**
 * @brief  设置配置寄存器
 * @param  None
 * @retval None:
 */
void ads1115_Init(unsigned int COM_Address)
{

	ads1115_I2C_INIT(COM_Address); // ads1115_I2C init

	/* 增益 */
	m_gain = GAIN_ONE;

	/* config the config reg */
	/*
	if (ConfigeRegister(m_channel))
	{
	  // deal error
	  printf("init configreg error\r\n");
	}else
	  printf("ads1115_Init:init configreg success\r\n");
	*/
}

/**
 * @brief  设置配置寄存器
 * @param  channel : 通道X(0..3)
 * @retval None:
 */
u8 ConfigeRegister(unsigned int COM_Address, u16 Ads_Address, unsigned char channel)
{

	/* Set config reg valus */
	uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE |	// 禁用比较器              (default val)
					  ADS1015_REG_CONFIG_CLAT_NONLAT |	// 非锁存比较器            (default val)
					  ADS1015_REG_CONFIG_CPOL_ACTVLOW | // ALERT/RDY 低电平有效    (default val)
					  ADS1015_REG_CONFIG_CMODE_TRAD |	// 具有迟滞的传统比较器     (default val)
					  ADS1015_REG_CONFIG_DR_1600SPS |	// 1600 samples per second (default val)
					  ADS1015_REG_CONFIG_MODE_CONTIN;	// Single-shot mode

	/* Set PGA/voltage range */
	config |= m_gain;

	/* Set 'start single-conversion' bit */
	config |= ADS1015_REG_CONFIG_OS_SINGLE;

	/* Set single-ended input channel */
	switch (channel)
	{
	case (0):
		config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
		break;
	case (1):
		config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
		break;
	case (2):
		config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
		break;
	case (3):
		config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
		break;
	}

	/* Write config register to the ADC */
	if (writeRegister(COM_Address, Ads_Address, ADS1015_REG_POINTER_CONFIG, config))
	{

		// deal no ack error
		return 1;
	}

	/* normal return */
	return 0;
}

/**
 * @brief  指针寄存器
 * @param  ads1115_I2CAddress : ADS115地址和读写位 0bit控制读写
 * @param  reg        :  要操作的寄存器
 * @retval None:
 */
u8 PointRegister(unsigned int COM_Address, uint8_t ads1115_I2CAddress, uint8_t reg)
{

	/* ads1115_I2C start */
	ads1115_I2C_Start(COM_Address);

	/* sent addr and write reg */
	ads1115_I2C_Send_Byte(COM_Address, ads1115_I2CAddress);
	/* ack */
	if (ads1115_I2C_Wait_Ack(COM_Address))
	{
		return 1;
	}

	/* Point reg is seted convert reg */
	ads1115_I2C_Send_Byte(COM_Address, reg);
	/* ack */
	if (ads1115_I2C_Wait_Ack(COM_Address))
	{
		return 1;
	}

	/* ads1115_I2C stop */
	ads1115_I2C_Stop(COM_Address);

	/* normal rerturn */
	return 0;
}

/**
 * @brief  设置增益
 * @param  gain : 增益
 * @retval None:
 */
void SetGain(adsGain_t gain)
{
	m_gain = gain;
}

/**
 * @brief  取得增益
 * @param  None
 * @retval None
 */
adsGain_t GetGain(void)
{
	return m_gain;
}

/**
 * @brief  写寄存器
 * @param  ads1115_I2CAddress : ADS115地址和读写位 0bit控制读写
 * @param  reg        :  要操作的寄存器
 * @param  value      :  写入寄存器的值
 * @retval None
 */
u8 writeRegister(unsigned int COM_Address, uint8_t ads1115_I2CAddress, uint8_t reg, uint16_t value)
{

	ads1115_I2C_Start(COM_Address);
	/* 地址和读写地址 */
	ads1115_I2C_Send_Byte(COM_Address, ads1115_I2CAddress);
	/* ACK */
	if (ads1115_I2C_Wait_Ack(COM_Address))
	{
		return 1;
	}

	/* 寄存器地址 */
	ads1115_I2C_Send_Byte(COM_Address, reg);
	/* ACK */
	if (ads1115_I2C_Wait_Ack(COM_Address))
	{
		return 1;
	}

	// write first byte
	ads1115_I2C_Send_Byte(COM_Address, (uint8_t)(value >> 8));
	/* ACK */
	if (ads1115_I2C_Wait_Ack(COM_Address))
	{
		return 1;
	}

	// write second byte
	ads1115_I2C_Send_Byte(COM_Address, (uint8_t)(value & 0xFF));
	/* ACK */
	if (ads1115_I2C_Wait_Ack(COM_Address))
	{
		return 1;
	}
	ads1115_I2C_Stop(COM_Address);

	/* normal return */
	return 0;
}

/**
 * @brief  读取convert寄存器
 * @param  None
 * @retval None
 */
uint16_t readConvertRegister(unsigned int COM_Address, u16 Ads_Address)
{

	u8 high = 0;
	u8 low = 0;

	/* ads1115_I2C start */
	ads1115_I2C_Start(COM_Address);
	/* addr and read reg */
	ads1115_I2C_Send_Byte(COM_Address, (uint8_t)(Ads_Address + 1));
	/* ACK */
	if (ads1115_I2C_Wait_Ack(COM_Address))
	{
		return 1;
	}

	high = ads1115_I2C_Read_Byte(COM_Address, 1);
	low = ads1115_I2C_Read_Byte(COM_Address, 1);

	/* ads1115_I2C stop */
	ads1115_I2C_Stop(COM_Address);

	return (high << 8 | low);
}

/**
 * @brief  获得adc的值
 * @param  None
 * @retval None
 */
uint16_t GetAds1115Values(unsigned int COM_Address, u16 Ads_Address)
{
	/* point to convert reg .ready to read adc values */
	if (PointRegister(COM_Address, Ads_Address, ADS1015_REG_POINTER_CONVERT))
	{
		// deal error
		printf("init PointRegister error\r\n");
	}

	return readConvertRegister(COM_Address, Ads_Address);
}

double ads1115_get_voltage_val(unsigned int COM_Address, u16 Ads_Address, u8 channel)
{
	double val;
	int16_t ad_val;

	if (ConfigeRegister(COM_Address, Ads_Address, channel))
	{
		// deal error
		printf("init channel %d configreg error\r\n", channel);
	}

	// ad_val = 12999;
	ad_val = GetAds1115Values(COM_Address, Ads_Address);

	if ((ad_val == 0x7FFF) | (ad_val == 0X8000)) // 是否超量程了
	{
		ad_val = 0;
		printf("over PGA\r\n");
	}

	switch (m_gain) // 量程对应的分辨率
	{
	case (GAIN_TWOTHIRDS):
		val = (double)ad_val * 187.5 / 1000000.0; //
		break;
	case (GAIN_ONE):
		val = (double)ad_val * 125 / 1000000.0;
		break;
	case (GAIN_TWO):
		val = (double)ad_val * 62.5 / 1000000.0;
		break;
	case (GAIN_FOUR):
		val = (double)ad_val * 31.25 / 1000000.0;
		break;
	case (GAIN_EIGHT):
		val = (double)ad_val * 15.625 / 1000000.0;
		break;
	case (GAIN_SIXTEEN):
		val = (double)ad_val * 7.8125 / 1000000.0;
		break;
	}

	return val;
}

void Get_Vol_ads1115(unsigned int COM_Address, uint16_t Ads_Address, uint8_t CHANNEL_NUM)
{
	int i = 0;
	int COM_index;
	float val;

	delay_ms(1000);
	val = ads1115_get_voltage_val(COM_Address, Ads_Address, CHANNEL_NUM); // 获得并打印电压值

	// when receive = 0,val = 0.000125xxxxx
	if (val > 0.000126)
	{
		if (Ads_Address == CMD_ADS1115_ADDR1)
		{
			printf("com %d ads1115(0x90) get vol   chaneel %d = %f\r\n", COM_Address, CHANNEL_NUM, val);
		}
		else
		{
			printf("com %d ads1115(0x92) get vol   chaneel %d = %f\r\n", COM_Address, CHANNEL_NUM, val);
		}
	}
}

/* ------------- ads1115_I2C drive ---------- */

/**
 * @brief  IIC初始化IO
 * @param  None
 * @retval None
 */
void ads1115_I2C_INIT(unsigned int COM_Address)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

	switch (COM_Address)
	{
	case 1:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM1_SCL | ADS1115_I2C_COM1_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM1_SCL_GPIOX, &GPIO_InitStructure);
		ADS1115_I2C_COM1_SCL_H;
		ADS1115_I2C_COM1_SDA_H;

	case 2:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM2_SCL | ADS1115_I2C_COM2_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		ADS1115_I2C_COM2_SCL_H;
		ADS1115_I2C_COM2_SDA_H;

	case 3:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM3_SCL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM3_SCL_GPIOX, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM3_SDA;
		GPIO_Init(ADS1115_I2C_COM3_SDA_GPIOX, &GPIO_InitStructure);
		ADS1115_I2C_COM3_SCL_H;
		ADS1115_I2C_COM3_SDA_H;

	case 4:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM4_SCL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM4_SCL_GPIOX, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM4_SDA;
		GPIO_Init(ADS1115_I2C_COM4_SDA_GPIOX, &GPIO_InitStructure);
		ADS1115_I2C_COM4_SCL_H;
		ADS1115_I2C_COM4_SDA_H;

	case 5:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM5_SCL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM5_SCL_GPIOX, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM5_SDA;
		GPIO_Init(ADS1115_I2C_COM5_SDA_GPIOX, &GPIO_InitStructure);
		ADS1115_I2C_COM5_SCL_H;
		ADS1115_I2C_COM5_SDA_H;

	case 6:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM6_SCL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM6_SCL_GPIOX, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM6_SDA;
		GPIO_Init(ADS1115_I2C_COM6_SDA_GPIOX, &GPIO_InitStructure);
		ADS1115_I2C_COM6_SCL_H;
		ADS1115_I2C_COM6_SDA_H;

	case 7:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM7_SCL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM7_SCL_GPIOX, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM7_SDA;
		GPIO_Init(ADS1115_I2C_COM7_SDA_GPIOX, &GPIO_InitStructure);
		ADS1115_I2C_COM7_SCL_H;
		ADS1115_I2C_COM7_SDA_H;

	case 8:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM8_SCL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM8_SCL_GPIOX, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM8_SDA;
		GPIO_Init(ADS1115_I2C_COM8_SDA_GPIOX, &GPIO_InitStructure);
		ADS1115_I2C_COM8_SCL_H;
		ADS1115_I2C_COM8_SDA_H;
	}
}

/**
 * @brief  SDA输出配置
 * @param  None
 * @retval None
 */
void ads1115_I2C_SDA_OUT(unsigned int COM_Address)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	switch (COM_Address)
	{
	case 1:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM1_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM1_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 2:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM2_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM2_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 3:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM3_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM3_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 4:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM4_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM4_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 5:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM5_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM5_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 6:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM6_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM6_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 7:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM7_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM7_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 8:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM8_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(ADS1115_I2C_COM8_SDA_GPIOX, &GPIO_InitStructure);
		break;
	}
}

/**
 * @brief  SDA输入配置
 * @param  None
 * @retval None
 */
void ads1115_I2C_SDA_IN(unsigned int COM_Address)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	switch (COM_Address)
	{
	case 1:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM1_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(ADS1115_I2C_COM1_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 2:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM2_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(ADS1115_I2C_COM2_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 3:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM3_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(ADS1115_I2C_COM3_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 4:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM4_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(ADS1115_I2C_COM4_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 5:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM5_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(ADS1115_I2C_COM5_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 6:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM6_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(ADS1115_I2C_COM6_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 7:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM7_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(ADS1115_I2C_COM7_SDA_GPIOX, &GPIO_InitStructure);
		break;

	case 8:
		GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_COM8_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(ADS1115_I2C_COM8_SDA_GPIOX, &GPIO_InitStructure);
		break;
	}
>>>>>>> 6f2502c (update_v1)
}

/**
  * @brief  产生起始信号信号
  * @param  None
  * @retval None

SCL   -------|_____


SDA    ---|________

*/
<<<<<<< HEAD
void ads1115_I2C_Start(void)
{
  ads1115_I2C_SDA_OUT();
  
  ads1115_I2C_SDA_H;
  ads1115_I2C_SCL_H;
  ads1115_delay_us(2);
  ads1115_I2C_SDA_L;
  ads1115_delay_us(2);
  ads1115_I2C_SCL_L;
=======
void ads1115_I2C_Start(unsigned int COM_Address)
{
	ads1115_I2C_SDA_OUT(COM_Address);

	switch (COM_Address)
	{

	case 1:
		ADS1115_I2C_COM1_SDA_H;
		ADS1115_I2C_COM1_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM1_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM1_SCL_L;
		break;

	case 2:
		ADS1115_I2C_COM2_SDA_H;
		ADS1115_I2C_COM2_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM2_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM2_SCL_L;
		break;

	case 3:
		ADS1115_I2C_COM3_SDA_H;
		ADS1115_I2C_COM3_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM3_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM3_SCL_L;
		break;

	case 4:
		ADS1115_I2C_COM4_SDA_H;
		ADS1115_I2C_COM4_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM4_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM4_SCL_L;
		break;

	case 5:
		ADS1115_I2C_COM5_SDA_H;
		ADS1115_I2C_COM5_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM5_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM5_SCL_L;
		break;

	case 6:
		ADS1115_I2C_COM6_SDA_H;
		ADS1115_I2C_COM6_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM6_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM6_SCL_L;
		break;

	case 7:
		ADS1115_I2C_COM7_SDA_H;
		ADS1115_I2C_COM7_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM7_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM7_SCL_L;
		break;

	case 8:
		ADS1115_I2C_COM8_SDA_H;
		ADS1115_I2C_COM8_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM8_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM8_SCL_L;
		break;

	default:
		printf("NO\n");
	}
>>>>>>> 6f2502c (update_v1)
}

/**
  * @brief  产生停止信号
  * @param  None
  * @retval None

SCL   ------------

SDA   ____|-------
<<<<<<< HEAD
  
*/
void ads1115_I2C_Stop(void)
{
   ads1115_I2C_SDA_OUT();

   ads1115_I2C_SCL_L;
   ads1115_I2C_SDA_L;
   ads1115_I2C_SCL_H;
   ads1115_delay_us(2);
   ads1115_I2C_SDA_H;
   ads1115_delay_us(2);
}

/**
  * @brief  主机产生应答信号ACK
  * @param  None
  * @retval None
  */
void ads1115_I2C_Ack(void)
{
   ads1115_I2C_SCL_L;
   ads1115_I2C_SDA_OUT();
   ads1115_I2C_SDA_L;
   ads1115_delay_us(2);
   ads1115_I2C_SCL_H;
   ads1115_delay_us(2);
   ads1115_I2C_SCL_L;
}

/**
  * @brief  主机不产生应答信号ACK
  * @param  None
  * @retval None
  */
void ads1115_I2C_NAck(void)
{
   ads1115_I2C_SCL_L;
   ads1115_I2C_SDA_OUT();
   ads1115_I2C_SDA_H;
   ads1115_delay_us(2);
   ads1115_I2C_SCL_H;
   ads1115_delay_us(2);
   ads1115_I2C_SCL_L;
=======

*/
void ads1115_I2C_Stop(unsigned int COM_Address)
{
	ads1115_I2C_SDA_OUT(COM_Address);

	switch (COM_Address)
	{

	case 1:
		ADS1115_I2C_COM1_SCL_L;
		ADS1115_I2C_COM1_SDA_L;
		ADS1115_I2C_COM1_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM1_SDA_H;
		ads1115_delay_us(2);
		break;

	case 2:
		ADS1115_I2C_COM2_SCL_L;
		ADS1115_I2C_COM2_SDA_L;
		ADS1115_I2C_COM2_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM2_SDA_H;
		ads1115_delay_us(2);
		break;

	case 3:
		ADS1115_I2C_COM3_SCL_L;
		ADS1115_I2C_COM3_SDA_L;
		ADS1115_I2C_COM3_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM3_SDA_H;
		ads1115_delay_us(2);
		break;

	case 4:
		ADS1115_I2C_COM4_SCL_L;
		ADS1115_I2C_COM4_SDA_L;
		ADS1115_I2C_COM4_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM4_SDA_H;
		ads1115_delay_us(2);
		break;

	case 5:
		ADS1115_I2C_COM5_SCL_L;
		ADS1115_I2C_COM5_SDA_L;
		ADS1115_I2C_COM5_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM5_SDA_H;
		ads1115_delay_us(2);
		break;

	case 6:
		ADS1115_I2C_COM6_SCL_L;
		ADS1115_I2C_COM6_SDA_L;
		ADS1115_I2C_COM6_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM6_SDA_H;
		ads1115_delay_us(2);
		break;

	case 7:
		ADS1115_I2C_COM7_SCL_L;
		ADS1115_I2C_COM7_SDA_L;
		ADS1115_I2C_COM7_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM7_SDA_H;
		ads1115_delay_us(2);
		break;

	case 8:
		ADS1115_I2C_COM8_SCL_L;
		ADS1115_I2C_COM8_SDA_L;
		ADS1115_I2C_COM8_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM8_SDA_H;
		ads1115_delay_us(2);
		break;
	}
}

/**
 * @brief  主机产生应答信号ACK
 * @param  None
 * @retval None
 */
void ads1115_I2C_Ack(unsigned int COM_Address)
{
	switch (COM_Address)
	{

	case 1:
		ADS1115_I2C_COM1_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM1_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM1_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM1_SCL_L;
		break;

	case 2:
		ADS1115_I2C_COM2_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM2_SDA_L;
		ads1115_delay_us(2);
		ADS1115_I2C_COM2_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM2_SCL_L;
		break;
	}
}

/**
 * @brief  主机不产生应答信号ACK
 * @param  None
 * @retval None
 */
void ads1115_I2C_NAck(unsigned int COM_Address)
{

	switch (COM_Address)
	{

	case 1:
		ADS1115_I2C_COM1_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM1_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM1_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM1_SCL_L;
		break;

	case 2:
		ADS1115_I2C_COM2_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM2_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM2_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM2_SCL_L;
		break;

	case 3:
		ADS1115_I2C_COM3_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM3_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM3_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM3_SCL_L;
		break;

	case 4:
		ADS1115_I2C_COM4_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM4_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM4_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM4_SCL_L;
		break;

	case 5:
		ADS1115_I2C_COM5_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM5_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM5_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM5_SCL_L;
		break;

	case 6:
		ADS1115_I2C_COM6_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM6_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM6_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM6_SCL_L;
		break;

	case 7:
		ADS1115_I2C_COM7_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM7_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM7_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM7_SCL_L;
		break;

	case 8:
		ADS1115_I2C_COM8_SCL_L;
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM8_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM8_SCL_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM8_SCL_L;
		break;
	}
>>>>>>> 6f2502c (update_v1)
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
<<<<<<< HEAD
u8 ads1115_I2C_Wait_Ack(void)
{
  u8 tempTime=0;

  ads1115_I2C_SDA_IN();

  ads1115_I2C_SDA_H;
  ads1115_delay_us(2);
  ads1115_I2C_SCL_H;
  ads1115_delay_us(2);

  while(GPIO_ReadInputDataBit(GPIOX_ads1115_I2C,ads1115_I2C_SDA))
  {
    tempTime++;
    if(tempTime>10)
    {
      // ads1115_I2C_Stop();
      return 1;
    }  
  }

  ads1115_I2C_SCL_L;
  return 0;
}
//ads1115_I2C 发送一个字节
=======
u8 ads1115_I2C_Wait_Ack(unsigned int COM_Address)
{
	u8 tempTime = 0;

	switch (COM_Address)
	{

	case 1:
		ads1115_I2C_SDA_IN(COM_Address);

		ADS1115_I2C_COM1_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM1_SCL_H;
		ads1115_delay_us(2);

		while (GPIO_ReadInputDataBit(ADS1115_I2C_COM1_SDA_GPIOX, ADS1115_I2C_COM1_SDA))
		{
			tempTime++;
			if (tempTime > 10)
			{
				return 1;
			}
		}
		ADS1115_I2C_COM1_SCL_L;
		break;

	case 2:
		ads1115_I2C_SDA_IN(COM_Address);

		ADS1115_I2C_COM2_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM2_SCL_H;
		ads1115_delay_us(2);

		while (GPIO_ReadInputDataBit(ADS1115_I2C_COM2_SDA_GPIOX, ADS1115_I2C_COM2_SDA))
		{
			tempTime++;
			if (tempTime > 10)
			{
				return 1;
			}
		}
		ADS1115_I2C_COM2_SCL_L;
		break;

	case 3:
		ads1115_I2C_SDA_IN(COM_Address);

		ADS1115_I2C_COM3_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM3_SCL_H;
		ads1115_delay_us(2);

		while (GPIO_ReadInputDataBit(ADS1115_I2C_COM3_SDA_GPIOX, ADS1115_I2C_COM3_SDA))
		{
			tempTime++;
			if (tempTime > 10)
			{
				return 1;
			}
		}
		ADS1115_I2C_COM3_SCL_L;
		break;

	case 4:
		ads1115_I2C_SDA_IN(COM_Address);

		ADS1115_I2C_COM4_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM4_SCL_H;
		ads1115_delay_us(2);

		while (GPIO_ReadInputDataBit(ADS1115_I2C_COM4_SDA_GPIOX, ADS1115_I2C_COM4_SDA))
		{
			tempTime++;
			if (tempTime > 10)
			{
				return 1;
			}
		}
		ADS1115_I2C_COM4_SCL_L;
		break;

	case 5:
		ads1115_I2C_SDA_IN(COM_Address);

		ADS1115_I2C_COM5_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM5_SCL_H;
		ads1115_delay_us(2);

		while (GPIO_ReadInputDataBit(ADS1115_I2C_COM5_SDA_GPIOX, ADS1115_I2C_COM5_SDA))
		{
			tempTime++;
			if (tempTime > 10)
			{
				return 1;
			}
		}
		ADS1115_I2C_COM5_SCL_L;
		break;

	case 6:
		ads1115_I2C_SDA_IN(COM_Address);

		ADS1115_I2C_COM6_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM6_SCL_H;
		ads1115_delay_us(2);

		while (GPIO_ReadInputDataBit(ADS1115_I2C_COM6_SDA_GPIOX, ADS1115_I2C_COM6_SDA))
		{
			tempTime++;
			if (tempTime > 10)
			{
				return 1;
			}
		}
		ADS1115_I2C_COM6_SCL_L;
		break;

	case 7:
		ads1115_I2C_SDA_IN(COM_Address);

		ADS1115_I2C_COM7_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM7_SCL_H;
		ads1115_delay_us(2);

		while (GPIO_ReadInputDataBit(ADS1115_I2C_COM7_SDA_GPIOX, ADS1115_I2C_COM7_SDA))
		{
			tempTime++;
			if (tempTime > 10)
			{
				return 1;
			}
		}
		ADS1115_I2C_COM7_SCL_L;
		break;

	case 8:
		ads1115_I2C_SDA_IN(COM_Address);

		ADS1115_I2C_COM8_SDA_H;
		ads1115_delay_us(2);
		ADS1115_I2C_COM8_SCL_H;
		ads1115_delay_us(2);

		while (GPIO_ReadInputDataBit(ADS1115_I2C_COM8_SDA_GPIOX, ADS1115_I2C_COM8_SDA))
		{
			tempTime++;
			if (tempTime > 10)
			{
				return 1;
			}
		}
		ADS1115_I2C_COM8_SCL_L;
		break;
	}

	return 0;
}

// ads1115_I2C 发送一个字节
>>>>>>> 6f2502c (update_v1)

/**
  * @brief  ads1115_I2C 发送一个字节
  * @param  txd 要发送的字符
  * @retval None:
  *
 SCL __|----|___|-----|___

 SDA __|--------|_________

数据:  |-- 1----| ----0---|

*/
<<<<<<< HEAD
void ads1115_I2C_Send_Byte(u8 txd)
{
  u8 i=0;

  ads1115_I2C_SDA_OUT();
  ads1115_I2C_SCL_L;//拉低时钟开始数据传输

  for(i=0;i<8;i++)
  {
    if((txd&0x80)>0) //0x80  1000 0000
      ads1115_I2C_SDA_H;
    else
      ads1115_I2C_SDA_L;

    txd<<=1;
    ads1115_I2C_SCL_H;
    ads1115_delay_us(2); //发送数据
    ads1115_I2C_SCL_L;
    ads1115_delay_us(2);
  }
}


/**
  * @brief  ads1115_I2C 读取一个字节
  * @param  ask: 1 读取有应答,0 没有应答
  * @retval None:
  */
u8 ads1115_I2C_Read_Byte(u8 ack)
{
   u8 i=0,receive=0;

   ads1115_I2C_SDA_IN();
   for(i=0;i<8;i++)
   {
    ads1115_I2C_SCL_L;
    ads1115_delay_us(2);
    ads1115_I2C_SCL_H;
    receive<<=1;
    if(GPIO_ReadInputDataBit(GPIOX_ads1115_I2C,ads1115_I2C_SDA))
       receive++;
    ads1115_delay_us(1);  
   }
   
   if(ack==0)
      ads1115_I2C_NAck();
   else
      ads1115_I2C_Ack();
   
  return receive;
}



/* -------------- delay --------------*/



/**
  * @brief  延时函数，延时us
  * @param  i(0..255)
  * @retval None:
  */
void ads1115_delay_us(u32 i)
{
  u32 temp;
  SysTick->LOAD=9*i;     //设置重装数值, 72MHZ时
  SysTick->CTRL=0X01;    //使能，减到零是无动作，采用外部时钟源
  SysTick->VAL=0;        //清零计数器
  do
  {
    temp=SysTick->CTRL;      //读取当前倒计数值
  }
  while((temp&0x01)&&(!(temp&(1<<16))));   //等待时间到达
  SysTick->CTRL=0;  //关闭计数器
  SysTick->VAL=0;   //清空计数器
}

/**
  * @brief  延时函数，延时ms
  * @param  i(0..255)
  * @retval None:
  */
void ads1115_delay_ms(u32 i)
{
  u32 temp;
  SysTick->LOAD=9000*i;   //设置重装数值, 72MHZ时
  SysTick->CTRL=0X01;   //使能，减到零是无动作，采用外部时钟源
  SysTick->VAL=0;     //清零计数器
  do
  {
    temp=SysTick->CTRL;    //读取当前倒计数值
  }
  while((temp&0x01)&&(!(temp&(1<<16))));  //等待时间到达
  SysTick->CTRL=0;  //关闭计数器
  SysTick->VAL=0;   //清空计数器
}




=======
void ads1115_I2C_Send_Byte(unsigned int COM_Address, u8 txd)
{
	u8 i = 0;

	switch (COM_Address)
	{

	case 1:
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM1_SCL_L; // 拉低时钟开始数据传输

		for (i = 0; i < 8; i++)
		{
			if ((txd & 0x80) > 0) // 0x80  1000 0000
				ADS1115_I2C_COM1_SDA_H;
			else
				ADS1115_I2C_COM1_SDA_L;

			txd <<= 1;
			ADS1115_I2C_COM1_SCL_H;
			ads1115_delay_us(2); // 发送数据
			ADS1115_I2C_COM1_SCL_L;
			ads1115_delay_us(2);
		}
		break;

	case 2:
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM2_SCL_L; // 拉低时钟开始数据传输

		for (i = 0; i < 8; i++)
		{
			if ((txd & 0x80) > 0) // 0x80  1000 0000
				ADS1115_I2C_COM2_SDA_H;
			else
				ADS1115_I2C_COM2_SDA_L;

			txd <<= 1;
			ADS1115_I2C_COM2_SCL_H;
			ads1115_delay_us(2); // 发送数据
			ADS1115_I2C_COM2_SCL_L;
			ads1115_delay_us(2);
		}
		break;

	case 3:
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM3_SCL_L; // 拉低时钟开始数据传输

		for (i = 0; i < 8; i++)
		{
			if ((txd & 0x80) > 0) // 0x80  1000 0000
				ADS1115_I2C_COM3_SDA_H;
			else
				ADS1115_I2C_COM3_SDA_L;

			txd <<= 1;
			ADS1115_I2C_COM3_SCL_H;
			ads1115_delay_us(2); // 发送数据
			ADS1115_I2C_COM3_SCL_L;
			ads1115_delay_us(2);
		}
		break;

	case 4:
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM4_SCL_L; // 拉低时钟开始数据传输

		for (i = 0; i < 8; i++)
		{
			if ((txd & 0x80) > 0) // 0x80  1000 0000
				ADS1115_I2C_COM4_SDA_H;
			else
				ADS1115_I2C_COM4_SDA_L;

			txd <<= 1;
			ADS1115_I2C_COM4_SCL_H;
			ads1115_delay_us(2); // 发送数据
			ADS1115_I2C_COM4_SCL_L;
			ads1115_delay_us(2);
		}
		break;

	case 5:
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM5_SCL_L; // 拉低时钟开始数据传输

		for (i = 0; i < 8; i++)
		{
			if ((txd & 0x80) > 0) // 0x80  1000 0000
				ADS1115_I2C_COM5_SDA_H;
			else
				ADS1115_I2C_COM5_SDA_L;

			txd <<= 1;
			ADS1115_I2C_COM5_SCL_H;
			ads1115_delay_us(2); // 发送数据
			ADS1115_I2C_COM5_SCL_L;
			ads1115_delay_us(2);
		}
		break;

	case 6:
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM6_SCL_L; // 拉低时钟开始数据传输

		for (i = 0; i < 8; i++)
		{
			if ((txd & 0x80) > 0) // 0x80  1000 0000
				ADS1115_I2C_COM6_SDA_H;
			else
				ADS1115_I2C_COM6_SDA_L;

			txd <<= 1;
			ADS1115_I2C_COM6_SCL_H;
			ads1115_delay_us(2); // 发送数据
			ADS1115_I2C_COM6_SCL_L;
			ads1115_delay_us(2);
		}
		break;

	case 7:
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM7_SCL_L; // 拉低时钟开始数据传输

		for (i = 0; i < 8; i++)
		{
			if ((txd & 0x80) > 0) // 0x80  1000 0000
				ADS1115_I2C_COM7_SDA_H;
			else
				ADS1115_I2C_COM7_SDA_L;

			txd <<= 1;
			ADS1115_I2C_COM7_SCL_H;
			ads1115_delay_us(2); // 发送数据
			ADS1115_I2C_COM7_SCL_L;
			ads1115_delay_us(2);
		}
		break;

	case 8:
		ads1115_I2C_SDA_OUT(COM_Address);
		ADS1115_I2C_COM8_SCL_L; // 拉低时钟开始数据传输

		for (i = 0; i < 8; i++)
		{
			if ((txd & 0x80) > 0) // 0x80  1000 0000
				ADS1115_I2C_COM8_SDA_H;
			else
				ADS1115_I2C_COM8_SDA_L;

			txd <<= 1;
			ADS1115_I2C_COM8_SCL_H;
			ads1115_delay_us(2); // 发送数据
			ADS1115_I2C_COM8_SCL_L;
			ads1115_delay_us(2);
		}
		break;
	}
}

/**
 * @brief  ads1115_I2C 读取一个字节
 * @param  ask: 1 读取有应答,0 没有应答
 * @retval None:
 */
u8 ads1115_I2C_Read_Byte(unsigned int COM_Address, u8 ack)
{
	u8 i = 0, receive = 0;
	ads1115_I2C_SDA_IN(COM_Address);

	switch (COM_Address)
	{

	case 1:
		for (i = 0; i < 8; i++)
		{
			ADS1115_I2C_COM1_SCL_L;
			ads1115_delay_us(2);
			ADS1115_I2C_COM1_SCL_H;
			receive <<= 1;
			if (GPIO_ReadInputDataBit(ADS1115_I2C_COM1_SDA_GPIOX, ADS1115_I2C_COM1_SDA))
				receive++;
			ads1115_delay_us(1);
		}
		break;

	case 2:
		for (i = 0; i < 8; i++)
		{
			ADS1115_I2C_COM2_SCL_L;
			ads1115_delay_us(2);
			ADS1115_I2C_COM2_SCL_H;
			receive <<= 1;
			if (GPIO_ReadInputDataBit(ADS1115_I2C_COM2_SDA_GPIOX, ADS1115_I2C_COM2_SDA))
				receive++;
			ads1115_delay_us(1);
		}
		break;

	case 3:
		for (i = 0; i < 8; i++)
		{
			ADS1115_I2C_COM3_SCL_L;
			ads1115_delay_us(2);
			ADS1115_I2C_COM3_SCL_H;
			receive <<= 1;
			if (GPIO_ReadInputDataBit(ADS1115_I2C_COM3_SDA_GPIOX, ADS1115_I2C_COM3_SDA))
				receive++;
			ads1115_delay_us(1);
		}
		break;

	case 4:
		for (i = 0; i < 8; i++)
		{
			ADS1115_I2C_COM4_SCL_L;
			ads1115_delay_us(2);
			ADS1115_I2C_COM4_SCL_H;
			receive <<= 1;
			if (GPIO_ReadInputDataBit(ADS1115_I2C_COM4_SDA_GPIOX, ADS1115_I2C_COM4_SDA))
				receive++;
			ads1115_delay_us(1);
		}
		break;

	case 5:
		for (i = 0; i < 8; i++)
		{
			ADS1115_I2C_COM5_SCL_L;
			ads1115_delay_us(2);
			ADS1115_I2C_COM5_SCL_H;
			receive <<= 1;
			if (GPIO_ReadInputDataBit(ADS1115_I2C_COM5_SDA_GPIOX, ADS1115_I2C_COM5_SDA))
				receive++;
			ads1115_delay_us(1);
		}
		break;

	case 6:
		for (i = 0; i < 8; i++)
		{
			ADS1115_I2C_COM6_SCL_L;
			ads1115_delay_us(2);
			ADS1115_I2C_COM6_SCL_H;
			receive <<= 1;
			if (GPIO_ReadInputDataBit(ADS1115_I2C_COM6_SDA_GPIOX, ADS1115_I2C_COM6_SDA))
				receive++;
			ads1115_delay_us(1);
		}
		break;

	case 7:
		for (i = 0; i < 8; i++)
		{
			ADS1115_I2C_COM7_SCL_L;
			ads1115_delay_us(2);
			ADS1115_I2C_COM7_SCL_H;
			receive <<= 1;
			if (GPIO_ReadInputDataBit(ADS1115_I2C_COM7_SDA_GPIOX, ADS1115_I2C_COM7_SDA))
				receive++;
			ads1115_delay_us(1);
		}
		break;

	case 8:
		for (i = 0; i < 8; i++)
		{
			ADS1115_I2C_COM8_SCL_L;
			ads1115_delay_us(2);
			ADS1115_I2C_COM8_SCL_H;
			receive <<= 1;
			if (GPIO_ReadInputDataBit(ADS1115_I2C_COM8_SDA_GPIOX, ADS1115_I2C_COM8_SDA))
				receive++;
			ads1115_delay_us(1);
		}
		break;
	}
	if (ack == 0)
		ads1115_I2C_NAck(COM_Address);
	else
		ads1115_I2C_Ack(COM_Address);

	return receive;
}

/* -------------- delay --------------*/

/**
 * @brief  延时函数，延时us
 * @param  i(0..255)
 * @retval None:
 */
void ads1115_delay_us(u32 i)
{
	u32 temp;
	SysTick->LOAD = 9 * i; // 设置重装数值, 72MHZ时
	SysTick->CTRL = 0X01;  // 使能，减到零是无动作，采用外部时钟源
	SysTick->VAL = 0;	   // 清零计数器
	do
	{
		temp = SysTick->CTRL;						  // 读取当前倒计数值
	} while ((temp & 0x01) && (!(temp & (1 << 16)))); // 等待时间到达
	SysTick->CTRL = 0;								  // 关闭计数器
	SysTick->VAL = 0;								  // 清空计数器
}

/**
 * @brief  延时函数，延时ms
 * @param  i(0..255)
 * @retval None:
 */
void ads1115_delay_ms(u32 i)
{
	u32 temp;
	SysTick->LOAD = 9000 * i; // 设置重装数值, 72MHZ时
	SysTick->CTRL = 0X01;	  // 使能，减到零是无动作，采用外部时钟源
	SysTick->VAL = 0;		  // 清零计数器
	do
	{
		temp = SysTick->CTRL;						  // 读取当前倒计数值
	} while ((temp & 0x01) && (!(temp & (1 << 16)))); // 等待时间到达
	SysTick->CTRL = 0;								  // 关闭计数器
	SysTick->VAL = 0;								  // 清空计数器
}
>>>>>>> 6f2502c (update_v1)
