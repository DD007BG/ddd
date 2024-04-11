#include "mlx90614.h"

struct MLX90614_Temp_s MOTO_Temp;



/*******************************************************************************
* 函数名: MLX90614MLX90614 发起始位 SMBus_StartBit
* 功能  : MLX90614 发起始位 产生起始位
* Input          : None
* Output         : None
* Return         : None

*******************************************************************************/
void SMBus_StartBit(void)
{
    SMBUS_SDA_H();		// Set SDA line
    SMBus_Delay(5);	    // Wait a few microseconds
    SMBUS_SCK_H();		// Set SCL line
    SMBus_Delay(5);	    // Generate bus free time between Stop
    SMBUS_SDA_L();		// Clear SDA line
    SMBus_Delay(5);	    // Hold time after (Repeated) Start
    // Condition. After this period, the first clock is generated.
    //(Thd:sta=4.0us min)在SCK=1时，检测到SDA由1到0表示通信开始（下降沿）
    SMBUS_SCK_L();	    // Clear SCL line
    SMBus_Delay(5);	    // Wait a few microseconds
}

/*******************************************************************************
* 函数名: SMBus_StopBit
* 功能: MLX90614 发停止位 STOP condition on SMBus
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SMBus_StopBit(void)
{
    SMBUS_SCK_L();		// Clear SCL line
    SMBus_Delay(5);	    // Wait a few microseconds
    SMBUS_SDA_L();		// Clear SDA line
    SMBus_Delay(5);	    // Wait a few microseconds
    SMBUS_SCK_H();		// Set SCL line
    SMBus_Delay(5);	    // Stop condition setup time(Tsu:sto=4.0us min)
    SMBUS_SDA_H();		// Set SDA line
}

/*******************************************************************************
* 函数名: SMBus_SendByte
* 功能: MLX90614 发送一个字节 Send a byte on SMBus
* Input          : Tx_buffer
* Output         : None
* Return         : None
*******************************************************************************/
u8 SMBus_SendByte(u8 Tx_buffer)
{
    u8	Bit_counter;
    u8	Ack_bit;
    u8	bit_out;

    for(Bit_counter=8; Bit_counter; Bit_counter--)
    {
        if (Tx_buffer&0x80)//如果最高位为1
        {
            bit_out=1;   // 把最高位置1
        }
        else  //如果最高位为0
        {
            bit_out=0;  // 把最高位置0
        }
        SMBus_SendBit(bit_out);	// 把最高位发送出去
        Tx_buffer<<=1;// 左移一位把最高位移出去等待下一个最高位，循环8次，每次都发最高位，就可把一个字节发出去了
    }
    Ack_bit=SMBus_ReceiveBit();
    return	Ack_bit;
}

/*******************************************************************************
* 函数名: SMBus_SendBit
* 功能: MLX90614 发送一个位 Send a bit on SMBus 82.5kHz
* Input          : bit_out
* Output         : None
* Return         : None
*******************************************************************************/
void SMBus_SendBit(u8 bit_out)
{
    if(bit_out==0)
    {
        SMBUS_SDA_L();
    }
    else
    {
        SMBUS_SDA_H();
    }
    SMBus_Delay(2);					// Tsu:dat = 250ns minimum
    SMBUS_SCK_H();					// Set SCL line
    SMBus_Delay(6);					// High Level of Clock Pulse
    SMBUS_SCK_L();					// Clear SCL line
    SMBus_Delay(3);					// Low Level of Clock Pulse
//	SMBUS_SDA_H();				    // Master release SDA line ,
    return;
}

/*******************************************************************************
* Function Name  : SMBus_ReceiveBit
* Description    : Receive a bit on SMBus
* Input          : None
* Output         : None
* Return         : Ack_bit
*******************************************************************************/
u8 SMBus_ReceiveBit(void)
{
    u8 Ack_bit;

    SMBUS_SDA_H();          //引脚靠外部电阻上拉，当作输入
	SMBus_Delay(2);			// High Level of Clock Pulse
    SMBUS_SCK_H();			// Set SCL line
    SMBus_Delay(5);			// High Level of Clock Pulse
    if (SMBUS_SDA_PIN())
    {
        Ack_bit=1;
    }
    else
    {
        Ack_bit=0;
    }
    SMBUS_SCK_L();			// Clear SCL line
    SMBus_Delay(3);			// Low Level of Clock Pulse

    return	Ack_bit;
}

/*******************************************************************************
* 函数名: SMBus_ReceiveByte
* 功能: Receive a byte on SMBus 从SMBus中接受一个字节的数据
* Input          : ack_nack
* Output         : None
* Return         : RX_buffer
*******************************************************************************/
u8 SMBus_ReceiveByte(u8 ack_nack)
{
    u8 	RX_buffer;
    u8	Bit_Counter;

    for(Bit_Counter=8; Bit_Counter; Bit_Counter--)
    {
        if(SMBus_ReceiveBit())// Get a bit from the SDA line
        {
            RX_buffer <<= 1;// If the bit is HIGH save 1  in RX_buffer
            RX_buffer |=0x01;//如果Ack_bit=1，把收到应答信号1与0000 0001 进行或运算，确保为1
        }
        else
        {
            RX_buffer <<= 1;// If the bit is LOW save 0 in RX_buffer
            RX_buffer &=0xfe;//如果Ack_bit=1，把收到应答信号0与1111 1110 进行与运算，确保为0
        }
    }
    SMBus_SendBit(ack_nack);// Sends acknowledgment bit 把应答信号发出去，如果1，就进行下一次通信，如果为0、，就拜拜了
    return RX_buffer;
}

/*******************************************************************************
* 函数名: SMBus_Delay
* 功能: 延时  一次循环约1us
* Input          : time
* Output         : None
* Return         : None
*******************************************************************************/
void SMBus_Delay(u16 time)
{
    u16 i, j;
    for (i=0; i<4; i++)
    {
        for (j=0; j<time; j++);
    }
}

/*******************************************************************************
* 函数名: SMBus_Init
* 功能: SMBus初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SMBus_Init()
{
    GPIO_InitTypeDef    GPIO_InitStructure;
	/* Enable SMBUS_PORT clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SMBUS_PORT, ENABLE);
    /*配置SMBUS_SCK、SMBUS_SDA为集电极开漏输出*/
    GPIO_InitStructure.GPIO_Pin = SMBUS_SCK | SMBUS_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SMBUS_PORT, &GPIO_InitStructure);

    SMBUS_SCK_H();
    SMBUS_SDA_H();
}

/*******************************************************************************
 * 函数名: SMBus_ReadMemory
 * 功能: READ DATA FROM RAM/EEPROM  从RAM和EEPROM中读取数据
 * Input          : slaveAddress, command
 * Return         : Data
 * SMBus_ReadMemory(0x00, 0x07) 0x00 表示IIC设备的从地址 从0x07这个寄存器开始读取
*******************************************************************************/
u16 SMBus_ReadMemory(u8 slaveAddress, u8 command)
{
    u16 data;			// Data storage (DataH:DataL)
    u8 Pec;				// PEC byte storage
    u8 DataL=0;			// Low data byte storage
    u8 DataH=0;			// High data byte storage
    u8 arr[6];			// Buffer for the sent bytes
    u8 PecReg;			// Calculated PEC byte storage
    u8 ErrorCounter;	// Defines the number of the attempts for communication with MLX90614

    ErrorCounter=0x00;	// Initialising of ErrorCounter
	slaveAddress <<= 1;	//2-7位表示从机地址 从机地址左移一位，把读写位空出来
	
    do
    {
repeat:
        SMBus_StopBit();			    //If slave send NACK stop comunication
        --ErrorCounter;				    //Pre-decrement ErrorCounter
        if(!ErrorCounter) 			    //ErrorCounter=0?
        {
            break;					    //Yes,go out from do-while{}
        }

        SMBus_StartBit();				//Start condition
        if(SMBus_SendByte(slaveAddress))//Send SlaveAddress 最低位Wr=0表示接下来写命令
        {
            goto	repeat;			    //Repeat comunication again
        }
        if(SMBus_SendByte(command))	    //Send command
        {
            goto	repeat;		    	//Repeat comunication again
        }

        SMBus_StartBit();					//Repeated Start condition
        if(SMBus_SendByte(slaveAddress+1))	//Send SlaveAddress 最低位Rd=1表示接下来读数据
        {
            goto	repeat;             	//Repeat comunication again
        }

        DataL = SMBus_ReceiveByte(ACK);	//Read low data,master must send ACK
        DataH = SMBus_ReceiveByte(ACK); //Read high data,master must send ACK
        Pec = SMBus_ReceiveByte(NACK);	//Read PEC byte, master must send NACK
        SMBus_StopBit();				//Stop condition

        arr[5] = slaveAddress;		//
        arr[4] = command;			//
        arr[3] = slaveAddress+1;	//Load array arr
        arr[2] = DataL;				//
        arr[1] = DataH;				//
        arr[0] = 0;					//
        PecReg=PEC_Calculation(arr);//Calculate CRC 数据校验
    }
    while(PecReg != Pec);//If received and calculated CRC are equal go out from do-while{}

	data = (DataH<<8) | DataL;	//data=DataH:DataL
    return data;
}

/*******************************************************************************
* 函数名: PEC_calculation
* 功能 : 数据校验
* Input          : pec[]
* Output         : None
* Return         : pec[0]-this byte contains calculated crc value
*******************************************************************************/
u8 PEC_Calculation(u8 pec[])
{
    u8 	crc[6];//存放多项式
    u8	BitPosition=47;//存放所有数据最高位，6*8=48 最高位就是47位
    u8	shift;
    u8	i;
    u8	j;
    u8	temp;

    do
    {
        /*Load pattern value 0x00 00 00 00 01 07*/
        crc[5]=0;
        crc[4]=0;
        crc[3]=0;
        crc[2]=0;
        crc[1]=0x01;
        crc[0]=0x07;

        /*Set maximum bit position at 47 ( six bytes byte5...byte0,MSbit=47)*/
        BitPosition=47;

        /*Set shift position at 0*/
        shift=0;

        /*Find first "1" in the transmited message beginning from the MSByte byte5*/
        i=5;
        j=0;
        while((pec[i]&(0x80>>j))==0 && i>0)
        {
            BitPosition--;
            if(j<7)
            {
                j++;
            }
            else
            {
                j=0x00;
                i--;
            }
        }/*End of while */

        /*Get shift value for pattern value*/
        shift=BitPosition-8;

        /*Shift pattern value */
        while(shift)
        {
            for(i=5; i<0xFF; i--)
            {
                if((crc[i-1]&0x80) && (i>0))
                {
                    temp=1;
                }
                else
                {
                    temp=0;
                }
                crc[i]<<=1;
                crc[i]+=temp;
            }/*End of for*/
            shift--;
        }/*End of while*/

        /*Exclusive OR between pec and crc*/
        for(i=0; i<=5; i++)
        {
            pec[i] ^=crc[i]; 
        }/*End of for*/
    }
    while(BitPosition>8); /*End of do-while*/

    return pec[0];
}

 /*
 *********************************************************************************************************
 *   函 数 名:清零eeprom数据
 *   功能说明: 
 *   形    参:  
 *   返 回 值: 
 *********************************************************************************************************
 */  
 void SMBus_CleanEEPROM(void)
 {
     
        u8 arr[6];          // Buffer for the sent bytes
        u8 PecReg;          // Calculated PEC byte storage

        SMBus_StartBit();                //Start condition
        SMBus_SendByte(0x00);//Send SlaveAddress 最低位Wr=0表示接下来写命令
        SMBus_SendByte(0x2E);
        SMBus_SendByte(0x00);
        SMBus_SendByte(0x00);

        arr[5] = 0;     //
        arr[4] = 0x00;          //
        arr[3] = 0x2e;  //Load array arr
        arr[2] = 0x00;              //
        arr[1] = 0x00;              //
        arr[0] = 0;                 // 
        PecReg=PEC_Calculation(arr);//Calculate CRC
        SMBus_SendByte(PecReg);
        SMBus_StopBit();                //Stop condition
 }
 /*
 *********************************************************************************************************
 *   函 数 名: 从修改从机地址
 *   功能说明: 
 *   形    参:  
 *   返 回 值: 
 *********************************************************************************************************
 */
 void SMBus_EditSlaveAddre(u16 SlaveAddress)
 {
         
 
        u8 arr[6];          // Buffer for the sent bytes
        u8 PecReg;          // Calculated PEC byte storage
        u8 dataH=(MLX_90614_SAx_BASE+SlaveAddress)>>8;
        u8 dataL=(MLX_90614_SAx_BASE+SlaveAddress)&0xff;
     
        SMBus_CleanEEPROM();
        delay_ms(5);
     
        SMBus_StartBit();                //Start condition
        SMBus_SendByte(0x00);//Send SlaveAddress 最低位Wr=0表示接下来写命令
        SMBus_SendByte(0x2E);
        SMBus_SendByte(dataL);
        SMBus_SendByte(dataH);
     
        arr[5] = 0;     //
        arr[4] = 0x00;          //
        arr[3] = 0x2e;  //Load array arr
        arr[2] = dataL;             //
        arr[1] = dataH;             //
        arr[0] = 0;                 // 
        PecReg=PEC_Calculation(arr);//Calculate CRC
        SMBus_SendByte(PecReg);
        SMBus_StopBit();                //Stop condition
        printf("修改完成，可以重启电源开始读取温度值！\r\n");

 }

 /*******************************************************************************
 * 函数名: SMBus_ReadTemp
 * 功能: 计算并返回温度值
 * Return         : SMBus_ReadMemory(0x00, 0x07)*0.02-273.15
*******************************************************************************/
float SMBus_ReadTemp(u8 slaveAddress)
{   
	float temp;
	temp = SMBus_ReadMemory(slaveAddress, RAM_ACCESS|RAM_TOBJ1)*0.02-273.15;
	return temp;
}

/*********************************END OF FILE*********************************/


void Get_Temp(uint8_t moto_num)
{
    switch(moto_num)
    {
        case 16: MOTO_Temp.MOTO16_temp  = SMBus_ReadTemp(MLX90614_SA16);
        case 15: MOTO_Temp.MOTO15_temp  = SMBus_ReadTemp(MLX90614_SA15);
        case 14: MOTO_Temp.MOTO14_temp  = SMBus_ReadTemp(MLX90614_SA14);
        case 13: MOTO_Temp.MOTO13_temp  = SMBus_ReadTemp(MLX90614_SA13);
        case 12: MOTO_Temp.MOTO12_temp  = SMBus_ReadTemp(MLX90614_SA12);
        case 11: MOTO_Temp.MOTO11_temp  = SMBus_ReadTemp(MLX90614_SA11);
        case 10: MOTO_Temp.MOTO10_temp  = SMBus_ReadTemp(MLX90614_SA10);
        case 9:  MOTO_Temp.MOTO9_temp   = SMBus_ReadTemp(MLX90614_SA9);
        case 8:  MOTO_Temp.MOTO8_temp   = SMBus_ReadTemp(MLX90614_SA8);
        case 7:  MOTO_Temp.MOTO7_temp   = SMBus_ReadTemp(MLX90614_SA7);
        case 6:  MOTO_Temp.MOTO6_temp   = SMBus_ReadTemp(MLX90614_SA6);
        case 5:  MOTO_Temp.MOTO5_temp   = SMBus_ReadTemp(MLX90614_SA5);
        case 4:  MOTO_Temp.MOTO4_temp   = SMBus_ReadTemp(MLX90614_SA4);
        case 3:  MOTO_Temp.MOTO3_temp   = SMBus_ReadTemp(MLX90614_SA3);
        case 2:  MOTO_Temp.MOTO2_temp   = SMBus_ReadTemp(MLX90614_SA2);
        case 1:  MOTO_Temp.MOTO1_temp   = SMBus_ReadTemp(MLX90614_SA1);break;
        default: printf("err: Get_Temp  input val wrong\r\n");
    }
    
    print_Temp(moto_num);
}

void print_Temp(uint8_t moto_num)
{
    switch(moto_num)
    {
        case 16: printf("MLX_SA16 Temp = %2.1f\r\n",MOTO_Temp.MOTO16_temp);
        case 15: printf("MLX_SA15 Temp = %2.1f\r\n",MOTO_Temp.MOTO15_temp);
        case 14: printf("MLX_SA14 Temp = %2.1f\r\n",MOTO_Temp.MOTO14_temp);
        case 13: printf("MLX_SA13 Temp = %2.1f\r\n",MOTO_Temp.MOTO13_temp);
        case 12: printf("MLX_SA12 Temp = %2.1f\r\n",MOTO_Temp.MOTO12_temp);
        case 11: printf("MLX_SA11 Temp = %2.1f\r\n",MOTO_Temp.MOTO11_temp);
        case 10: printf("MLX_SA10 Temp = %2.1f\r\n",MOTO_Temp.MOTO10_temp);
        case 9:  printf("MLX_SA9  Temp = %2.1f\r\n",MOTO_Temp.MOTO9_temp);
        case 8:  printf("MLX_SA8  Temp = %2.1f\r\n",MOTO_Temp.MOTO8_temp);
        case 7:  printf("MLX_SA7  Temp = %2.1f\r\n",MOTO_Temp.MOTO7_temp);
        case 6:  printf("MLX_SA6  Temp = %2.1f\r\n",MOTO_Temp.MOTO6_temp);
        case 5:  printf("MLX_SA5  Temp = %2.1f\r\n",MOTO_Temp.MOTO5_temp);
        case 4:  printf("MLX_SA4  Temp = %2.1f\r\n",MOTO_Temp.MOTO4_temp);
        case 3:  printf("MLX_SA3  Temp = %2.1f\r\n",MOTO_Temp.MOTO3_temp);
        case 2:  printf("MLX_SA2  Temp = %2.1f\r\n",MOTO_Temp.MOTO2_temp);
        case 1:  printf("MLX_SA1  Temp = %2.1f\r\n",MOTO_Temp.MOTO1_temp);break;
        default: printf("err: print_Temp  input val wrong\r\n");
    }
}


