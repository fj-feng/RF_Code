/**
  ******************************************************************************
  * 文件名 ：   SX126X_Driver.c
  * 作者   ：   LSD RF Team
  * 版本   ：   V1.0.0
  * 时间   ：   15-Aug-2018
  * 文件描述：
  *     该文件为SX126X模块的驱动层，包含对SX126X模块的工作模式控制，FIFO操作，发
  *射功率、频率等设置，以及一些数据处理的公共函数
  *    客户在使用SX126X模块时候需要移植该文件，如果SX126X_HAL.c中各个函数功能都已
  *实现且函数名等都没有改动，那本文件可以不用做更改直接使用，客户在应用层直接调
  *用本文件函数就可以实现各种操作。
*******************************************************************************/
#include "SX126X_Driver.h"
#include "stm32l4xx_hal.h"
#include "timer.h"
#define VERSION        LSD4RFC_2L722N10     //选择所选用产品型号，例如：
                                                                  //     LSD4RFC_2L722N10																							         
																																  //     LSD4RFC_2L714N10
																																	 //    L_LRMRL22_97NN4
 
 
//static RadioPacketTypes_t PacketType;

float G_BandWidthKHz = 500.0;//本地计算Symbol周期使用
float G_TsXms = 1.024;//1.024ms
//static RadioPacketTypes_t PacketType;

S_LoRaConfig G_LoRaConfig = {
    470000000,
    LORA_BW_125,
    LORA_SF7,
    LORA_CR_4_5,
    22,
    LORA_PACKET_EXPLICIT,
    LORA_CRC_ON,
    LORA_IQ_NORMAL,
    8,
    64,
};

S_LoRaPara G_LoRaPara;

static RadioOperatingModes_t OperatingMode;
/*
  注意，本文件的函数实现了从RF的驱动与MCU的兼容

*/
/**
  * @简介：SX126X 唤醒
  * @参数：无
  * @返回值：无
  */
void SX126xWakeup( void )
{
    SX126X_NSS_OUTPUT(GPIO_PIN_RESET);
    SX126X_ReadWriteByte( RADIO_GET_STATUS );
    SX126X_ReadWriteByte( 0x00 );
    SX126X_NSS_OUTPUT(GPIO_PIN_SET);
    // Wait for chip to be ready.
    SX126xWaitOnBusy( );
}
/**
  * @简介：获取当前工作模式
  * @参数：无
  * @返回值：OperatingMode
  */
RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}
/**
  * @简介：检查SX126X是否处于准备状态
  * @参数：无
  * @返回值：无
  */
void SX126xCheckDeviceReady( void )
{
    if( ( SX126xGetOperatingMode( ) == MODE_SLEEP ) || ( SX126xGetOperatingMode( ) == MODE_RX_DC ) )
    {
        SX126xWakeup( );
    }
    SX126xWaitOnBusy( );
}
/**
  * @简介：写命令
  * @参数：command，buffer，size
  * @返回值：无
  */
void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );
    SX126X_NSS_OUTPUT(GPIO_PIN_RESET);
    SX126X_ReadWriteByte(( uint8_t )command);
    for( uint16_t i = 0; i < size; i++ )
    {
        SX126X_ReadWriteByte(buffer[i]);
    }
    SX126X_NSS_OUTPUT(GPIO_PIN_SET);
    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}
/**
  * @简介：读命令
  * @参数：command，buffer，size
  * @返回值：无
  */
void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{

    SX126xCheckDeviceReady( );
    SX126X_NSS_OUTPUT(GPIO_PIN_RESET);
    SX126X_ReadWriteByte(( uint8_t )command);
    SX126X_ReadWriteByte(0);
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SX126X_ReadWriteByte(0x00);
    }
    SX126X_NSS_OUTPUT(GPIO_PIN_SET);
    SX126xWaitOnBusy( );
}
/**
  * @简介：写寄存器组
  * @参数：address，buffer，size
  * @返回值：无
  */
void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );
    SX126X_NSS_OUTPUT(GPIO_PIN_RESET);
    SX126X_ReadWriteByte(RADIO_WRITE_REGISTER);
    SX126X_ReadWriteByte((address & 0xFF00 ) >> 8);
    SX126X_ReadWriteByte( address & 0x00FF );
    for( uint16_t i = 0; i < size; i++ )
    {
        SX126X_ReadWriteByte( buffer[i] );
    }
    SX126X_NSS_OUTPUT(GPIO_PIN_SET);
    SX126xWaitOnBusy( );
}
/**
  * @简介：写单个寄存器
  * @参数：address，value
  * @返回值：无
  */
void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}
/**
  * @简介：读寄存器组
  * @参数：address，buffer，size
  * @返回值：无
  */
void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    SX126X_NSS_OUTPUT(GPIO_PIN_RESET);
    SX126X_ReadWriteByte(RADIO_READ_REGISTER );
    SX126X_ReadWriteByte((address & 0xFF00 ) >> 8);
    SX126X_ReadWriteByte( address & 0x00FF );
    SX126X_ReadWriteByte(0);
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SX126X_ReadWriteByte(0x00);
    }
    SX126X_NSS_OUTPUT(GPIO_PIN_SET);
    SX126xWaitOnBusy( );
}
/**
  * @简介：读单个寄存器
  * @参数：address
  * @返回值：data
  */
uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}
/**
  * @简介：写Buffer
  * @参数：offset,buffer,size
  * @返回值：无
  */
void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );
    SX126X_NSS_OUTPUT(GPIO_PIN_RESET);
    SX126X_ReadWriteByte( RADIO_WRITE_BUFFER );
    SX126X_ReadWriteByte( offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        SX126X_ReadWriteByte( buffer[i] );
    }
    SX126X_NSS_OUTPUT(GPIO_PIN_SET);
    SX126xWaitOnBusy( );
}
/**
  * @简介：读Buffer
  * @参数：offset,buffer,size
  * @返回值：无
  */
void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );
    SX126X_NSS_OUTPUT(GPIO_PIN_RESET);
    SX126X_ReadWriteByte( RADIO_READ_BUFFER );
    SX126X_ReadWriteByte( offset );
    SX126X_ReadWriteByte(0);
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SX126X_ReadWriteByte(0x00);
    }
    SX126X_NSS_OUTPUT(GPIO_PIN_SET);
    SX126xWaitOnBusy( );
}
void SX126xWriteFifo( uint8_t *buffer, uint8_t size )
{
    SX126xWriteBuffer( 0, buffer, size );
}

void SX126xReadFifo( uint8_t *buffer, uint8_t size )
{
    SX126xReadBuffer( 0, buffer, size );
}


//-------------------------SX126X 基本设置-----------------------//
//该部分函数为对SX126X模块复位、发射功率、发射频率等参数设置，以
//及SX126X工作模式设置、数据包读写等
//--------------------------------------------------------------//

/**
  * @简介：SX126X  复位设置
  * @参数：无
  * @返回值：无
  */
void SX126X_Reset(void)
{
    HAL_Delay(10);
    SX126X_RESET_OUTPUT(GPIO_PIN_RESET);
    HAL_Delay(20);
    SX126X_RESET_OUTPUT(GPIO_PIN_SET);
    HAL_Delay(10);
}
/**
  * @简介：SX126X  IO初始化及配置
  * @参数：无
  * @返回值：无
  */
void SX126X_InitIo(void)
{
    SX126X_DIO1_INPUT();
    SX126X_DIO2_INPUT();
    SX126X_Busy_INPUT();
//    SX126X_SPIGPIO_Init();
    SX126X_RESET_OUTPUT(GPIO_PIN_SET);
    SX126X_SWCTL1_OUTPUT(GPIO_PIN_RESET);
    SX126X_SWCTL2_OUTPUT(GPIO_PIN_RESET);
//	  IO_SET(GPIO_PIN_RESET);
}
/**
  * @简介：SX126X  TX/RX的PA切换
  * @参数：bool txEnable  切换逻辑；真：作为TX。假：作为RX   为硬件两个PA控制IO口
  * @返回值:无
  */
void SX126XWriteRxTx( bool txEnable )
{
    if( txEnable != 0 )       //如果为真，为TX
    {
        SX126X_SWCTL1_OUTPUT(GPIO_PIN_RESET);
        SX126X_SWCTL2_OUTPUT(GPIO_PIN_SET);
    }
    else  //为假，为RX
    {
        SX126X_SWCTL1_OUTPUT(GPIO_PIN_SET);
        SX126X_SWCTL2_OUTPUT(GPIO_PIN_RESET);
    }
}
/**
  * @简介：SX126X 切断高频开关，防止高频开关工作，增加功耗
  * @参数：无
  * @返回值:无
  */
void SX126XSwitchOff(void)
{
    SX126X_SWCTL1_OUTPUT(GPIO_PIN_RESET);
    SX126X_SWCTL2_OUTPUT(GPIO_PIN_RESET);
}

/**
  * @简介：DIO1中断开启使能
  * @参数：无
  * @返回值：无
  */
void DIO1_EnableInterrupt(void)
{
    SX126X_DIO1_INTENABLE();
}
/**
  * @简介：DIO0中断关闭使能
  * @参数：无
  * @返回值：无
  */
void DIO1_DisableInterrupt(void)
{
    SX126X_DIO1_INTDISABLE();
}
/**
  * @简介：DIO1引脚状态获取
  * @参数：无
  * @返回值：State表示DIO1获取的电平，高电平"1",低电平"0"
  */
GPIO_PinState DIO1_GetState(void)
{
    GPIO_PinState State;
    State = SX126X_DIO1_GetState();
    return State;
}
/**
  * @简介：DIO2中断开启使能
  * @参数：无
  * @返回值：无
  */
void DIO2_EnableInterrupt(void)
{
    SX126X_DIO2_INTENABLE();
}
/**
  * @简介：DIO2中断关闭使能
  * @参数：无
  * @返回值：无
  */
void DIO2_DisableInterrupt(void)
{
    SX126X_DIO2_INTDISABLE();
}
/**
  * @简介：DIO2引脚状态获取
  * @参数：无
  * @返回值：State表示DIO2获取的电平，高电平"1",低电平"0"
  */
GPIO_PinState DIO2_GetState(void)
{
    GPIO_PinState State;
    State = SX126X_DIO2_GetState();
    return State;
}
/**
  * @简介：Busy中断开启使能
  * @参数：无
  * @返回值：无
  */
void Busy_EnableInterrupt(void)
{
    SX126X_Busy_INTENABLE();
}
/**
  * @简介：Busy中断关闭使能
  * @参数：无
  * @返回值：无
  */
void Busy_DisableInterrupt(void)
{
    SX126X_Busy_INTDISABLE();
}
/**
  * @简介：Busy引脚状态获取
  * @参数：无
  * @返回值：State表示Busy获取的电平，高电平"1",低电平"0"
  */
GPIO_PinState Busy_GetState(void)
{
    GPIO_PinState State;
    State = SX126X_Busy_GetState();
    return State;
}
/**
  * @简介：Standby模式
  * @参数：STDBY_RC or STDBY_XOSC
  * @返回值：无
*/
void SX126xSetStandby( RadioStandbyModes_t standbyConfig )
{
		RadioStatus_t status;
	    unsigned int nTimes = 65535;
    SX126xWriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
		status=SX126xGetStatus();
	
    if( standbyConfig == STDBY_RC )
    {
			 do
			{
					status=SX126xGetStatus();
					nTimes--;
			}
			while((status.Fields.ChipMode != 0X02) && nTimes);
      OperatingMode = MODE_STDBY_RC;
    }
    else
    {
			 do
			{
					status=SX126xGetStatus();
					nTimes--;
			}
			while((status.Fields.ChipMode != 0X03) && nTimes);
      OperatingMode = MODE_STDBY_XOSC;
    }
}

/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xSleep( void )
{	 
    SleepParams_t params = { 0 };
    params.Fields.WarmStart = 1;
    SX126xSetSleep( params );
    SX126XSwitchOff();//切断高频开关，防止休眠模式下高频开关增加功耗
		
   // HAL_Delay(2);
}
/**
  * @简介：SX126X进入LORA睡眠模式
  * @参数：无
  * @返回值：无
  */
void SX126xSetSleep( SleepParams_t sleepConfig )
{
    SX126xWriteCommand( RADIO_SET_SLEEP, &sleepConfig.Value, 1 );
	  OperatingMode = MODE_SLEEP;
}

/**
  * @简介：Lora模式
  * @参数：packetType
  * @返回值：无
*/
void SX126xSetPacketType( RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    //PacketType = packetType;

    if( packetType == PACKET_TYPE_GFSK )
    {
        SX126xWriteRegister( REG_BIT_SYNC, 0x00 );
    }
    SX126xWriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
}
/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xSetRegulatorMode( RadioRegulatorMode_t mode )
{
    SX126xWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}
/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xSetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX126xWriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}

/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xCalibrate( CalibrationParams_t calibParam )
{
    SX126xWriteCommand( RADIO_CALIBRATE, ( uint8_t* )&calibParam, 1 );
}

/**
  * @简介：该函数主要用于防止当低温等情况出现PLL Lock等异常时，可能出现RF无法工作，及时清除错误标志及进行校准
  * @参数：
  * @返回值：无
*/
void SX126xCalibrate_err()
{
		RadioError_t error;
		error=SX126xGetDeviceErrors();
		if(error.Value!=0)
		{
			SX126xClearDeviceErrors();
			CalibrationParams_t calibParam;
			calibParam.Value = 0x7F;
			SX126xCalibrate(calibParam);
		}
}	

/**
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xCalibrateImage( uint32_t freq )
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xD8;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    SX126xWriteCommand( RADIO_CALIBRATEIMAGE, calFreq, 2 );
}
/**
  * @简介:频率设置
  * @参数：
  * @返回值：无
*/
tSX126xError SX126xSetRfFrequency(uint32_t frequency )
{
    if((frequency > 960000000) || (frequency < 410000000))
        return PARAMETER_INVALID;
    uint8_t buf[4];
    uint32_t freq = 0;
    SX126xCalibrateImage(frequency);
    freq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
    buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freq & 0xFF );
    SX126xWriteCommand( RADIO_SET_RFFREQUENCY, buf, 4);
		 return NORMAL;
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];
    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    SX126xWriteCommand( RADIO_CFG_DIOIRQ, buf, 8 );
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut )
{
    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    SX126xWriteCommand( RADIO_SET_PACONFIG, buf, 4 );
}


#if (VERSION==LSD4RFC_2L714N10)
/**
  * @简介：发射功率设置，该函数配置是结合硬糒SD4RFC-2L714N10考虑功耗及功率最优设置； 
  * @参数：Power输入值范围：-3~22；
  * @返回值:
  */

void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
	uint8_t buf[2];
    if( power >= 14 )
    {
        power = 15;
    }
    else if( power < -3 )
    {
        power = -3;
    }	 
    SX126xSetPaConfig( 0x04, 0x06, 0x00, 0x01 );
		
    SX126xWriteRegister( REG_OCP, 0x38 ); // current max 140mA for the whole device
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SX126xWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

#else
/**
  * @简介：发射功率设置，该函数配置是结合硬件LSD4RFC-2L722N10考虑功耗及功率最优设置； 
  * @参数：Power输入值范围：-3~22；
  * @返回值:
  */
void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];
    if( power > 22 )
    {
        power = 22;
    }
    else if( power < -3 )
    {
        power = -3;
    }	 
    SX126xSetPaConfig( 0x04, 0x07, 0x00, 0x01 );
		
    SX126xWriteRegister( REG_OCP, 0x38 ); // current max 140mA for the whole device
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SX126xWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}
  #endif


/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_800_US );
}

/**
  * @简介:调制参数设置 ，SF、BW、CR、低数率优化设置
  * @参数：无
  * @返回值：无
*/
void SX126xSetLoraModulationParams(void)
{
    uint8_t n;

    uint8_t buf[8] = { 0x00, 0x00, 0x00, 0x00};
    n = 4;
    buf[0] = G_LoRaConfig.SpreadingFactor;
    buf[1] = G_LoRaConfig.BandWidth;
    buf[2] =  G_LoRaConfig.CodingRate;

		if(G_TsXms > 16.0)
    {
        buf[3] = LOWDATARATEOPTIMIZE_ON;
    }
    else
    {
        buf[3] = LOWDATARATEOPTIMIZE_OFF;
    }

		
	//当LoRa模式下时，BW为500KHz 时寄存器0x0889的第二位为0，否则其它BW或GFSK模式时第二位应该为1
 	if(  G_LoRaConfig.BandWidth == LORA_BW_500 )
     {
			 SX126xWriteRegister(TxModulation, SX126xReadRegister(TxModulation) & 0xfB);
     } 
		 else
		 {
			  SX126xWriteRegister(TxModulation, SX126xReadRegister(TxModulation) | 0x04);
		 }
		
    SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );
}
/**
  * @简介:前导码、报头格式、PL、CRC等设置
  * @参数：无
  * @返回值：无
*/
void SX126xSetLoraPacketParams(void)
{
    //SX126xSetPacketType( PACKET_TYPE_LORA );
    uint8_t n;
    uint8_t buf[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

		//SF6、SF7时要求前导长度大于等于12
		if( ( G_LoRaConfig.SpreadingFactor == LORA_SF5 ) || (G_LoRaConfig.SpreadingFactor == LORA_SF6 ) )
      {
				if( G_LoRaConfig.PreambleLength < 12 )
					{
						G_LoRaConfig.PreambleLength = 12;
					}
              
      }   

    n = 6;
    buf[0] = (G_LoRaConfig.PreambleLength >> 8 ) & 0xFF;
    buf[1] = G_LoRaConfig.PreambleLength;
    buf[2] = G_LoRaConfig.HeaderType;
    buf[3] = G_LoRaConfig.PayloadLength;
    buf[4] = G_LoRaConfig.CrcMode;
    buf[5] = G_LoRaConfig.InvertIQ;
    SX126xWriteCommand( RADIO_SET_PACKETPARAMS, buf, n );
		
		 // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
		if( G_LoRaConfig.InvertIQ == LORA_IQ_INVERTED )
		{
				// RegIqPolaritySetup = @address 0x0736
				SX126xWriteRegister( 0x0736, SX126xReadRegister( 0x0736 ) & ~( 1 << 2 ) );
		}
		else
		{
				// RegIqPolaritySetup @address 0x0736
				SX126xWriteRegister( 0x0736, SX126xReadRegister( 0x0736 ) | ( 1 << 2 ) );
		}
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetStopRxTimerOnPreambleDetect( bool enable )
{
    SX126xWriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, ( uint8_t* )&enable, 1 );
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xSetLoRaSymbNumTimeout( uint8_t SymbNum )
{
    SX126xWriteCommand( RADIO_SET_LORASYMBTIMEOUT, &SymbNum, 1 );
}
/**
  * @简介：SX126X 配置频率检查判断
  * @参数：无
  * @返回值：true or false
  */
bool LoRaConfig_Check(void)
{
if((G_LoRaConfig.LoRa_Freq < 410000000) || (G_LoRaConfig.LoRa_Freq > 960000000))
        return false;

G_LoRaConfig.BandWidth = (RadioLoRaBandwidths_t)(G_LoRaConfig.BandWidth & 0x0F);

    if((0x3>=G_LoRaConfig.BandWidth)&(0x7<=G_LoRaConfig.BandWidth))//LLCC68 BW 限制
        return false;
  //计算BandWidth
    switch(G_LoRaConfig.BandWidth) {
    case  LORA_BW_500:
        G_BandWidthKHz = 500.0;
        break;
    case  LORA_BW_250:
        G_BandWidthKHz = 250.0;
        break;
    case  LORA_BW_125:
        G_BandWidthKHz = 125.0;
        break;
    case  LORA_BW_062:
        G_BandWidthKHz = 62.5;
        break;
    case  LORA_BW_041:
        G_BandWidthKHz = 41.67;
        break;
    case  LORA_BW_031:
        G_BandWidthKHz = 31.25;
        break;
    case  LORA_BW_020:
        G_BandWidthKHz = 20.83;
        break;
    case  LORA_BW_015:
        G_BandWidthKHz = 15.63;
        break;
    case  LORA_BW_010:
        G_BandWidthKHz = 10.42;
        break;
    case  LORA_BW_007:
        G_BandWidthKHz = 7.81;
        break;
    }
		
		//LLCC68 速率限制
		 G_LoRaConfig.SpreadingFactor = (RadioLoRaSpreadingFactors_t)(G_LoRaConfig.SpreadingFactor & 0x0F);
			if(G_LoRaConfig.BandWidth==LORA_BW_125)
			{
				if((G_LoRaConfig.SpreadingFactor >= LORA_SF10) || (G_LoRaConfig.SpreadingFactor < LORA_SF5))
				return false;
			}
			if(G_LoRaConfig.BandWidth==LORA_BW_250)
			{
				if((G_LoRaConfig.SpreadingFactor >= LORA_SF11) || (G_LoRaConfig.SpreadingFactor < LORA_SF5))
				return false;
			}
			if(G_LoRaConfig.BandWidth==LORA_BW_500)
			{
				if((G_LoRaConfig.SpreadingFactor >= LORA_SF12) || (G_LoRaConfig.SpreadingFactor < LORA_SF5))
				return false;
			}
		
		//计算LoRa码元周期，单位ms
    G_TsXms = (2 << ((G_LoRaConfig.SpreadingFactor) - 1)) / G_BandWidthKHz;
		
		 G_LoRaConfig.CodingRate = (RadioLoRaCodingRates_t)(G_LoRaConfig.CodingRate & 0x07);
    if((G_LoRaConfig.CodingRate > LORA_CR_4_8) || (G_LoRaConfig.CodingRate < LORA_CR_4_5))
        return false;
    if(G_LoRaConfig.PowerCfig > 22)
        return false;
		 return true;
   
}
/**
* @简介:SX126X初始化
  * @参数：无
  * @返回值：tSX126xError   错误枚举内容
  */

tSX126xError SX126x_Lora_init(void)
{
    if(false == LoRaConfig_Check()) //如果输入参数错误
    {
      return PARAMETER_INVALID;  //报错输入
    }
		SX126X_SPI_Init();              //SPI初始化
    SX126X_InitIo();                // PAIO口初始化
		
		/*------------------------------------------------
        SX126X Lora模式初始化 --------  */
    SX126X_Reset();                 //复位RF
		SX126xWakeup();
		SX126xSetStandby(STDBY_RC);    //切换到LoRamode，standby状态
		SX126xWriteRegister(TxClampConfig, SX126xReadRegister(TxClampConfig) | 0x1E); //SX126X 芯片PA内部有过压保护，如果失配会导致功率降低问题；
		                                                                //修改寄存器TxClampConfig来优化PA的阈值，位4-1必须
		                                                                //设置为"1111“（默认为0100）---2019.07.31
		SX126xSetRegulatorMode( USE_DCDC );
		SX126xSetDio2AsRfSwitchCtrl(1);
		SX126xSetPacketType( PACKET_TYPE_LORA );  //设置成LORA模式
    /*------------------------------------------------
    SPI 验证                   */
    uint8_t test = 0;
    SX126xWriteRegister( REG_LR_PAYLOADLENGTH, 0x09);   //选一个用不到的寄存器来做验证
    test = SX126xReadRegister( REG_LR_PAYLOADLENGTH );
    if(test != 0x09)
        return SPI_READCHECK_WRONG;
		
		SX126xWriteRegister(REG_RX_GAIN, 0x96);//开启Rx Boosted gain,默认0x94不开启
    SX126xSetRfFrequency(G_LoRaConfig.LoRa_Freq);
    SX126xSetRfTxPower( G_LoRaConfig.PowerCfig );
		SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetLoraModulationParams();   //SF、BW、CR、LOWDATARATEOPTIMIZE
    SX126xSetLoraPacketParams();  //PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ
    //SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    return NORMAL;
}
/**
  * @简介:清标志
  * @参数：irq
  * @返回值：无
*/
void SX126xClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    SX126xWriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}
/**
  * @简介:获取标志
  * @参数：无
  * @返回值：对应标志
*/
uint16_t SX126xGetIrqStatus( void )
{
    uint8_t irqStatus[2];

    SX126xReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
    return ( irqStatus[0] << 8 ) | irqStatus[1];
}

RadioError_t SX126xGetDeviceErrors( void )
{
    uint8_t err[] = { 0, 0 };
    RadioError_t error = { .Value = 0 };

    SX126xReadCommand( RADIO_GET_ERROR, ( uint8_t* )err, 2 );
    error.Fields.PaRamp     = ( err[0] & ( 1 << 0 ) ) >> 0;
    error.Fields.PllLock    = ( err[1] & ( 1 << 6 ) ) >> 6;
    error.Fields.XoscStart  = ( err[1] & ( 1 << 5 ) ) >> 5;
    error.Fields.ImgCalib   = ( err[1] & ( 1 << 4 ) ) >> 4;
    error.Fields.AdcCalib   = ( err[1] & ( 1 << 3 ) ) >> 3;
    error.Fields.PllCalib   = ( err[1] & ( 1 << 2 ) ) >> 2;
    error.Fields.Rc13mCalib = ( err[1] & ( 1 << 1 ) ) >> 1;
    error.Fields.Rc64kCalib = ( err[1] & ( 1 << 0 ) ) >> 0;
    return error;
}

void SX126xClearDeviceErrors( void )
{
    uint8_t buf[2] = { 0x00, 0x00 };
    SX126xWriteCommand( RADIO_CLR_ERROR, buf, 2 );
}


/**
  * @简介:当在LoRa数据包接收模式时，开启RxTimeout，且在隐性报头（Implicit Mode）模式时，可能出现接收完成后，超时定时器不关闭现象，需要进行停止RTC和清除超时事件
  * @参数：
  * @返回值：无
*/
void SX126xClearTimeoutEvent( void )
{
		// WORKAROUND - Implicit Header Mode Timeout Behavior, see DS_SX1261-2_V1.2 datasheet chapter 15.3
		// RegRtcControl = @address 0x0902
		SX126xWriteRegister( 0x0902, 0x00 );
		// RegEventMask = @address 0x0944
		SX126xWriteRegister( 0x0944, SX126xReadRegister( 0x0944 ) | ( 1 << 1 ) );
		// WORKAROUND END
}
						
/**
  * @简介:DIO2开关控制
  * @参数：enable
  * @返回值：无
*/
void SX126xSetDio2AsRfSwitchCtrl( uint8_t enable )
{
    SX126xWriteCommand( RADIO_SET_RFSWITCHMODE, &enable, 1 );
}

/**
  * @简介:DIO3 TCXO电源控制及功能开启
  * @参数：enable
  * @返回值：无
*/
void SX126xSetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout )
{
    uint8_t buf[4];

    buf[0] = tcxoVoltage & 0x07;
    buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( timeout & 0xFF );

    SX126xWriteCommand( RADIO_SET_TCXOMODE, buf, 4 );
}

RadioStatus_t SX126xGetStatus( void )
{
    uint8_t stat = 0;
    RadioStatus_t status;

    SX126xReadCommand( RADIO_GET_STATUS, ( uint8_t * )&stat, 1 );
    status.Value = stat;
    return status;
}
/**
  * @简介:获人彩薄RSSI值
  * @参数：无
  * @返回值：RSSI值
*/
int8_t SX126xGetRssiInst( void )
{
    uint8_t buf[1];
    int8_t rssi = 0;

    SX126xReadCommand( RADIO_GET_RSSIINST, buf, 1 );
    rssi = -buf[0] >> 1;
    return rssi;
}
/**
  * @简介:
  * @参数：
  * @返回值：无
*/
void SX126xGetRxBufferStatus( uint8_t *payloadLength )  //, uint8_t *rxStartBufferPointer
{
    uint8_t status[2];

    SX126xReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

    // In case of LORA fixed header, the payloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    if( ( SX126xReadRegister( REG_LR_PACKETPARAMS ) >> 7 == 1 ) )   
    {
        *payloadLength = SX126xReadRegister( REG_LR_PAYLOADLENGTH );  
    }                         //这里的判断其实和外层判断报头为显隐性是一样的，这两个寄存器规格书中未给出，但是验证是等效成立的。
    else
    {
        *payloadLength = status[0];
    }
    // *rxStartBufferPointer = status[1];
}
/**
  * @简介:获取数据包RSSI及SNR值
  * @参数：
  * @返回值：无
*/
void SX126xGetPacketStatus( S_LoRaPara *pktStatus )
{
    uint8_t status[3];

    SX126xReadCommand( RADIO_GET_PACKETSTATUS, status, 3 );

    pktStatus->AvgPacket_RSSI = -status[0] >> 1;   //上一个数据包的RSSI平均值
    ( status[1] < 128 ) ? ( pktStatus->Packet_SNR = status[1] >> 2 ) : ( pktStatus->Packet_SNR = ( ( status[1] - 256 ) >> 2 ) );
    pktStatus->LastPacket_RSSI = -status[2] >> 1;
}

void SX126xSetFs( void )
{
    SX126xWriteCommand( RADIO_SET_FS, 0, 0 );
}

/**
  * @简介:进入发射
  * @参数：
  * @返回值：无
*/
void SX126xSetTx( uint32_t timeout )
{
    uint8_t buf[3];
    SX126xClearIrqStatus( IRQ_RADIO_ALL );
    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );

    SX126xWriteCommand( RADIO_SET_TX, buf, 3 );
}
/**
  * @简介:进入接收
  * @参数：
  * @返回值：无
*/
void SX126xSetRx( uint32_t timeout )
{
    uint8_t buf[3];
    SX126xClearIrqStatus( IRQ_RADIO_ALL );
    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    SX126xWriteCommand( RADIO_SET_RX, buf, 3 );
}

/**
  * @简介:发送数据
  * @参数：PL、size、timeout
  * @返回值：无
*/
void SX126xSendPayload( uint8_t *payload, uint8_t size, uint32_t timeout )
{
    SX126xWriteBuffer( 0x00, payload, size );
    SX126xSetTx( timeout );
}
/**
  * @简介:发送数据包
  * @参数：data
  * @返回值：无
*/
extern uint8_t communication_states;
void SX126X_TxPacket( uint8_t*data )
{
	  SX126xSetStandby(STDBY_XOSC);  //这里如果不是使用TCXO版本，STDBY_XOSC和STDBY_RC都无影响，如果使用的是TCXO版本，需要使用STDBY_XOSC，否则超时时间会增加到切换到TX模式周期中；
		SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
		SX126xSetRegulatorMode( USE_DCDC );
    SX126XWriteRxTx(true);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    SX126xSetDioIrqParams( IRQ_TX_DONE|IRQ_RX_TX_TIMEOUT ,
                           IRQ_TX_DONE|IRQ_RX_TX_TIMEOUT ,
                          IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );//开启Tx、Tx超时中断，映射到DIO1；
                           //(p1,p2,p3,p4),P1:开哪个，P2:映射到DIO1，P3：映射到DIO2，P4映射到DIO3；
//    SX126xSetLoraModulationParams( );   //SF、BW、CR、LOWDATARATEOPTIMIZE
    SX126xSetLoraPacketParams( );  //PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSendPayload( data, G_LoRaConfig.PayloadLength, 0x2fffff );// 开启了发射超时，时间为：0x2fffff*15.625≈50s,
	             //这是一个安全措施，可能由于某种突发导执行了该命令但是不产生TxDOne,这时设置Tx超时就可以保证状态不被打乱，用户根据实际应用配置情况进行设置超时时间
} 
/**
  * @简介  :发送数据包
  * @参数  ：uint8_t*cbuf:发送数据指针
  * @返回值：无
  * @说明  ：接收完数据后DIO1从低电平变成高电平
*/
void LSD_RF_RXmode()
{
	 DIO1_EnableInterrupt();
   SX126X_StartRx();//进入接收模式，准备接收数据
}

/**
  * @简介  :发送数据包
  * @参数  ：uint8_t*cbuf:发送数据指针
  * @返回值：无
  * @说明  ：数据发送完成后DIO1从低电平变成高电平，每次调用次函数，会自动现将DIO1变为低电平，等待高电平
*/
void LSD_RF_SendPacket(uint8_t*cbuf)
{
   unsigned long int j=16777215;                   //超时用，用户需要根据实际情况来调整
	 DIO1_DisableInterrupt();
   SX126X_TxPacket(cbuf );
   while((DIO1_GetState()== GPIO_PIN_RESET)&&j)j--;  //等待DIO1电平为高
	 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);                                   
}

/**
  * @简介:进入接收
  * @参数：无
  * @返回值：无
*/
void SX126X_StartRx( void )
{
    SX126xSetStandby(STDBY_XOSC);  //这里如果不是使用TCXO版本，STDBY_XOSC和STDBY_RC都无影响，如果使用的是TCXO版本，需要使用STDBY_XOSC，否则超时时间会增加到切换到RX模式周期中；
		SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
		SX126xSetRegulatorMode( USE_DCDC );
    SX126XWriteRxTx(false);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    SX126xSetDioIrqParams( IRQ_RX_DONE|IRQ_CRC_ERROR|IRQ_RX_TX_TIMEOUT, 
                           IRQ_RX_DONE|IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );  //打开RX、CRC、RX超时中断，映射RX中断到DIO1；
	  SX126xSetStopRxTimerOnPreambleDetect( false );//用来选择定时器停止是检测前导码还是报头；
    SX126xSetLoRaSymbNumTimeout(0); //设置用来验证正确LoRa信号的字符个数，多数情况下设置为0即可；
	      //一般设置为0时，即表示接收机验证一个LoRa信号字符，但是由于各种突发情况存在极低的可能误判，所以加大验证数值可以提高一定的正确性；
//    SX126xSetLoraModulationParams();   //SF、BW、CR、LOWDATARATEOPTIMIZE设置
//    SX126xSetLoraPacketParams();  //PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ设置
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetRx( 0x2fffff ); // 开启了接收超时，时间为：0x2fffff*15.625us≈50s,用户根据实际应用配置情况进行设置超时时间；（默认情况是检测到报头（LoRa）或同步字（FSK）才会终止该定时器）
}
/**
  * @简介:获取数据
  * @参数：buffer，size
  * @返回值：无
*/
void SX126xGetPayload( uint8_t *buffer, uint8_t *size)
{
    SX126xReadBuffer( 0x00, buffer, *size );
}

/**
  * @简介：RF接收数据包
  * @参数：uint8_t*cbuf接收数组指针
  * @返回值：无
  */
void SX126X_RxPacket(uint8_t*cbuf)
{

    SX126xGetPacketStatus(&G_LoRaPara);//收到一包数据包后，返回SNR、RSSI等各种信息
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    if( G_LoRaConfig.HeaderType == LORA_PACKET_EXPLICIT ) {
        //有头那么从寄存器中读，否则按照G_LoRaConfig设置长度读取FIFO
        SX126xGetRxBufferStatus(&G_LoRaConfig.PayloadLength);
    }
    SX126xGetPayload(cbuf, &G_LoRaConfig.PayloadLength);

}

/**
  * @简介：WOR初始化
  * @参数：无
  * @返回值：无
  */

/*
cadSymbolNum:表示用来检测CAD信号的符号数，影响功耗及CAD误判率，数字越大，功耗越大，误判率越低；
cadDetPeak：
cadDetMin：这个两个参数设置和灵敏度有关，受影响于SF和BW；
cadExitMode:CAD_ONLY\CAD_RX 指执行完CAD后进入的状态；
cadTimeout:指CAD完成后如果进入RX，执行RX的时间；
对于cadSymbolNum、cadDetPeak、cadExitMode这几个参数推荐有下：
1、按照通信成功率最佳推荐：
  SF   	 cadSymbolNum 				cadDetPeak				cadExitMode						CAD Consumption(nAh)
	7			LORA_CAD_02_SYMBOL				22								10											2.84				
	8			LORA_CAD_02_SYMBOL				22								10											5.75
	9			LORA_CAD_04_SYMBOL				23								10											20.44
	10		LORA_CAD_04_SYMBOL				24								10											41.36
	11		LORA_CAD_04_SYMBOL				25								10											134.55

2、按照通信成功率兼容考虑消耗最最佳推荐：
  SF   	 cadSymbolNum 				cadDetPeak				cadExitMode						CAD Consumption(nAh)
	7			LORA_CAD_02_SYMBOL				22								10											2.84				
	8			LORA_CAD_02_SYMBOL				22								10											5.75
	9			LORA_CAD_02_SYMBOL				24								10											11.7
	10		LORA_CAD_02_SYMBOL				25								10											23.86
	11		LORA_CAD_02_SYMBOL				26								10											48.79
	
	以上推荐是根据实际测试结果给出来的，实际应用中如有更严格要求可调整参数进行实测判断结果；
*/
void SX126xSetCadParams( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode, uint32_t cadTimeout )
{
    uint8_t buf[7];

    buf[0] = ( uint8_t )cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = ( uint8_t )cadExitMode;
    buf[4] = ( uint8_t )( ( cadTimeout >> 16 ) & 0xFF );
    buf[5] = ( uint8_t )( ( cadTimeout >> 8 ) & 0xFF );
    buf[6] = ( uint8_t )( cadTimeout & 0xFF );
    SX126xWriteCommand( RADIO_SET_CADPARAMS, buf, 5 );
}


/**
  * @简介：SX126X 进CAD
  * @参数：无
  * @返回值：无
  */
void SX126xSetCad( void )
{
    SX126xWriteCommand( RADIO_SET_CAD, 0, 0 );
}
/**
  * @简介：SX126X CAD初始化
  * @参数：无
  * @返回值：无
  */
void SX126X_CADinit(void)
{
	
  SX126xSetStandby(STDBY_RC);
	SX126xSetPacketType( PACKET_TYPE_LORA );  //设置成LORA模式
	SX126xSetRegulatorMode( USE_DCDC );
	
	SX126xSetCadParams(  LORA_CAD_02_SYMBOL, 22, 10, LORA_CAD_ONLY, 0xff );//这里按照SF=7时推荐配置；参数配置与SF、BW配置及功耗考虑有关，可以参照本函书说明配置
	
	SX126XWriteRxTx(false);
	SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
	SX126xSetDioIrqParams( IRQ_CAD_DONE|IRQ_CAD_ACTIVITY_DETECTED ,
                           IRQ_CAD_DONE ,
                           IRQ_RADIO_NONE,
	                         IRQ_RADIO_NONE );
													 //(p1,p2,p3,p4),P1:开哪个，P2:映射到DIO1，P3：映射到DIO2，P4映射到DIO3；
}
/**
  * @简介：WOR初始化
  * @参数：无
  * @返回值：无
  */
void SX126X_WORInit(void)
{
    SX126X_CADinit();        //CAD功能初始化
}
/**
  * @简介：SX126X 启动CAD，采样信道情况一次   采样时间约为(2^SF+32)/BW
  * @参数：无
  * @返回值：无
  */
void SX126X_CAD_Sample(void)
{
		SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
	  SX126xSetStandby(STDBY_RC);
    SX126xSetCad();
}
/**
  * @简介：执行WOR操作
  * @参数：uint8_t cclen  0：进入睡眠;1：进入CAD检测模式
  * @返回值：无
  */
void SX126X_WOR_Execute(uint8_t cclen)
{
    switch(cclen)
    {
    case 0:   //启动睡眠
    {
        SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
			  SX126xSetStandby(STDBY_RC);
        SX126xSleep();  //进入睡眠模式
    }
    break;
    case 1:   //进入CAD检测模式
    {
        SX126X_CAD_Sample();     //启动CAD一次
    }
    break;
    default:
        break;
    }
}


/**
  * @简介：WOR到RX
  * @参数：无
  * @返回值：无
  */
void SX126X_WOR_Exit(void)
{
		SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
    SX126xSetStandby(STDBY_RC);
  	SX126xSetPacketType( PACKET_TYPE_LORA );  //设置成LORA模式
  	SX126xSetRegulatorMode( USE_DCDC );
	  SX126XWriteRxTx(false);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    SX126xSetDioIrqParams( IRQ_RX_DONE|IRQ_CRC_ERROR, 
                           IRQ_RX_DONE,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );  //打开RX、CRC中断，映射RX中断到DIO1；
	  SX126xSetStopRxTimerOnPreambleDetect( false );
    SX126xSetLoRaSymbNumTimeout(0);
	  G_LoRaConfig.PreambleLength=0xffff;
//	  SX126xSetLoraModulationParams( );   //SF、BW、CR、LOWDATARATEOPTIMIZE
    SX126xSetLoraPacketParams( );  //PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ设置
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetRx( 0xffffff ); // Rx Continuout
}

/**
  * @简介：SX126X发送唤醒包
  * @参数：uint8_t*data：发送数据指针,前导长度
  * @返回值：无
  */
void SX126X_Awake(uint8_t*cbuf, uint16_t Preamble_Length)
{
		SX126xCalibrate_err();	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
	  SX126xSetStandby(STDBY_RC);
	  SX126xSetRegulatorMode( USE_DCDC );
    SX126XWriteRxTx(true);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
	  SX126xSetDioIrqParams( IRQ_TX_DONE ,
                           IRQ_TX_DONE ,
                           IRQ_RADIO_NONE,
	                         IRQ_RADIO_NONE );
													 //(p1,p2,p3,p4),P1:开哪个，P2:映射到DIO1，P3：映射到DIO2，P4映射到DIO3；
	  G_LoRaConfig.PreambleLength=Preamble_Length;
	  SX126xSetLoraPacketParams( );//PreambleLength、HeaderType、PayloadLength、CrcMode、InvertIQ设置
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSendPayload( cbuf, G_LoRaConfig.PayloadLength, 0xffffff );
}
