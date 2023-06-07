#include "SX126X_Driver_FSK.h"
#include "stm32l4xx_hal.h"
#include "timer.h"


#define VERSION        LSD4RF_2R717N40     //选择所选用产品型号，例如： LSD4RF_2R717N40
                                                                 //     LSD4RF_2R717N30
																																 //     LSD4RF_2R722N20      
																																 //     LSD4RF_2R714N10 
																																 
																																 
//uint8_t MaxPayloadLength = 0xFF;


S_FSKConfig G_FSKConfig = {
    470000000,                       //Freq
	  22,                              //Power
    50000,                           //BitRate
    25000,                           //Fdev
    MOD_SHAPING_G_BT_1,              //ModulationShaping
    100000,                          //Bandwidth
    5,                               //PreambleLength
    RADIO_PREAMBLE_DETECTOR_08_BITS, //PreambleMinDetect
    3,                               //SyncWordLength
    RADIO_ADDRESSCOMP_FILT_OFF,      //AddrComp
    RADIO_PACKET_VARIABLE_LENGTH,                          //HeaderType
	  64,	
		RADIO_CRC_2_BYTES_CCIT,
		RADIO_DC_FREEWHITENING,
};

S_FSKPara G_FSKPara;
static RadioOperatingModes_t OperatingMode;

void SX126xSetStopRxTimerOnPreambleDetect( bool enable )
{
    SX126xWriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, ( uint8_t* )&enable, 1 );
}



/**
  * @简介：SX126X 配置频率检查判断
  * @参数：无
  * @返回值：true or false
  */
bool SX126X_FSKConfig_Check(void) 
{
  if(( G_FSKConfig.FSK_Freq < 160000000) || ( G_FSKConfig.FSK_Freq > 950000000))
        return false;
	
	if(( G_FSKConfig.BitRate < 600) || ( G_FSKConfig.BitRate> 300000))
        return false;
	
	if(( G_FSKConfig.BitRate < 600) || ( G_FSKConfig.BitRate> 300000))
        return false;
	if(( 	G_FSKConfig.Bandwidth < 4800) || ( G_FSKConfig.BitRate> 500000))
        return false;
	if(( G_FSKConfig.PreambleLength < 1) || ( G_FSKConfig.PreambleLength> 8191))
        return false;
	if(( G_FSKConfig.SyncWordLength < 0) || ( G_FSKConfig.PreambleLength> 8))
        return false;
	 return true;
	
}

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
tSX126x_FSK_Error SX126xSetRfFrequency(uint32_t frequency )
{
    if((G_FSKConfig.FSK_Freq > 920000000) || (G_FSKConfig.FSK_Freq < 410000000))
        return PARAMETER_INVALID;
    uint8_t buf[4];
    uint32_t freq = 0;
    SX126xCalibrateImage( G_FSKConfig.FSK_Freq);
    freq = ( uint32_t )( ( double )G_FSKConfig.FSK_Freq / ( double )FREQ_STEP );
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


/**
  * @简介：发射功率设置，该函数配置是结合硬件LSD4RF-2R717N30 考虑功耗及功率最优设置； 
  * @参数：Power输入值范围：-3~22；
  * @返回值:
  */

#if (VERSION==LSD4RF_2R717N30)
void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];
    if( power > 20 )
    {
        power = 20;
    }
    else if( power < -3 )
    {
        power = -3;
    }
		 switch(power)
    {
    case 20: 
     {
       power=22;
       SX126xSetPaConfig( 0x04, 0x07, 0x00, 0x01 );//实际输出20dBm左右
     }
        break;
    case 19:
     {
       power=22;
       SX126xSetPaConfig( 0x02, 0x07, 0x00, 0x01 );//实际输出19dBm左右
     }
        break;
		 case 18: 
     {
       power=22;
       SX126xSetPaConfig( 0x03, 0x05, 0x00, 0x01 );//实际输出18dBm左右
     }
    case 17: 
     {
       power=22;
       SX126xSetPaConfig( 0x02, 0x03, 0x00, 0x01 );//实际输出17dBm左右
     }
		  break;
    default:
		{
        power = power + 5;
        SX126xSetPaConfig( 0x02, 0x03, 0x00, 0x01 );//0~17dBm
    }
	 }
    SX126xWriteRegister( REG_OCP, 0x38 ); // current max 140mA for the whole device
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SX126xWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

#elif (VERSION==LSD4RF_2R714N10)
/**
  * @简介：发射功率设置，该函数配置是结合硬糒SD4RF-2R714N10考虑功耗及功率最优设置； 
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
  * @简介：发射功率设置，该函数配置是结合硬件LSD4RF-2R717N40\LSD4RF-2R722N20\考虑功耗及功率最优设置； 
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

void SX126X_FSKTxPacket( uint8_t*data )
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
//    SX126xSetFSKModulationParams( );   //
    SX126xSetFSKPacketParams( );  //
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSendPayload( data, G_FSKConfig.PayloadLength, 0x2fffff );// 开启了发射超时，时间为：0x2fffff*15.625≈50s,
	             //这是一个安全措施，可能由于某种突发导执行了该命令但是不产生TxDOne,这时设置Tx超时就可以保证状态不被打乱，用户根据实际应用配置情况进行设置超时时间
} 

/**
  * @简介:进入接收
  * @参数：无
  * @返回值：无
*/
void SX126X_FSKStartRx( void )
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
	  //SX126xSetFSKModulationParams( );   //
    SX126xSetFSKPacketParams( );  //
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetRx( 0x9C400 ); // 开启了接收超时，时间为：0x9C400*15.625us≈10s,用户根据实际应用配置情况进行设置超时时间；（默认情况是检测到报头（LoRa）或同步字（FSK）才会终止该定时器）
}


/**
  * @简介:获取数据包RSSI及SNR值
  * @参数：
  * @返回值：无
*/
void SX126xGetPacketStatus( S_FSKPara *pktStatus )
{
     uint8_t status[3];
     SX126xReadCommand( RADIO_GET_PACKETSTATUS, status, 3 );
		 pktStatus->RxStatus = status[0];
		 pktStatus->RssiSync = -status[1] >> 1;
		 pktStatus->RssiAvg = -status[2] >> 1;
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
void SX126X_FSKRxPacket(uint8_t*cbuf)
{

    SX126xGetPacketStatus(&G_FSKPara);//收到一包数据包后，返回SNR、RSSI等各种信息
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    if( G_FSKConfig.HeaderType == RADIO_PACKET_VARIABLE_LENGTH ) {
        //有头那么从寄存器中读，否则按照G_LoRaConfig设置长度读取FIFO
        SX126xGetRxBufferStatus(&G_FSKConfig.PayloadLength);
    }
    SX126xGetPayload(cbuf, &G_FSKConfig.PayloadLength);
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
  * @简介：
  * @参数：
  * @返回值：无
*/
void SX126xSetRegulatorMode( RadioRegulatorMode_t mode )
{
    SX126xWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}

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

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 4800  , 0x1F },
    { 5800  , 0x17 },
    { 7300  , 0x0F },
    { 9700  , 0x1E },
    { 11700 , 0x16 },
    { 14600 , 0x0E },
    { 19500 , 0x1D },
    { 23400 , 0x15 },
    { 29300 , 0x0D },
    { 39000 , 0x1C },
    { 46900 , 0x14 },
    { 58600 , 0x0C },
    { 78200 , 0x1B },
    { 93800 , 0x13 },
    { 117300, 0x0B },
    { 156200, 0x1A },
    { 187200, 0x12 },
    { 234300, 0x0A },
    { 312000, 0x19 },
    { 373600, 0x11 },
    { 467000, 0x09 },
    { 500000, 0x00 }, // Invalid Bandwidth
};


/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    if( bandwidth == 0 )
    {
        return( 0x1F );
    }

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth <= FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i+1].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void SX126xSetCrcSeed( uint16_t seed )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( seed >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( seed & 0xFF );
    SX126xWriteRegisters( REG_LR_CRCSEEDBASEADDR, buf, 2 );
}

void SX126xSetCrcPolynomial( uint16_t polynomial )
{
    uint8_t buf[2];
    buf[0] = ( uint8_t )( ( polynomial >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( polynomial & 0xFF );
    SX126xWriteRegisters( REG_LR_CRCPOLYBASEADDR, buf, 2 );
}


uint8_t SX126xSetSyncWord( uint8_t *syncWord )
{
    SX126xWriteRegisters( REG_LR_SYNCWORDBASEADDRESS, syncWord, 8 );
    return 0;
}


void SX126xSetWhiteningSeed( uint16_t seed )  //SX126X增白过程由一个9位的LFSR生成；
{
    uint8_t regValue = 0;
		regValue = SX126xReadRegister( REG_LR_WHITSEEDBASEADDR_MSB ) & 0xFE;
		regValue = ( ( seed >> 8 ) & 0x01 ) | regValue;
		SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_MSB, regValue ); // only 1 bit.
		SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_LSB, ( uint8_t )seed );
}
/**
  * @简介:
  * @参数：无
  * @返回值：无
*/
void SX126xSetFSKModulationParams(void)
{
	 uint8_t n;
    uint32_t tempVal = 0;
    uint8_t buf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
   
        n = 8;
        tempVal = ( uint32_t )( 32 * ( ( double )XTAL_FREQ / ( double ) G_FSKConfig.BitRate) );
        buf[0] = ( tempVal >> 16 ) & 0xFF;
        buf[1] = ( tempVal >> 8 ) & 0xFF;
        buf[2] = tempVal & 0xFF;
        buf[3] = G_FSKConfig.ModulationShaping;
        buf[4] = RadioGetFskBandwidthRegValue(G_FSKConfig.Bandwidth);
        tempVal = ( uint32_t )( ( double )G_FSKConfig.Fdev / ( double )FREQ_STEP );
        buf[5] = ( tempVal >> 16 ) & 0xFF;
        buf[6] = ( tempVal >> 8 ) & 0xFF;
        buf[7] = ( tempVal& 0xFF );
        SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );
}

/**
  * @简介:前导码、报头格式、PL、CRC等设置
  * @参数：无
  * @返回值：无
*/
void SX126xSetFSKPacketParams(void)
{
	  uint8_t n;
    uint8_t crcVal = 0;
    uint8_t buf[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
		
		 if( G_FSKConfig.CrcLength == RADIO_CRC_2_BYTES_IBM )
        {
            SX126xSetCrcSeed( CRC_IBM_SEED );
            SX126xSetCrcPolynomial( CRC_POLYNOMIAL_IBM );
            crcVal = RADIO_CRC_2_BYTES;
        }
        else if( G_FSKConfig.CrcLength == RADIO_CRC_2_BYTES_CCIT )
        {
            SX126xSetCrcSeed( CRC_CCITT_SEED );
            SX126xSetCrcPolynomial( CRC_POLYNOMIAL_CCITT );
            crcVal = RADIO_CRC_2_BYTES_INV;
        }
        else
        {
            crcVal = G_FSKConfig.CrcLength;
        }
			
		  	n = 9;
        buf[0] = ( (G_FSKConfig.PreambleLength<<3) >> 8 ) & 0xFF;     //PreambleLength<<3 convert from byte to bit
        buf[1] = (G_FSKConfig.PreambleLength<<3); // convert from byte to bit
        buf[2] = G_FSKConfig.PreambleMinDetect;
        buf[3] = ( G_FSKConfig.SyncWordLength << 3); // convert from byte to bit
        buf[4] = G_FSKConfig.AddrComp;
        buf[5] = G_FSKConfig.HeaderType;
        buf[6] = G_FSKConfig.PayloadLength;
        buf[7] = crcVal;
        buf[8] = G_FSKConfig.DcFree;
			  SX126xWriteCommand( RADIO_SET_PACKETPARAMS, buf, n );
		
}

/**
* @简介:SX126X初始化
  * @参数：无
  * @返回值：tSX126xError   错误枚举内容
  */
tSX126x_FSK_Error SX126x_FSK_init(void)
{
    if(false == SX126X_FSKConfig_Check()) //如果输入参数错误
    {
      return PARAMETER_INVALID;  //报错输入
    }
		SX126X_SPI_Init();              //SPI初始化
    SX126X_InitIo();                // PAIO口初始化
		
		/*------------------------------------------------
        SX126X Lora模式初始化 --------  */
    SX126X_Reset();                 //复位RF
		SX126xWakeup( );
    SX126xSetStandby(STDBY_RC);    //切换到LoRamode，standby状态
		SX126xWriteRegister(TxClampConfig, SX126xReadRegister(TxClampConfig) | 0x1E); //SX126X 芯片PA内部有过压保护，如果失配会导致功率降低问题；
		                                                                //修改寄存器TxClampConfig来优化PA的阈值，位4-1必须
		SX126xSetRegulatorMode( USE_DCDC );
	
	if (VERSION==LSD4RF_2R717N40)
		{
			SX126xClearDeviceErrors(); //如果使用DIO3去控制TCXO，那么每当上电之后或者是以Cold-Start的条件从Sleep唤醒，需要
		                              //调用ClearDeviceErrors命令去清除OSC_START_ERR标志
			                            //如果不清除在低温冷启动时可能会出现TCXO不工作情况---2019.07.29
			SX126xSetDio3AsTcxoCtrl(TCXO_CTRL_2_7V, 320);  //超时时间设置 320*15.625us=5ms
			SX126xWriteRegister( REG_XTA_TRIM, 0x2F ); 
			
			CalibrationParams_t calibParam;
			calibParam.Value = 0x7F;
			SX126xCalibrate(calibParam); //Calibrate the RC13, RC64, ADC , Image、PLL
																	//防止上电之后或者是以Cold-Start的条件从Sleep唤醒，或者低温运行时sleep状态切换过来时出现PLL Lock，XoscStart等错误现象，导致RF无法工作；
		}
		
			
		
		SX126xSetPacketType( PACKET_TYPE_GFSK );  //设置成FSK模式
    /*------------------------------------------------
    SPI 验证                   */
    uint8_t test = 0;
    SX126xWriteRegister( REG_LR_PAYLOADLENGTH, 0x09);   //选一个用不到的寄存器来做验证
    test = SX126xReadRegister( REG_LR_PAYLOADLENGTH );
    if(test != 0x09)
       return SPI_READCHECK_WRONG;
		
		SX126xWriteRegister(REG_RX_GAIN, 0x96);//开启Rx Boosted gain,默认0x94不开启
    SX126xSetRfFrequency(G_FSKConfig.FSK_Freq);
    SX126xSetRfTxPower( G_FSKConfig.PowerCfig );
		SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetFSKModulationParams( );   //BR、BW、Fdev
    SX126xSetFSKPacketParams( );  //PreambleLength、DcFree、CRC、PreambleMinDetect、SyncWordLength、AddrComp、HeaderType、
		
		if(G_FSKConfig.AddrComp == RADIO_ADDRESSCOMP_FILT_NODE) //设置节点地址
		{
			SX126xWriteRegister(NodeAddrReg,0x11);
		}
		if(G_FSKConfig.AddrComp == RADIO_ADDRESSCOMP_FILT_NODE_BROAD)//设置节点地址和广播地址
		{
			SX126xWriteRegister(NodeAddrReg,0x11);
			SX126xWriteRegister(BroadcastReg,0x12);
		}
		
	  SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );//同步字内容设置，有效个数取决于SyncWordLength设置大小
		SX126xSetWhiteningSeed( 0x01FF );
    //SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    return NORMAL;
}