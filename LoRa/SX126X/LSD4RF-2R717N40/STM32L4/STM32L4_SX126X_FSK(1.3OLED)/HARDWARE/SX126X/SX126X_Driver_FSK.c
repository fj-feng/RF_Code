#include "SX126X_Driver_FSK.h"
#include "stm32l4xx_hal.h"
#include "timer.h"


#define VERSION        LSD4RF_2R717N40     //ѡ����ѡ�ò�Ʒ�ͺţ����磺 LSD4RF_2R717N40
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
  * @��飺SX126X ����Ƶ�ʼ���ж�
  * @��������
  * @����ֵ��true or false
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
  * @��飺SX126X ����
  * @��������
  * @����ֵ����
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
  * @��飺Standbyģʽ
  * @������STDBY_RC or STDBY_XOSC
  * @����ֵ����
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
  * @��飺��ȡ��ǰ����ģʽ
  * @��������
  * @����ֵ��OperatingMode
  */
RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}
/**
  * @��飺���SX126X�Ƿ���׼��״̬
  * @��������
  * @����ֵ����
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
  * @��飺д����
  * @������command��buffer��size
  * @����ֵ����
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
  * @��飺������
  * @������command��buffer��size
  * @����ֵ����
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
  * @��飺д�Ĵ�����
  * @������address��buffer��size
  * @����ֵ����
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
  * @��飺д�����Ĵ���
  * @������address��value
  * @����ֵ����
  */
void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}
/**
  * @��飺���Ĵ�����
  * @������address��buffer��size
  * @����ֵ����
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
  * @��飺�������Ĵ���
  * @������address
  * @����ֵ��data
  */
uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}
/**
  * @��飺дBuffer
  * @������offset,buffer,size
  * @����ֵ����
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
  * @��飺��Buffer
  * @������offset,buffer,size
  * @����ֵ����
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


//-------------------------SX126X ��������-----------------------//
//�ò��ֺ���Ϊ��SX126Xģ�鸴λ�����书�ʡ�����Ƶ�ʵȲ������ã���
//��SX126X����ģʽ���á����ݰ���д��
//--------------------------------------------------------------//

/**
  * @��飺SX126X  ��λ����
  * @��������
  * @����ֵ����
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
  * @��飺SX126X  IO��ʼ��������
  * @��������
  * @����ֵ����
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
  * @��飺SX126X  TX/RX��PA�л�
  * @������bool txEnable  �л��߼����棺��ΪTX���٣���ΪRX   ΪӲ������PA����IO��
  * @����ֵ:��
  */
void SX126XWriteRxTx( bool txEnable )
{
    if( txEnable != 0 )       //���Ϊ�棬ΪTX
    {
        SX126X_SWCTL1_OUTPUT(GPIO_PIN_RESET);
        SX126X_SWCTL2_OUTPUT(GPIO_PIN_SET);
    }
    else  //Ϊ�٣�ΪRX
    {
        SX126X_SWCTL1_OUTPUT(GPIO_PIN_SET);
        SX126X_SWCTL2_OUTPUT(GPIO_PIN_RESET);
    }
}
/**
  * @��飺SX126X �жϸ�Ƶ���أ���ֹ��Ƶ���ع��������ӹ���
  * @��������
  * @����ֵ:��
  */
void SX126XSwitchOff(void)
{
    SX126X_SWCTL1_OUTPUT(GPIO_PIN_RESET);
    SX126X_SWCTL2_OUTPUT(GPIO_PIN_RESET);
}

/**
  * @��飺DIO1�жϿ���ʹ��
  * @��������
  * @����ֵ����
  */
void DIO1_EnableInterrupt(void)
{
    SX126X_DIO1_INTENABLE();
}
/**
  * @��飺DIO0�жϹر�ʹ��
  * @��������
  * @����ֵ����
  */
void DIO1_DisableInterrupt(void)
{
    SX126X_DIO1_INTDISABLE();
}
/**
  * @��飺DIO1����״̬��ȡ
  * @��������
  * @����ֵ��State��ʾDIO1��ȡ�ĵ�ƽ���ߵ�ƽ"1",�͵�ƽ"0"
  */
GPIO_PinState DIO1_GetState(void)
{
    GPIO_PinState State;
    State = SX126X_DIO1_GetState();
    return State;
}
/**
  * @��飺DIO2�жϿ���ʹ��
  * @��������
  * @����ֵ����
  */
void DIO2_EnableInterrupt(void)
{
    SX126X_DIO2_INTENABLE();
}
/**
  * @��飺DIO2�жϹر�ʹ��
  * @��������
  * @����ֵ����
  */
void DIO2_DisableInterrupt(void)
{
    SX126X_DIO2_INTDISABLE();
}
/**
  * @��飺DIO2����״̬��ȡ
  * @��������
  * @����ֵ��State��ʾDIO2��ȡ�ĵ�ƽ���ߵ�ƽ"1",�͵�ƽ"0"
  */
GPIO_PinState DIO2_GetState(void)
{
    GPIO_PinState State;
    State = SX126X_DIO2_GetState();
    return State;
}
/**
  * @��飺Busy�жϿ���ʹ��
  * @��������
  * @����ֵ����
  */
void Busy_EnableInterrupt(void)
{
    SX126X_Busy_INTENABLE();
}
/**
  * @��飺Busy�жϹر�ʹ��
  * @��������
  * @����ֵ����
  */
void Busy_DisableInterrupt(void)
{
    SX126X_Busy_INTDISABLE();
}
/**
  * @��飺Busy����״̬��ȡ
  * @��������
  * @����ֵ��State��ʾBusy��ȡ�ĵ�ƽ���ߵ�ƽ"1",�͵�ƽ"0"
  */
GPIO_PinState Busy_GetState(void)
{
    GPIO_PinState State;
    State = SX126X_Busy_GetState();
    return State;
}

/**
  * @��飺Standbyģʽ
  * @������STDBY_RC or STDBY_XOSC
  * @����ֵ����
*/
void SX126xSleep( void )
{	 
    SleepParams_t params = { 0 };
    params.Fields.WarmStart = 1;
    SX126xSetSleep( params );
    SX126XSwitchOff();//�жϸ�Ƶ���أ���ֹ����ģʽ�¸�Ƶ�������ӹ���
		
   // HAL_Delay(2);
}
/**
  * @��飺SX126X����LORA˯��ģʽ
  * @��������
  * @����ֵ����
  */
void SX126xSetSleep( SleepParams_t sleepConfig )
{
    SX126xWriteCommand( RADIO_SET_SLEEP, &sleepConfig.Value, 1 );
	  OperatingMode = MODE_SLEEP;
}


/**
  * @��飺
  * @������
  * @����ֵ����
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
  * @���:Ƶ������
  * @������
  * @����ֵ����
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
  * @���:
  * @������
  * @����ֵ����
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
  * @���:
  * @������
  * @����ֵ����
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
  * @��飺���书�����ã��ú��������ǽ��Ӳ��LSD4RF-2R717N30 ���ǹ��ļ������������ã� 
  * @������Power����ֵ��Χ��-3~22��
  * @����ֵ:
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
       SX126xSetPaConfig( 0x04, 0x07, 0x00, 0x01 );//ʵ�����20dBm����
     }
        break;
    case 19:
     {
       power=22;
       SX126xSetPaConfig( 0x02, 0x07, 0x00, 0x01 );//ʵ�����19dBm����
     }
        break;
		 case 18: 
     {
       power=22;
       SX126xSetPaConfig( 0x03, 0x05, 0x00, 0x01 );//ʵ�����18dBm����
     }
    case 17: 
     {
       power=22;
       SX126xSetPaConfig( 0x02, 0x03, 0x00, 0x01 );//ʵ�����17dBm����
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
  * @��飺���书�����ã��ú��������ǽ��Ӳ�LSD4RF-2R714N10���ǹ��ļ������������ã� 
  * @������Power����ֵ��Χ��-3~22��
  * @����ֵ:
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
  * @��飺���书�����ã��ú��������ǽ��Ӳ��LSD4RF-2R717N40\LSD4RF-2R722N20\���ǹ��ļ������������ã� 
  * @������Power����ֵ��Χ��-3~22��
  * @����ֵ:
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
  * @��飺Loraģʽ
  * @������packetType
  * @����ֵ����
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
  * @���:���뷢��
  * @������
  * @����ֵ����
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
  * @���:�������
  * @������
  * @����ֵ����
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
  * @���:��������
  * @������PL��size��timeout
  * @����ֵ����
*/
void SX126xSendPayload( uint8_t *payload, uint8_t size, uint32_t timeout )
{
    SX126xWriteBuffer( 0x00, payload, size );
    SX126xSetTx( timeout );
}

void SX126X_FSKTxPacket( uint8_t*data )
{
	  SX126xSetStandby(STDBY_XOSC);  //�����������ʹ��TCXO�汾��STDBY_XOSC��STDBY_RC����Ӱ�죬���ʹ�õ���TCXO�汾����Ҫʹ��STDBY_XOSC������ʱʱ������ӵ��л���TXģʽ�����У�
	  
		SX126xCalibrate_err();	//��ֹ�ϵ�֮���������Cold-Start��������Sleep���ѣ����ߵ�������ʱsleep״̬�л�����ʱ����PLL Lock��XoscStart�ȴ������󣬵���RF�޷�������
		SX126xSetRegulatorMode( USE_DCDC );
    SX126XWriteRxTx(true);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    SX126xSetDioIrqParams( IRQ_TX_DONE|IRQ_RX_TX_TIMEOUT ,
                           IRQ_TX_DONE|IRQ_RX_TX_TIMEOUT ,
                          IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );//����Tx��Tx��ʱ�жϣ�ӳ�䵽DIO1��
                           //(p1,p2,p3,p4),P1:���ĸ���P2:ӳ�䵽DIO1��P3��ӳ�䵽DIO2��P4ӳ�䵽DIO3��
//    SX126xSetFSKModulationParams( );   //
    SX126xSetFSKPacketParams( );  //
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSendPayload( data, G_FSKConfig.PayloadLength, 0x2fffff );// �����˷��䳬ʱ��ʱ��Ϊ��0x2fffff*15.625��50s,
	             //����һ����ȫ��ʩ����������ĳ��ͻ����ִ���˸�����ǲ�����TxDOne,��ʱ����Tx��ʱ�Ϳ��Ա�֤״̬�������ң��û�����ʵ��Ӧ����������������ó�ʱʱ��
} 

/**
  * @���:�������
  * @��������
  * @����ֵ����
*/
void SX126X_FSKStartRx( void )
{
    SX126xSetStandby(STDBY_XOSC);  //�����������ʹ��TCXO�汾��STDBY_XOSC��STDBY_RC����Ӱ�죬���ʹ�õ���TCXO�汾����Ҫʹ��STDBY_XOSC������ʱʱ������ӵ��л���RXģʽ�����У�
	  SX126xCalibrate_err();	//��ֹ�ϵ�֮���������Cold-Start��������Sleep���ѣ����ߵ�������ʱsleep״̬�л�����ʱ����PLL Lock��XoscStart�ȴ������󣬵���RF�޷�������
		SX126xSetRegulatorMode( USE_DCDC );
    SX126XWriteRxTx(false);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    SX126xSetDioIrqParams( IRQ_RX_DONE|IRQ_CRC_ERROR|IRQ_RX_TX_TIMEOUT, 
                           IRQ_RX_DONE|IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );  //��RX��CRC��RX��ʱ�жϣ�ӳ��RX�жϵ�DIO1��
	  SX126xSetStopRxTimerOnPreambleDetect( false );//����ѡ��ʱ��ֹͣ�Ǽ��ǰ���뻹�Ǳ�ͷ��
	  //SX126xSetFSKModulationParams( );   //
    SX126xSetFSKPacketParams( );  //
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetRx( 0x9C400 ); // �����˽��ճ�ʱ��ʱ��Ϊ��0x9C400*15.625us��10s,�û�����ʵ��Ӧ����������������ó�ʱʱ�䣻��Ĭ������Ǽ�⵽��ͷ��LoRa����ͬ���֣�FSK���Ż���ֹ�ö�ʱ����
}


/**
  * @���:��ȡ���ݰ�RSSI��SNRֵ
  * @������
  * @����ֵ����
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
  * @���:
  * @������
  * @����ֵ����
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
    }                         //������ж���ʵ������жϱ�ͷΪ��������һ���ģ��������Ĵ����������δ������������֤�ǵ�Ч�����ġ�
    else
    {
        *payloadLength = status[0];
    }
    // *rxStartBufferPointer = status[1];
}
/**
  * @���:��ȡ����
  * @������buffer��size
  * @����ֵ����
*/
void SX126xGetPayload( uint8_t *buffer, uint8_t *size)
{
    SX126xReadBuffer( 0x00, buffer, *size );
}

/**
  * @��飺RF�������ݰ�
  * @������uint8_t*cbuf��������ָ��
  * @����ֵ����
  */
void SX126X_FSKRxPacket(uint8_t*cbuf)
{

    SX126xGetPacketStatus(&G_FSKPara);//�յ�һ�����ݰ��󣬷���SNR��RSSI�ȸ�����Ϣ
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    if( G_FSKConfig.HeaderType == RADIO_PACKET_VARIABLE_LENGTH ) {
        //��ͷ��ô�ӼĴ����ж���������G_LoRaConfig���ó��ȶ�ȡFIFO
        SX126xGetRxBufferStatus(&G_FSKConfig.PayloadLength);
    }
    SX126xGetPayload(cbuf, &G_FSKConfig.PayloadLength);
}

/**
  * @���:���־
  * @������irq
  * @����ֵ����
*/
void SX126xClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    SX126xWriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}
/**
  * @���:��ȡ��־
  * @��������
  * @����ֵ����Ӧ��־
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
  * @���:����LoRa���ݰ�����ģʽʱ������RxTimeout���������Ա�ͷ��Implicit Mode��ģʽʱ�����ܳ��ֽ�����ɺ󣬳�ʱ��ʱ�����ر�������Ҫ����ֹͣRTC�������ʱ�¼�
  * @������
  * @����ֵ����
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
  * @���:DIO2���ؿ���
  * @������enable
  * @����ֵ����
*/
void SX126xSetDio2AsRfSwitchCtrl( uint8_t enable )
{
    SX126xWriteCommand( RADIO_SET_RFSWITCHMODE, &enable, 1 );
}

/**
  * @���:DIO3 TCXO��Դ���Ƽ����ܿ���
  * @������enable
  * @����ֵ����
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
  * @��飺
  * @������
  * @����ֵ����
*/
void SX126xSetRegulatorMode( RadioRegulatorMode_t mode )
{
    SX126xWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}

/**
  * @���:
  * @������
  * @����ֵ����
*/
void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_800_US );
}
/**
  * @��飺
  * @������
  * @����ֵ����
*/
void SX126xSetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX126xWriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}

/**
  * @��飺
  * @������
  * @����ֵ����
*/
void SX126xCalibrate( CalibrationParams_t calibParam )
{
    SX126xWriteCommand( RADIO_CALIBRATE, ( uint8_t* )&calibParam, 1 );
}

/**
  * @��飺�ú�����Ҫ���ڷ�ֹ�����µ��������PLL Lock���쳣ʱ�����ܳ���RF�޷���������ʱ��������־������У׼
  * @������
  * @����ֵ����
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


void SX126xSetWhiteningSeed( uint16_t seed )  //SX126X���׹�����һ��9λ��LFSR���ɣ�
{
    uint8_t regValue = 0;
		regValue = SX126xReadRegister( REG_LR_WHITSEEDBASEADDR_MSB ) & 0xFE;
		regValue = ( ( seed >> 8 ) & 0x01 ) | regValue;
		SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_MSB, regValue ); // only 1 bit.
		SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_LSB, ( uint8_t )seed );
}
/**
  * @���:
  * @��������
  * @����ֵ����
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
  * @���:ǰ���롢��ͷ��ʽ��PL��CRC������
  * @��������
  * @����ֵ����
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
* @���:SX126X��ʼ��
  * @��������
  * @����ֵ��tSX126xError   ����ö������
  */
tSX126x_FSK_Error SX126x_FSK_init(void)
{
    if(false == SX126X_FSKConfig_Check()) //��������������
    {
      return PARAMETER_INVALID;  //��������
    }
		SX126X_SPI_Init();              //SPI��ʼ��
    SX126X_InitIo();                // PAIO�ڳ�ʼ��
		
		/*------------------------------------------------
        SX126X Loraģʽ��ʼ�� --------  */
    SX126X_Reset();                 //��λRF
		SX126xWakeup( );
    SX126xSetStandby(STDBY_RC);    //�л���LoRamode��standby״̬
		SX126xWriteRegister(TxClampConfig, SX126xReadRegister(TxClampConfig) | 0x1E); //SX126X оƬPA�ڲ��й�ѹ���������ʧ��ᵼ�¹��ʽ������⣻
		                                                                //�޸ļĴ���TxClampConfig���Ż�PA����ֵ��λ4-1����
		SX126xSetRegulatorMode( USE_DCDC );
	
	if (VERSION==LSD4RF_2R717N40)
		{
			SX126xClearDeviceErrors(); //���ʹ��DIO3ȥ����TCXO����ôÿ���ϵ�֮���������Cold-Start��������Sleep���ѣ���Ҫ
		                              //����ClearDeviceErrors����ȥ���OSC_START_ERR��־
			                            //���������ڵ���������ʱ���ܻ����TCXO���������---2019.07.29
			SX126xSetDio3AsTcxoCtrl(TCXO_CTRL_2_7V, 320);  //��ʱʱ������ 320*15.625us=5ms
			SX126xWriteRegister( REG_XTA_TRIM, 0x2F ); 
			
			CalibrationParams_t calibParam;
			calibParam.Value = 0x7F;
			SX126xCalibrate(calibParam); //Calibrate the RC13, RC64, ADC , Image��PLL
																	//��ֹ�ϵ�֮���������Cold-Start��������Sleep���ѣ����ߵ�������ʱsleep״̬�л�����ʱ����PLL Lock��XoscStart�ȴ������󣬵���RF�޷�������
		}
		
			
		
		SX126xSetPacketType( PACKET_TYPE_GFSK );  //���ó�FSKģʽ
    /*------------------------------------------------
    SPI ��֤                   */
    uint8_t test = 0;
    SX126xWriteRegister( REG_LR_PAYLOADLENGTH, 0x09);   //ѡһ���ò����ļĴ���������֤
    test = SX126xReadRegister( REG_LR_PAYLOADLENGTH );
    if(test != 0x09)
       return SPI_READCHECK_WRONG;
		
		SX126xWriteRegister(REG_RX_GAIN, 0x96);//����Rx Boosted gain,Ĭ��0x94������
    SX126xSetRfFrequency(G_FSKConfig.FSK_Freq);
    SX126xSetRfTxPower( G_FSKConfig.PowerCfig );
		SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetFSKModulationParams( );   //BR��BW��Fdev
    SX126xSetFSKPacketParams( );  //PreambleLength��DcFree��CRC��PreambleMinDetect��SyncWordLength��AddrComp��HeaderType��
		
		if(G_FSKConfig.AddrComp == RADIO_ADDRESSCOMP_FILT_NODE) //���ýڵ��ַ
		{
			SX126xWriteRegister(NodeAddrReg,0x11);
		}
		if(G_FSKConfig.AddrComp == RADIO_ADDRESSCOMP_FILT_NODE_BROAD)//���ýڵ��ַ�͹㲥��ַ
		{
			SX126xWriteRegister(NodeAddrReg,0x11);
			SX126xWriteRegister(BroadcastReg,0x12);
		}
		
	  SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );//ͬ�����������ã���Ч����ȡ����SyncWordLength���ô�С
		SX126xSetWhiteningSeed( 0x01FF );
    //SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    return NORMAL;
}