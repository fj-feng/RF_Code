/**
  ******************************************************************************
  * �ļ��� ��   SX126X_Driver.c
  * ����   ��   LSD RF Team
  * �汾   ��   V1.0.0
  * ʱ��   ��   15-Aug-2018
  * �ļ�������
  *     ���ļ�ΪSX126Xģ��������㣬������SX126Xģ��Ĺ���ģʽ���ƣ�FIFO��������
  *�书�ʡ�Ƶ�ʵ����ã��Լ�һЩ���ݴ���Ĺ�������
  *    �ͻ���ʹ��SX126Xģ��ʱ����Ҫ��ֲ���ļ������SX126X_HAL.c�и����������ܶ���
  *ʵ���Һ������ȶ�û�иĶ����Ǳ��ļ����Բ���������ֱ��ʹ�ã��ͻ���Ӧ�ò�ֱ�ӵ�
  *�ñ��ļ������Ϳ���ʵ�ָ��ֲ�����
*******************************************************************************/
#include "SX126X_Driver.h"
#include "stm32l4xx_hal.h"
#include "timer.h"
#define VERSION        LSD4RFC_2L722N10     //ѡ����ѡ�ò�Ʒ�ͺţ����磺
                                                                  //     LSD4RFC_2L722N10																							         
																																  //     LSD4RFC_2L714N10
																																	 //    L_LRMRL22_97NN4
 
 
//static RadioPacketTypes_t PacketType;

float G_BandWidthKHz = 500.0;//���ؼ���Symbol����ʹ��
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
  ע�⣬���ļ��ĺ���ʵ���˴�RF��������MCU�ļ���

*/
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
//	  IO_SET(GPIO_PIN_RESET);
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
  * @��飺
  * @������
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
  * @��飺
  * @������
  * @����ֵ����
*/
void SX126xSetRegulatorMode( RadioRegulatorMode_t mode )
{
    SX126xWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
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


#if (VERSION==LSD4RFC_2L714N10)
/**
  * @��飺���书�����ã��ú��������ǽ��Ӳ�LSD4RFC-2L714N10���ǹ��ļ������������ã� 
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
  * @��飺���书�����ã��ú��������ǽ��Ӳ��LSD4RFC-2L722N10���ǹ��ļ������������ã� 
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
  * @���:
  * @������
  * @����ֵ����
*/
void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_800_US );
}

/**
  * @���:���Ʋ������� ��SF��BW��CR���������Ż�����
  * @��������
  * @����ֵ����
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

		
	//��LoRaģʽ��ʱ��BWΪ500KHz ʱ�Ĵ���0x0889�ĵڶ�λΪ0����������BW��GFSKģʽʱ�ڶ�λӦ��Ϊ1
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
  * @���:ǰ���롢��ͷ��ʽ��PL��CRC������
  * @��������
  * @����ֵ����
*/
void SX126xSetLoraPacketParams(void)
{
    //SX126xSetPacketType( PACKET_TYPE_LORA );
    uint8_t n;
    uint8_t buf[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

		//SF6��SF7ʱҪ��ǰ�����ȴ��ڵ���12
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
  * @���:
  * @������
  * @����ֵ����
*/
void SX126xSetStopRxTimerOnPreambleDetect( bool enable )
{
    SX126xWriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, ( uint8_t* )&enable, 1 );
}
/**
  * @���:
  * @������
  * @����ֵ����
*/
void SX126xSetLoRaSymbNumTimeout( uint8_t SymbNum )
{
    SX126xWriteCommand( RADIO_SET_LORASYMBTIMEOUT, &SymbNum, 1 );
}
/**
  * @��飺SX126X ����Ƶ�ʼ���ж�
  * @��������
  * @����ֵ��true or false
  */
bool LoRaConfig_Check(void)
{
if((G_LoRaConfig.LoRa_Freq < 410000000) || (G_LoRaConfig.LoRa_Freq > 960000000))
        return false;

G_LoRaConfig.BandWidth = (RadioLoRaBandwidths_t)(G_LoRaConfig.BandWidth & 0x0F);

    if((0x3>=G_LoRaConfig.BandWidth)&(0x7<=G_LoRaConfig.BandWidth))//LLCC68 BW ����
        return false;
  //����BandWidth
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
		
		//LLCC68 ��������
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
		
		//����LoRa��Ԫ���ڣ���λms
    G_TsXms = (2 << ((G_LoRaConfig.SpreadingFactor) - 1)) / G_BandWidthKHz;
		
		 G_LoRaConfig.CodingRate = (RadioLoRaCodingRates_t)(G_LoRaConfig.CodingRate & 0x07);
    if((G_LoRaConfig.CodingRate > LORA_CR_4_8) || (G_LoRaConfig.CodingRate < LORA_CR_4_5))
        return false;
    if(G_LoRaConfig.PowerCfig > 22)
        return false;
		 return true;
   
}
/**
* @���:SX126X��ʼ��
  * @��������
  * @����ֵ��tSX126xError   ����ö������
  */

tSX126xError SX126x_Lora_init(void)
{
    if(false == LoRaConfig_Check()) //��������������
    {
      return PARAMETER_INVALID;  //��������
    }
		SX126X_SPI_Init();              //SPI��ʼ��
    SX126X_InitIo();                // PAIO�ڳ�ʼ��
		
		/*------------------------------------------------
        SX126X Loraģʽ��ʼ�� --------  */
    SX126X_Reset();                 //��λRF
		SX126xWakeup();
		SX126xSetStandby(STDBY_RC);    //�л���LoRamode��standby״̬
		SX126xWriteRegister(TxClampConfig, SX126xReadRegister(TxClampConfig) | 0x1E); //SX126X оƬPA�ڲ��й�ѹ���������ʧ��ᵼ�¹��ʽ������⣻
		                                                                //�޸ļĴ���TxClampConfig���Ż�PA����ֵ��λ4-1����
		                                                                //����Ϊ"1111����Ĭ��Ϊ0100��---2019.07.31
		SX126xSetRegulatorMode( USE_DCDC );
		SX126xSetDio2AsRfSwitchCtrl(1);
		SX126xSetPacketType( PACKET_TYPE_LORA );  //���ó�LORAģʽ
    /*------------------------------------------------
    SPI ��֤                   */
    uint8_t test = 0;
    SX126xWriteRegister( REG_LR_PAYLOADLENGTH, 0x09);   //ѡһ���ò����ļĴ���������֤
    test = SX126xReadRegister( REG_LR_PAYLOADLENGTH );
    if(test != 0x09)
        return SPI_READCHECK_WRONG;
		
		SX126xWriteRegister(REG_RX_GAIN, 0x96);//����Rx Boosted gain,Ĭ��0x94������
    SX126xSetRfFrequency(G_LoRaConfig.LoRa_Freq);
    SX126xSetRfTxPower( G_LoRaConfig.PowerCfig );
		SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetLoraModulationParams();   //SF��BW��CR��LOWDATARATEOPTIMIZE
    SX126xSetLoraPacketParams();  //PreambleLength��HeaderType��PayloadLength��CrcMode��InvertIQ
    //SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    return NORMAL;
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
  * @���:���˲ʱ�RSSIֵ
  * @��������
  * @����ֵ��RSSIֵ
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
  * @���:��ȡ���ݰ�RSSI��SNRֵ
  * @������
  * @����ֵ����
*/
void SX126xGetPacketStatus( S_LoRaPara *pktStatus )
{
    uint8_t status[3];

    SX126xReadCommand( RADIO_GET_PACKETSTATUS, status, 3 );

    pktStatus->AvgPacket_RSSI = -status[0] >> 1;   //��һ�����ݰ���RSSIƽ��ֵ
    ( status[1] < 128 ) ? ( pktStatus->Packet_SNR = status[1] >> 2 ) : ( pktStatus->Packet_SNR = ( ( status[1] - 256 ) >> 2 ) );
    pktStatus->LastPacket_RSSI = -status[2] >> 1;
}

void SX126xSetFs( void )
{
    SX126xWriteCommand( RADIO_SET_FS, 0, 0 );
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
/**
  * @���:�������ݰ�
  * @������data
  * @����ֵ����
*/
extern uint8_t communication_states;
void SX126X_TxPacket( uint8_t*data )
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
//    SX126xSetLoraModulationParams( );   //SF��BW��CR��LOWDATARATEOPTIMIZE
    SX126xSetLoraPacketParams( );  //PreambleLength��HeaderType��PayloadLength��CrcMode��InvertIQ
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSendPayload( data, G_LoRaConfig.PayloadLength, 0x2fffff );// �����˷��䳬ʱ��ʱ��Ϊ��0x2fffff*15.625��50s,
	             //����һ����ȫ��ʩ����������ĳ��ͻ����ִ���˸�����ǲ�����TxDOne,��ʱ����Tx��ʱ�Ϳ��Ա�֤״̬�������ң��û�����ʵ��Ӧ����������������ó�ʱʱ��
} 
/**
  * @���  :�������ݰ�
  * @����  ��uint8_t*cbuf:��������ָ��
  * @����ֵ����
  * @˵��  �����������ݺ�DIO1�ӵ͵�ƽ��ɸߵ�ƽ
*/
void LSD_RF_RXmode()
{
	 DIO1_EnableInterrupt();
   SX126X_StartRx();//�������ģʽ��׼����������
}

/**
  * @���  :�������ݰ�
  * @����  ��uint8_t*cbuf:��������ָ��
  * @����ֵ����
  * @˵��  �����ݷ�����ɺ�DIO1�ӵ͵�ƽ��ɸߵ�ƽ��ÿ�ε��ôκ��������Զ��ֽ�DIO1��Ϊ�͵�ƽ���ȴ��ߵ�ƽ
*/
void LSD_RF_SendPacket(uint8_t*cbuf)
{
   unsigned long int j=16777215;                   //��ʱ�ã��û���Ҫ����ʵ�����������
	 DIO1_DisableInterrupt();
   SX126X_TxPacket(cbuf );
   while((DIO1_GetState()== GPIO_PIN_RESET)&&j)j--;  //�ȴ�DIO1��ƽΪ��
	 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);                                   
}

/**
  * @���:�������
  * @��������
  * @����ֵ����
*/
void SX126X_StartRx( void )
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
    SX126xSetLoRaSymbNumTimeout(0); //����������֤��ȷLoRa�źŵ��ַ��������������������Ϊ0���ɣ�
	      //һ������Ϊ0ʱ������ʾ���ջ���֤һ��LoRa�ź��ַ����������ڸ���ͻ��������ڼ��͵Ŀ������У����ԼӴ���֤��ֵ�������һ������ȷ�ԣ�
//    SX126xSetLoraModulationParams();   //SF��BW��CR��LOWDATARATEOPTIMIZE����
//    SX126xSetLoraPacketParams();  //PreambleLength��HeaderType��PayloadLength��CrcMode��InvertIQ����
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetRx( 0x2fffff ); // �����˽��ճ�ʱ��ʱ��Ϊ��0x2fffff*15.625us��50s,�û�����ʵ��Ӧ����������������ó�ʱʱ�䣻��Ĭ������Ǽ�⵽��ͷ��LoRa����ͬ���֣�FSK���Ż���ֹ�ö�ʱ����
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
void SX126X_RxPacket(uint8_t*cbuf)
{

    SX126xGetPacketStatus(&G_LoRaPara);//�յ�һ�����ݰ��󣬷���SNR��RSSI�ȸ�����Ϣ
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    if( G_LoRaConfig.HeaderType == LORA_PACKET_EXPLICIT ) {
        //��ͷ��ô�ӼĴ����ж���������G_LoRaConfig���ó��ȶ�ȡFIFO
        SX126xGetRxBufferStatus(&G_LoRaConfig.PayloadLength);
    }
    SX126xGetPayload(cbuf, &G_LoRaConfig.PayloadLength);

}

/**
  * @��飺WOR��ʼ��
  * @��������
  * @����ֵ����
  */

/*
cadSymbolNum:��ʾ�������CAD�źŵķ�������Ӱ�칦�ļ�CAD�����ʣ�����Խ�󣬹���Խ��������Խ�ͣ�
cadDetPeak��
cadDetMin����������������ú��������йأ���Ӱ����SF��BW��
cadExitMode:CAD_ONLY\CAD_RX ִָ����CAD������״̬��
cadTimeout:ָCAD��ɺ��������RX��ִ��RX��ʱ�䣻
����cadSymbolNum��cadDetPeak��cadExitMode�⼸�������Ƽ����£�
1������ͨ�ųɹ�������Ƽ���
  SF   	 cadSymbolNum 				cadDetPeak				cadExitMode						CAD Consumption(nAh)
	7			LORA_CAD_02_SYMBOL				22								10											2.84				
	8			LORA_CAD_02_SYMBOL				22								10											5.75
	9			LORA_CAD_04_SYMBOL				23								10											20.44
	10		LORA_CAD_04_SYMBOL				24								10											41.36
	11		LORA_CAD_04_SYMBOL				25								10											134.55

2������ͨ�ųɹ��ʼ��ݿ�������������Ƽ���
  SF   	 cadSymbolNum 				cadDetPeak				cadExitMode						CAD Consumption(nAh)
	7			LORA_CAD_02_SYMBOL				22								10											2.84				
	8			LORA_CAD_02_SYMBOL				22								10											5.75
	9			LORA_CAD_02_SYMBOL				24								10											11.7
	10		LORA_CAD_02_SYMBOL				25								10											23.86
	11		LORA_CAD_02_SYMBOL				26								10											48.79
	
	�����Ƽ��Ǹ���ʵ�ʲ��Խ���������ģ�ʵ��Ӧ�������и��ϸ�Ҫ��ɵ�����������ʵ���жϽ����
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
  * @��飺SX126X ��CAD
  * @��������
  * @����ֵ����
  */
void SX126xSetCad( void )
{
    SX126xWriteCommand( RADIO_SET_CAD, 0, 0 );
}
/**
  * @��飺SX126X CAD��ʼ��
  * @��������
  * @����ֵ����
  */
void SX126X_CADinit(void)
{
	
  SX126xSetStandby(STDBY_RC);
	SX126xSetPacketType( PACKET_TYPE_LORA );  //���ó�LORAģʽ
	SX126xSetRegulatorMode( USE_DCDC );
	
	SX126xSetCadParams(  LORA_CAD_02_SYMBOL, 22, 10, LORA_CAD_ONLY, 0xff );//���ﰴ��SF=7ʱ�Ƽ����ã�����������SF��BW���ü����Ŀ����йأ����Բ��ձ�����˵������
	
	SX126XWriteRxTx(false);
	SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
	SX126xSetDioIrqParams( IRQ_CAD_DONE|IRQ_CAD_ACTIVITY_DETECTED ,
                           IRQ_CAD_DONE ,
                           IRQ_RADIO_NONE,
	                         IRQ_RADIO_NONE );
													 //(p1,p2,p3,p4),P1:���ĸ���P2:ӳ�䵽DIO1��P3��ӳ�䵽DIO2��P4ӳ�䵽DIO3��
}
/**
  * @��飺WOR��ʼ��
  * @��������
  * @����ֵ����
  */
void SX126X_WORInit(void)
{
    SX126X_CADinit();        //CAD���ܳ�ʼ��
}
/**
  * @��飺SX126X ����CAD�������ŵ����һ��   ����ʱ��ԼΪ(2^SF+32)/BW
  * @��������
  * @����ֵ����
  */
void SX126X_CAD_Sample(void)
{
		SX126xCalibrate_err();	//��ֹ�ϵ�֮���������Cold-Start��������Sleep���ѣ����ߵ�������ʱsleep״̬�л�����ʱ����PLL Lock��XoscStart�ȴ������󣬵���RF�޷�������
	  SX126xSetStandby(STDBY_RC);
    SX126xSetCad();
}
/**
  * @��飺ִ��WOR����
  * @������uint8_t cclen  0������˯��;1������CAD���ģʽ
  * @����ֵ����
  */
void SX126X_WOR_Execute(uint8_t cclen)
{
    switch(cclen)
    {
    case 0:   //����˯��
    {
        SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
			  SX126xSetStandby(STDBY_RC);
        SX126xSleep();  //����˯��ģʽ
    }
    break;
    case 1:   //����CAD���ģʽ
    {
        SX126X_CAD_Sample();     //����CADһ��
    }
    break;
    default:
        break;
    }
}


/**
  * @��飺WOR��RX
  * @��������
  * @����ֵ����
  */
void SX126X_WOR_Exit(void)
{
		SX126xCalibrate_err();	//��ֹ�ϵ�֮���������Cold-Start��������Sleep���ѣ����ߵ�������ʱsleep״̬�л�����ʱ����PLL Lock��XoscStart�ȴ������󣬵���RF�޷�������
    SX126xSetStandby(STDBY_RC);
  	SX126xSetPacketType( PACKET_TYPE_LORA );  //���ó�LORAģʽ
  	SX126xSetRegulatorMode( USE_DCDC );
	  SX126XWriteRxTx(false);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL );
    SX126xSetDioIrqParams( IRQ_RX_DONE|IRQ_CRC_ERROR, 
                           IRQ_RX_DONE,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );  //��RX��CRC�жϣ�ӳ��RX�жϵ�DIO1��
	  SX126xSetStopRxTimerOnPreambleDetect( false );
    SX126xSetLoRaSymbNumTimeout(0);
	  G_LoRaConfig.PreambleLength=0xffff;
//	  SX126xSetLoraModulationParams( );   //SF��BW��CR��LOWDATARATEOPTIMIZE
    SX126xSetLoraPacketParams( );  //PreambleLength��HeaderType��PayloadLength��CrcMode��InvertIQ����
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetRx( 0xffffff ); // Rx Continuout
}

/**
  * @��飺SX126X���ͻ��Ѱ�
  * @������uint8_t*data����������ָ��,ǰ������
  * @����ֵ����
  */
void SX126X_Awake(uint8_t*cbuf, uint16_t Preamble_Length)
{
		SX126xCalibrate_err();	//��ֹ�ϵ�֮���������Cold-Start��������Sleep���ѣ����ߵ�������ʱsleep״̬�л�����ʱ����PLL Lock��XoscStart�ȴ������󣬵���RF�޷�������
	  SX126xSetStandby(STDBY_RC);
	  SX126xSetRegulatorMode( USE_DCDC );
    SX126XWriteRxTx(true);
	  SX126xClearIrqStatus( IRQ_RADIO_ALL ); //clear flags
	  SX126xSetDioIrqParams( IRQ_TX_DONE ,
                           IRQ_TX_DONE ,
                           IRQ_RADIO_NONE,
	                         IRQ_RADIO_NONE );
													 //(p1,p2,p3,p4),P1:���ĸ���P2:ӳ�䵽DIO1��P3��ӳ�䵽DIO2��P4ӳ�䵽DIO3��
	  G_LoRaConfig.PreambleLength=Preamble_Length;
	  SX126xSetLoraPacketParams( );//PreambleLength��HeaderType��PayloadLength��CrcMode��InvertIQ����
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSendPayload( cbuf, G_LoRaConfig.PayloadLength, 0xffffff );
}
