#include <stdint.h>
#include <math.h>
#ifndef SX126X_DRIVER_FSK_H
#define SX126X_DRIVER_FSK_H
#include "SX126X_Hal.h"

#define  LSD4RF_2R717N40      1
#define  LSD4RF_2R717N30      2
#define  LSD4RF_2R722N20      3
#define  LSD4RF_2R714N10      4
#define  L_LRMFP20_77NN4      5


#define XTAL_FREQ                                   ( double )32000000
#define FREQ_DIV                                    ( double )pow( 2.0, 25.0 )
#define FREQ_STEP                                   ( double )( XTAL_FREQ / FREQ_DIV )

#define RX_BUFFER_SIZE                              256


/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME                           3 // [ms]

/*!
 * \brief Compensation delay for SetAutoTx/Rx functions in 15.625 microseconds
 */
#define AUTO_RX_TX_OFFSET                           2

/*!
 * \brief LFSR initial value to compute IBM type CRC
 */
#define CRC_IBM_SEED                                0xFFFF

/*!
 * \brief LFSR initial value to compute CCIT type CRC
 */
#define CRC_CCITT_SEED                              0x1D0F

/*!
 * \brief Polynomial used to compute IBM CRC
 */
#define CRC_POLYNOMIAL_IBM                          0x8005

/*!
 * \brief Polynomial used to compute CCIT CRC
 */
#define CRC_POLYNOMIAL_CCITT                        0x1021

/*!
 * \brief The address of the register holding the first byte defining the CRC seed
 *
 */
#define REG_LR_CRCSEEDBASEADDR                      0x06BC

/*!
 * \brief The address of the register holding the first byte defining the CRC polynomial
 */
#define REG_LR_CRCPOLYBASEADDR                      0x06BE

/*!
 * \brief The address of the register holding the first byte defining the whitening seed
 */
#define REG_LR_WHITSEEDBASEADDR_MSB                 0x06B8
#define REG_LR_WHITSEEDBASEADDR_LSB                 0x06B9

/*!
 * \brief The address of the register holding the packet configuration
 */
#define REG_LR_PACKETPARAMS                         0x0704

/*!
 * \brief The address of the register holding the payload size
 */
#define REG_LR_PAYLOADLENGTH                        0x0702

/*!
 * \brief The addresses of the registers holding SyncWords values
 */
#define REG_LR_SYNCWORDBASEADDRESS                  0x06C0

/*!
 * \brief The addresses of the register holding LoRa Modem SyncWord value
 */
#define REG_LR_SYNCWORD                             0x0740

/*!
 * Syncword for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x1424

/*!
 * Syncword for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x3444

/*!
 * The address of the register giving a 4 bytes random number
 */
#define RANDOM_NUMBER_GENERATORBASEADDR             0x0819

/*!
 * The address of the register holding RX Gain value (0x94: power saving, 0x96: rx boosted)
 */
#define REG_RX_GAIN                                 0x08AC
 
/*!
 * The address of the register holding Bit Sync configuration
 */
#define REG_BIT_SYNC                                0x06AC


#define NodeAddrReg                                0x06CD
#define BroadcastReg                               0x06CE


/*!
 * Change the value on the device internal trimming capacitor
 */
#define REG_XTA_TRIM                                0x0911

/*!
 * Set the current max value in the over current protection
 */
#define REG_OCP                                     0x08E7

/*!
 * Set the PA clamping threshold
 */
#define TxClampConfig                               0x08D8


/*!
 * Set the 
 */
#define TxModulation                                0x0889
/*!
 * \brief Represents all possible opcode understood by the radio
 */
typedef enum RadioCommands_e
{
    RADIO_GET_STATUS                        = 0xC0,
    RADIO_WRITE_REGISTER                    = 0x0D,
    RADIO_READ_REGISTER                     = 0x1D,
    RADIO_WRITE_BUFFER                      = 0x0E,
    RADIO_READ_BUFFER                       = 0x1E,
    RADIO_SET_SLEEP                         = 0x84,
    RADIO_SET_STANDBY                       = 0x80,
    RADIO_SET_FS                            = 0xC1,
    RADIO_SET_TX                            = 0x83,
    RADIO_SET_RX                            = 0x82,
    RADIO_SET_RXDUTYCYCLE                   = 0x94,
    RADIO_SET_CAD                           = 0xC5,
    RADIO_SET_TXCONTINUOUSWAVE              = 0xD1,
    RADIO_SET_TXCONTINUOUSPREAMBLE          = 0xD2,
    RADIO_SET_PACKETTYPE                    = 0x8A,
    RADIO_GET_PACKETTYPE                    = 0x11,
    RADIO_SET_RFFREQUENCY                   = 0x86,
    RADIO_SET_TXPARAMS                      = 0x8E,
    RADIO_SET_PACONFIG                      = 0x95,
    RADIO_SET_CADPARAMS                     = 0x88,
    RADIO_SET_BUFFERBASEADDRESS             = 0x8F,
    RADIO_SET_MODULATIONPARAMS              = 0x8B,
    RADIO_SET_PACKETPARAMS                  = 0x8C,
    RADIO_GET_RXBUFFERSTATUS                = 0x13,
    RADIO_GET_PACKETSTATUS                  = 0x14,
    RADIO_GET_RSSIINST                      = 0x15,
    RADIO_GET_STATS                         = 0x10,
    RADIO_RESET_STATS                       = 0x00,
    RADIO_CFG_DIOIRQ                        = 0x08,
    RADIO_GET_IRQSTATUS                     = 0x12,
    RADIO_CLR_IRQSTATUS                     = 0x02,
    RADIO_CALIBRATE                         = 0x89,
    RADIO_CALIBRATEIMAGE                    = 0x98,
    RADIO_SET_REGULATORMODE                 = 0x96,
		RADIO_CLR_ERROR                         = 0x07,
    RADIO_SET_PRAMSWAPCMD                   = 0x8D,
    RADIO_GET_ERROR                         = 0x17,
    RADIO_SET_TCXOMODE                      = 0x97,
    RADIO_SET_TXFALLBACKMODE                = 0x93,
    RADIO_SET_RFSWITCHMODE                  = 0x9D,
    RADIO_SET_STOPRXTIMERONPREAMBLE         = 0x9F,
    RADIO_SET_LORASYMBTIMEOUT               = 0xA0,
}RadioCommands_t;



typedef enum
{
    MOD_SHAPING_OFF                         = 0x00,
    MOD_SHAPING_G_BT_03                     = 0x08,
    MOD_SHAPING_G_BT_05                     = 0x09,
    MOD_SHAPING_G_BT_07                     = 0x0A,
    MOD_SHAPING_G_BT_1                      = 0x0B,
}RadioModShapings_t;

/*!
 * \brief Represents the preamble length used to detect the packet on Rx side
 */
typedef enum
{
    RADIO_PREAMBLE_DETECTOR_OFF             = 0x00,         //!< Preamble detection length off
    RADIO_PREAMBLE_DETECTOR_08_BITS         = 0x04,         //!< Preamble detection length 8 bits
    RADIO_PREAMBLE_DETECTOR_16_BITS         = 0x05,         //!< Preamble detection length 16 bits
    RADIO_PREAMBLE_DETECTOR_24_BITS         = 0x06,         //!< Preamble detection length 24 bits
    RADIO_PREAMBLE_DETECTOR_32_BITS         = 0x07,         //!< Preamble detection length 32 bit
}RadioPreambleDetection_t;


/*!
 * \brief Represents the possible combinations of SyncWord correlators activated
 */
typedef enum
{
    RADIO_ADDRESSCOMP_FILT_OFF              = 0x00,         //!< No correlator turned on, i.e. do not search for SyncWord
    RADIO_ADDRESSCOMP_FILT_NODE             = 0x01,
    RADIO_ADDRESSCOMP_FILT_NODE_BROAD       = 0x02,
}RadioAddressComp_t;


/*!
 * \brief Represents the CRC length
 */
typedef enum
{
    RADIO_CRC_OFF                           = 0x01,         //!< No CRC in use
    RADIO_CRC_1_BYTES                       = 0x00,
    RADIO_CRC_2_BYTES                       = 0x02,
    RADIO_CRC_1_BYTES_INV                   = 0x04,
    RADIO_CRC_2_BYTES_INV                   = 0x06,
    RADIO_CRC_2_BYTES_IBM                   = 0xF1,
    RADIO_CRC_2_BYTES_CCIT                  = 0xF2,
}RadioCrcTypes_t;

/*!
 *  \brief Radio packet length mode
 */
typedef enum
{
    RADIO_PACKET_FIXED_LENGTH               = 0x00,         //!< The packet is known on both sides, no header included in the packet
    RADIO_PACKET_VARIABLE_LENGTH            = 0x01,         //!< The packet is on variable size, header included
}RadioPacketLengthModes_t;


/*!
 * \brief Radio whitening mode activated or deactivated
 */
typedef enum
{
    RADIO_DC_FREE_OFF                       = 0x00,
    RADIO_DC_FREEWHITENING                  = 0x01,
}RadioDcFree_t;



typedef struct S_FSKConfig
{
    uint32_t                      FSK_Freq;
	  int8_t                        PowerCfig;
	  uint32_t                      BitRate;
    uint32_t                      Fdev;
    RadioModShapings_t            ModulationShaping;
    uint32_t                       Bandwidth;
	  uint16_t                      PreambleLength;    //!< The preamble Tx length for GFSK packet type in bit
    RadioPreambleDetection_t      PreambleMinDetect; //!< The preamble Rx length minimal for GFSK packet type
    uint8_t                       SyncWordLength;    //!< The synchronization word length for GFSK packet type
    RadioAddressComp_t            AddrComp;          //!< Activated SyncWord correlators
    RadioPacketLengthModes_t      HeaderType;        //!< If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
    uint8_t                       PayloadLength;     //!< Size of the payload in the GFSK packet
    RadioCrcTypes_t               CrcLength;         //!< Size of the CRC block in the GFSK packet
    RadioDcFree_t                 DcFree;	
}S_FSKConfig;


typedef struct S_FSKPara
{
  uint8_t* BufferPointer;
  uint8_t PayloadLength;              //1~127
	uint8_t RxStatus;
	int8_t RssiAvg;                                //!< The averaged RSSI
	int8_t RssiSync;                               //!< The RSSI measured on last packet
}S_FSKPara;

/*!
 * \brief Represents the possible radio system error states
 */
typedef union
{
    struct
    {
        uint8_t Rc64kCalib              : 1;                    //!< RC 64kHz oscillator calibration failed
        uint8_t Rc13mCalib              : 1;                    //!< RC 13MHz oscillator calibration failed
        uint8_t PllCalib                : 1;                    //!< PLL calibration failed
        uint8_t AdcCalib                : 1;                    //!< ADC calibration failed
        uint8_t ImgCalib                : 1;                    //!< Image calibration failed
        uint8_t XoscStart               : 1;                    //!< XOSC oscillator failed to start
        uint8_t PllLock                 : 1;                    //!< PLL lock failed
        uint8_t                         : 1;                    //!< Buck converter failed to start
        uint8_t PaRamp                  : 1;                    //!< PA ramp failed
        uint8_t                         : 7;                    //!< Reserved
    }Fields;
    uint16_t Value;
}RadioError_t;

typedef enum
{
    NORMAL,                //正常      
    PARAMETER_INVALID,    //参数不可用
    SPI_READCHECK_WRONG,  //SPI出错
}tSX126x_FSK_Error;           //定义出错枚举

typedef enum
{
    PACKET_TYPE_GFSK                        = 0x00,
    PACKET_TYPE_LORA                        = 0x01,
    PACKET_TYPE_NONE                        = 0x0F,
}RadioPacketTypes_t;

typedef enum
{
    MODE_SLEEP                              = 0x00,         //! The radio is in sleep mode
    MODE_STDBY_RC,                                          //! The radio is in standby mode with RC oscillator
    MODE_STDBY_XOSC,                                        //! The radio is in standby mode with XOSC oscillator
    MODE_FS,                                                //! The radio is in frequency synthesis mode
    MODE_TX,                                                //! The radio is in transmit mode
    MODE_RX,                                                //! The radio is in receive mode
    MODE_RX_DC,                                             //! The radio is in receive duty cycle mode
    MODE_CAD                                                //! The radio is in channel activity detection mode
}RadioOperatingModes_t;

typedef enum
{
    USE_LDO                                 = 0x00, // default
    USE_DCDC                                = 0x01,
}RadioRegulatorMode_t;


typedef enum
{
    RADIO_RAMP_10_US                        = 0x00,
    RADIO_RAMP_20_US                        = 0x01,
    RADIO_RAMP_40_US                        = 0x02,
    RADIO_RAMP_80_US                        = 0x03,
    RADIO_RAMP_200_US                       = 0x04,
    RADIO_RAMP_800_US                       = 0x05,
    RADIO_RAMP_1700_US                      = 0x06,
    RADIO_RAMP_3400_US                      = 0x07,
}RadioRampTimes_t;


typedef enum
{
    IRQ_RADIO_NONE                          = 0x0000,
    IRQ_TX_DONE                             = 0x0001,
    IRQ_RX_DONE                             = 0x0002,
    IRQ_PREAMBLE_DETECTED                   = 0x0004,
    IRQ_SYNCWORD_VALID                      = 0x0008,
    IRQ_HEADER_VALID                        = 0x0010,
    IRQ_HEADER_ERROR                        = 0x0020,
    IRQ_CRC_ERROR                           = 0x0040,
    IRQ_CAD_DONE                            = 0x0080,
    IRQ_CAD_ACTIVITY_DETECTED               = 0x0100,
    IRQ_RX_TX_TIMEOUT                       = 0x0200,
    IRQ_RADIO_ALL                           = 0xFFFF,
}RadioIrqMasks_t;

/*!
 * \brief Structure describing the radio status
 */
typedef union RadioStatus_u
{
    uint8_t Value;
    struct
    {   //bit order is lsb -> msb
        uint8_t Reserved  : 1;  //!< Reserved
        uint8_t CmdStatus : 3;  //!< Command status
        uint8_t ChipMode  : 3;  //!< Chip mode
        uint8_t CpuBusy   : 1;  //!< Flag for CPU radio busy
    }Fields;
}RadioStatus_t;


/*!
 * \brief Represents a calibration configuration
 */
typedef union
{
    struct
    {
        uint8_t RC64KEnable    : 1;                             //!< Calibrate RC64K clock
        uint8_t RC13MEnable    : 1;                             //!< Calibrate RC13M clock
        uint8_t PLLEnable      : 1;                             //!< Calibrate PLL
        uint8_t ADCPulseEnable : 1;                             //!< Calibrate ADC Pulse
        uint8_t ADCBulkNEnable : 1;                             //!< Calibrate ADC bulkN
        uint8_t ADCBulkPEnable : 1;                             //!< Calibrate ADC bulkP
        uint8_t ImgEnable      : 1;
        uint8_t                : 1;
    }Fields;
    uint8_t Value;
}CalibrationParams_t;

typedef enum
{
    STDBY_RC                                = 0x00,
    STDBY_XOSC                              = 0x01,
}RadioStandbyModes_t;

/*!
 * \brief Represents the volatge used to control the TCXO on/off from DIO3
 */
typedef enum
{
    TCXO_CTRL_1_6V                          = 0x00,
    TCXO_CTRL_1_7V                          = 0x01,
    TCXO_CTRL_1_8V                          = 0x02,
    TCXO_CTRL_2_2V                          = 0x03,
    TCXO_CTRL_2_4V                          = 0x04,
    TCXO_CTRL_2_7V                          = 0x05,
    TCXO_CTRL_3_0V                          = 0x06,
    TCXO_CTRL_3_3V                          = 0x07,
}RadioTcxoCtrlVoltage_t;

/*!
 * \brief Represents a sleep mode configuration
 */
typedef union
{
    struct
    {
        uint8_t WakeUpRTC               : 1;                    //!< Get out of sleep mode if wakeup signal received from RTC
        uint8_t Reset                   : 1;
        uint8_t WarmStart               : 1;
        uint8_t Reserved                : 5;
    }Fields;
    uint8_t Value;
}SleepParams_t;




extern S_FSKConfig G_FSKConfig;
extern S_FSKPara G_FSKPara;

void SX126xSetStopRxTimerOnPreambleDetect( bool enable );
bool SX126X_FSKConfig_Check(void) ;
void SX126xWakeup( void );
void SX126xSetStandby( RadioStandbyModes_t standbyConfig );
RadioOperatingModes_t SX126xGetOperatingMode( void );
void SX126xCheckDeviceReady( void );
void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size );
void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size );
void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
void SX126xWriteRegister( uint16_t address, uint8_t value );
void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
uint8_t SX126xReadRegister( uint16_t address );
void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );
void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );
void SX126xWriteFifo( uint8_t *buffer, uint8_t size );
void SX126xReadFifo( uint8_t *buffer, uint8_t size );
void SX126X_Reset(void);
void SX126X_InitIo(void);
void SX126XWriteRxTx( bool txEnable );
void SX126XSwitchOff(void);
void DIO1_EnableInterrupt(void);
void DIO1_DisableInterrupt(void);
GPIO_PinState DIO1_GetState(void);
void DIO2_EnableInterrupt(void);
void DIO2_DisableInterrupt(void);
GPIO_PinState DIO2_GetState(void);
void Busy_EnableInterrupt(void);
void Busy_DisableInterrupt(void);
GPIO_PinState Busy_GetState(void);
void SX126xSleep( void );
void SX126xSetSleep( SleepParams_t sleepConfig );
void SX126xCalibrateImage( uint32_t freq );
tSX126x_FSK_Error SX126xSetRfFrequency(uint32_t frequency );
void SX126xSetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
void SX126xSetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut );
void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime );
void SX126xSetPacketType( RadioPacketTypes_t packetType );
void SX126xSetTx( uint32_t timeout );
void SX126xSetRx( uint32_t timeout );
void SX126xSendPayload( uint8_t *payload, uint8_t size, uint32_t timeout );
void SX126X_FSKTxPacket( uint8_t*data );
void SX126X_FSKStartRx( void );
void SX126xGetPacketStatus( S_FSKPara *pktStatus );
void SX126xGetRxBufferStatus( uint8_t *payloadLength );
void SX126xGetPayload( uint8_t *buffer, uint8_t *size);
void SX126X_FSKRxPacket(uint8_t*cbuf);
void SX126xClearIrqStatus( uint16_t irq );
uint16_t SX126xGetIrqStatus( void );
void SX126xClearDeviceErrors( void );
void SX126xClearTimeoutEvent( void );
void SX126xSetDio2AsRfSwitchCtrl( uint8_t enable );
void SX126xSetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout );
RadioStatus_t SX126xGetStatus( void );
void SX126xSetRegulatorMode( RadioRegulatorMode_t mode );
void SX126xSetRfTxPower( int8_t power );
void SX126xSetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress );
void SX126xCalibrate( CalibrationParams_t calibParam );
void SX126xCalibrate_err(void);
	
static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth );
void SX126xSetCrcSeed( uint16_t seed );
void SX126xSetCrcPolynomial( uint16_t polynomial );
uint8_t SX126xSetSyncWord( uint8_t *syncWord );
void SX126xSetWhiteningSeed( uint16_t seed );
void SX126xSetFSKModulationParams(void);
void SX126xSetFSKPacketParams(void);
tSX126x_FSK_Error SX126x_FSK_init(void);



#endif
