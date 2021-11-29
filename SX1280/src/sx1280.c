
/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Driver for SX1280 devices

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
*/
//#include "stm32f0xx.h"
#include "main.h"
#include <string.h>
#include "sx1280.h"
#include "sx1280_hal.h"
//#include "delay.h"
#include <gpio.h>
/*!
 * \brief Radio registers definition
 *
 */
typedef struct
{
    uint16_t      Addr;                             //!< 寄存器地址
    uint8_t       Value;                            //!< 寄存器的值
}RadioRegisters_t;

/*!
 * \brief 无线电硬件寄存器初始化定义
 */
// { Address, RegValue }
#define RADIO_INIT_REGISTERS_VALUE  { NULL }

/*!
 * \brief 无线电硬件寄存器初始化
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t OperatingMode; //模式（TX RX FS ...）

/*!
 * \brief Stores the current packet type set in the radio
 */
static RadioPacketTypes_t PacketType;       //数据包类型（LORA GFSK ...）

/*!
 * \brief Stores the current LoRa bandwidth set in the radio
 */
static RadioLoRaBandwidths_t LoRaBandwidth; //LORA带宽

/*!
 * \brief Holds the polling state of the driver
 */
static bool PollingMode;                    //轮询

/*!
 * 硬件DIO IRQ回调初始化
 */
DioIrqHandler *DioIrq[] = { SX1280OnDioIrq };

/*
 函数功能：激活轮询模式（polling mode）后，将由应用程序调用ProcessIrqs( )--->根据IRQ判断芯片是TXDone RXDone txTimeout... 
           否则，驱动程序会在无线电中断时自动调用ProcessIrqs（）,在stm32f1xx_it.c文件的EXTI3_IRQHandler中断中（206行）
 */
void SX1280OnDioIrq( void );

/*!
 * \brief Holds a flag raised on radio interrupt
 */
static bool IrqState;

static RadioCallbacks_t* RadioCallbacks;//回调函数 是TXDone txTimeout rxError...

int32_t SX1280complement2( const uint32_t num, const uint8_t bitCnt )
{
    int32_t retVal = ( int32_t )num;
    if( num >= 2<<( bitCnt - 2 ) )
    {
        retVal -= 2<<( bitCnt - 1 );
    }
    return retVal;
}

/*
函数功能：初始化
*/
void SX1280Init( RadioCallbacks_t *callbacks )
{
    RadioCallbacks = callbacks;//回调函数结构体

    SX1280HalInit( DioIrq );//在这个函数中进行了复位
    
}

void SX1280SetRegistersDefault( void )
{
    for( int16_t i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1280HalWriteRegister( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }
}

/*
函数功能：获取芯片信息
*/
uint16_t SX1280GetFirmwareVersion( void )
{
    return( ( ( SX1280HalReadRegister( REG_LR_FIRMWARE_VERSION_MSB ) ) << 8 ) | ( SX1280HalReadRegister( REG_LR_FIRMWARE_VERSION_MSB + 1 ) ) );
}

/*
函数功能：得到芯片状态
*/
RadioStatus_t SX1280GetStatus( void )
{
    uint8_t stat = 0;
    RadioStatus_t status;//存放状态的结构体

    SX1280HalReadCommand( RADIO_GET_STATUS, ( uint8_t * )&stat, 1 );
    status.Value = stat;
    return status;
}

/*
函数功能：得到芯片模式
*/
RadioOperatingModes_t SX1280GetOpMode( void )
{
    return OperatingMode;
}

/*
函数功能：设置sleep 模式
*/
void SX1280SetSleep( SleepParams_t sleepConfig )
{
    uint8_t sleep = ( sleepConfig.WakeUpRTC << 3 ) |
                    ( sleepConfig.InstructionRamRetention << 2 ) |
                    ( sleepConfig.DataBufferRetention << 1 ) |
                    ( sleepConfig.DataRamRetention );

    OperatingMode = MODE_SLEEP;
    SX1280HalWriteCommand( RADIO_SET_SLEEP, &sleep, 1 );
}

/*
函数功能：设置Standby 模式
*/
void SX1280SetStandby( RadioStandbyModes_t standbyConfig )
{
    SX1280HalWriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
    if( standbyConfig == STDBY_RC )
    {
        OperatingMode = MODE_STDBY_RC;
    }
    else
    {
        OperatingMode = MODE_STDBY_XOSC;
    }
}

/*
函数功能：设置FS 模式
*/
void SX1280SetFs( void )
{
    SX1280HalWriteCommand( RADIO_SET_FS, 0, 0 );
    OperatingMode = MODE_FS;
}

/*
函数功能：设置TX 模式
超时时间=timeout.Step*timeout.NbSteps
timeout.Step：超时的基值          RADIO_TICK_SIZE_1000_US
timeout.NbSteps：超时的持续时间   1000
超时时间就为1S
*/
void SX1280SetTx( TickTime_t timeout )
{
    uint8_t buf[3];
    buf[0] = timeout.Step;
    buf[1] = ( uint8_t )( ( timeout.NbSteps >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( timeout.NbSteps & 0x00FF );

    SX1280ClearIrqStatus( IRQ_RADIO_ALL );

    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetTx
    if( SX1280GetPacketType( ) == PACKET_TYPE_RANGING )//测距
    {
        SX1280SetRangingRole( RADIO_RANGING_ROLE_MASTER );
    }
    //SX1280HalWriteCommand( RADIO_SET_TX, buf, 3 );
    SX1280HalWriteCommand( RADIO_SET_TX, 0, 0 );
    OperatingMode = MODE_TX;
}

/*
函数功能：设置RX 模式
超时时间=timeout.Step*timeout.NbSteps
timeout.Step：超时的基值          RADIO_TICK_SIZE_1000_US
timeout.NbSteps：超时的持续时间   1000
超时时间就为1S
*/
void SX1280SetRx( TickTime_t timeout )
{
    uint8_t buf[3];
    buf[0] = timeout.Step;
    buf[1] = ( uint8_t )( ( timeout.NbSteps >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( timeout.NbSteps & 0x00FF );

    SX1280ClearIrqStatus( IRQ_RADIO_ALL );
    
    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetRx
    if( SX1280GetPacketType( ) == PACKET_TYPE_RANGING )
    {
        SX1280SetRangingRole( RADIO_RANGING_ROLE_SLAVE );
    }
    //SX1280HalWriteCommand( RADIO_SET_RX, buf, 3 );
    SX1280HalWriteCommand( RADIO_SET_RX, 0, 0 );
 
    OperatingMode = MODE_RX;
}

/*
函数功能：收发器设置为嗅探模式，以便它定期查找新数据包（占空比操作）
睡眠时间=Step*RxNbStepSleep
持续时间=Step*NbStepRx
*/
void SX1280SetRxDutyCycle( RadioTickSizes_t Step, uint16_t NbStepRx, uint16_t RxNbStepSleep )
{
    uint8_t buf[5];

    buf[0] = Step;
    buf[1] = ( uint8_t )( ( NbStepRx >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( NbStepRx & 0x00FF );
    buf[3] = ( uint8_t )( ( RxNbStepSleep >> 8 ) & 0x00FF );
    buf[4] = ( uint8_t )( RxNbStepSleep & 0x00FF );
    SX1280HalWriteCommand( RADIO_SET_RXDUTYCYCLE, buf, 5 );
    OperatingMode = MODE_RX;
}

/*
函数功能：设置为CAD模式
*/
void SX1280SetCad( void )
{
    SX1280HalWriteCommand( RADIO_SET_CAD, 0, 0 );
    OperatingMode = MODE_CAD;
}

/*
SetTxContinuousWave（）是一个测试命令，用于以选定的频率和输出功率生成连续波（RF音调）。
在主机发送模式配置命令之前，设备将保持在Tx Continuous Wave模式下。
*/
void SX1280SetTxContinuousWave( void )
{
    SX1280HalWriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
}

/*
SetTxContinuousPreamble（）是一个测试命令，可生成GFSK，BLE或FLRC调制中的“ 0”和“ 1”以及LoRa
中的符号0的无穷序列。在主机发送模式配置命令之前，设备将保持在Tx Continuous Wave中
*/
void SX1280SetTxContinuousPreamble( void )
{
    SX1280HalWriteCommand( RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
}

/*
函数功能：设置数据包类型
*/
void SX1280SetPacketType( RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    PacketType = packetType;

    SX1280HalWriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
}

/*
函数功能：得到数据包类型
*/
RadioPacketTypes_t SX1280GetPacketType( void )
{
    return PacketType;
}

/*
函数功能：设置频点
*/
void SX1280SetRfFrequency( uint32_t frequency )
{
    uint8_t buf[3];
    #if 0
    uint32_t freq = 0;

    freq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
    buf[0] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( freq & 0xFF );
    #endif
    buf[0] = ( uint8_t )( ( frequency >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( frequency >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( frequency & 0xFF );
    SX1280HalWriteCommand( RADIO_SET_RFFREQUENCY, buf, 3 );
}

/*
函数功能：设置TX输出功率 和 Tx斜坡时间
*/
void SX1280SetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];

    //在SPI / UART上发送的功率值在[0..31]范围内，物理输出功率在[-18..13] dBm范围内
    buf[0] = power + 18;
    buf[1] = ( uint8_t )rampTime;
    SX1280HalWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

/*
函数功能：义在其上检测到“通道活动”（CAD）的符号数量
*/
void SX1280SetCadParams( RadioLoRaCadSymbols_t cadSymbolNum )
{
    SX1280HalWriteCommand( RADIO_SET_CADPARAMS, ( uint8_t* )&cadSymbolNum, 1 );
    OperatingMode = MODE_CAD;
}

/*
函数功能：设置基地址
*/
void SX1280SetBufferBaseAddresses( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX1280HalWriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}

/*
函数功能：配置无线电的调制参数
参数：传入数据包类型调制参数 （数据包类型(lora) 扩频因子 带宽 编码率）
*/
void SX1280SetModulationParams( ModulationParams_t *modulationParams )
{
    uint8_t buf[3];

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != modulationParams->PacketType )
    {
        SX1280SetPacketType( modulationParams->PacketType );
    }

    switch( modulationParams->PacketType )
    {
        case PACKET_TYPE_GFSK:
            buf[0] = modulationParams->Params.Gfsk.BitrateBandwidth;
            buf[1] = modulationParams->Params.Gfsk.ModulationIndex;
            buf[2] = modulationParams->Params.Gfsk.ModulationShaping;
            break;

        case PACKET_TYPE_LORA:
            /**************************自己加的****************************************/
            buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
            buf[1] = modulationParams->Params.LoRa.Bandwidth;
            buf[2] = modulationParams->Params.LoRa.CodingRate;
            LoRaBandwidth = modulationParams->Params.LoRa.Bandwidth;
            break;
            /************************************************************************/
        case PACKET_TYPE_RANGING:
            buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
            buf[1] = modulationParams->Params.LoRa.Bandwidth;
            buf[2] = modulationParams->Params.LoRa.CodingRate;
            LoRaBandwidth = modulationParams->Params.LoRa.Bandwidth;
            break;

        case PACKET_TYPE_FLRC:
            buf[0] = modulationParams->Params.Flrc.BitrateBandwidth;
            buf[1] = modulationParams->Params.Flrc.CodingRate;
            buf[2] = modulationParams->Params.Flrc.ModulationShaping;
            break;

        case PACKET_TYPE_BLE:
            buf[0] = modulationParams->Params.Ble.BitrateBandwidth;
            buf[1] = modulationParams->Params.Ble.ModulationIndex;
            buf[2] = modulationParams->Params.Ble.ModulationShaping;
            break;

        case PACKET_TYPE_NONE:
            buf[0] = NULL;
            buf[1] = NULL;
            buf[2] = NULL;
            break;
    }
    SX1280HalWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, 3 );
}

/*
函数功能：设置数据包参数（前导码 标头 有效负载大小 CRC IQ...）
*/
void SX1280SetPacketParams( PacketParams_t *packetParams )
{
    uint8_t buf[7];

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != packetParams->PacketType )
    {
        SX1280SetPacketType( packetParams->PacketType );
    }

    switch( packetParams->PacketType )
    {
        case PACKET_TYPE_GFSK:
            buf[0] = packetParams->Params.Gfsk.PreambleLength;
            buf[1] = packetParams->Params.Gfsk.SyncWordLength;
            buf[2] = packetParams->Params.Gfsk.SyncWordMatch;
            buf[3] = packetParams->Params.Gfsk.HeaderType;
            buf[4] = packetParams->Params.Gfsk.PayloadLength;
            buf[5] = packetParams->Params.Gfsk.CrcLength;
            buf[6] = packetParams->Params.Gfsk.Whitening;
            break;

        case PACKET_TYPE_LORA:
            buf[0] = packetParams->Params.LoRa.PreambleLength;
            buf[1] = packetParams->Params.LoRa.HeaderType;
            buf[2] = packetParams->Params.LoRa.PayloadLength;
            buf[3] = packetParams->Params.LoRa.CrcMode;
            buf[4] = packetParams->Params.LoRa.InvertIQ;
            buf[5] = NULL;
            buf[6] = NULL;
            break;
        case PACKET_TYPE_RANGING:
            buf[0] = packetParams->Params.LoRa.PreambleLength;
            buf[1] = packetParams->Params.LoRa.HeaderType;
            buf[2] = packetParams->Params.LoRa.PayloadLength;
            buf[3] = packetParams->Params.LoRa.CrcMode;
            buf[4] = packetParams->Params.LoRa.InvertIQ;
            buf[5] = NULL;
            buf[6] = NULL;
            break;

        case PACKET_TYPE_FLRC:
            buf[0] = packetParams->Params.Flrc.PreambleLength;
            buf[1] = packetParams->Params.Flrc.SyncWordLength;
            buf[2] = packetParams->Params.Flrc.SyncWordMatch;
            buf[3] = packetParams->Params.Flrc.HeaderType;
            buf[4] = packetParams->Params.Flrc.PayloadLength;
            buf[5] = packetParams->Params.Flrc.CrcLength;
            buf[6] = packetParams->Params.Flrc.Whitening;
            break;

        case PACKET_TYPE_BLE:
            buf[0] = packetParams->Params.Ble.ConnectionState;
            buf[1] = packetParams->Params.Ble.CrcField;
            buf[2] = packetParams->Params.Ble.BlePacketType;
            buf[3] = packetParams->Params.Ble.Whitening;
            buf[4] = NULL;
            buf[5] = NULL;
            buf[6] = NULL;
            break;

        case PACKET_TYPE_NONE:
            buf[0] = NULL;
            buf[1] = NULL;
            buf[2] = NULL;
            buf[3] = NULL;
            buf[4] = NULL;
            buf[5] = NULL;
            buf[6] = NULL;
            break;
    }
    SX1280HalWriteCommand( RADIO_SET_PACKETPARAMS, buf, 7 );
}

/*
函数功能：此命令返回最后接收的 数据包的长度 和 接收的第一个字节的地址。
          地址是相对于数据缓冲区的第一个字节的偏移量。
*/
void SX1280GetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBufferPointer )
{
    uint8_t status[2];

    SX1280HalReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

   //如果是LORA固定标头，则通过读取寄存器REG_LR_PAYLOADLENGTH 获得有效负载长度
   
    if( ( SX1280GetPacketType( ) == PACKET_TYPE_LORA ) && ( SX1280HalReadRegister( REG_LR_PACKETPARAMS ) >> 7 == 1 ) )
    {
        *payloadLength = SX1280HalReadRegister( REG_LR_PAYLOADLENGTH );
    }
    else if( SX1280GetPacketType( ) == PACKET_TYPE_BLE )
    {
        // In the case of BLE, the size returned in status[0] do not include the 2-byte length PDU header
        // so it is added there
        *payloadLength = status[0] + 2;
    }
    else
    {
        *payloadLength = status[0];
    }

    *rxStartBufferPointer = status[1];
}

/*
函数功能：得到数据包的信息
*/
void SX1280GetPacketStatus( PacketStatus_t *pktStatus )
{
    uint8_t status[5];

    SX1280HalReadCommand( RADIO_GET_PACKETSTATUS, status, 5 );

    pktStatus->packetType = SX1280GetPacketType( );
    switch( pktStatus->packetType )
    {
        case PACKET_TYPE_GFSK:
            pktStatus->Params.Gfsk.RssiAvg = -status[0] / 2;
            pktStatus->Params.Gfsk.RssiSync = -status[1] / 2;

            pktStatus->Params.Gfsk.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.Gfsk.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.Gfsk.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
            pktStatus->Params.Gfsk.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.Gfsk.SyncAddrStatus = status[4] & 0x07;
            break;

        case PACKET_TYPE_LORA:
            /**********************************自己加的代码*****************************************************/
            pktStatus->Params.LoRa.RssiPkt = -status[0] / 2;
            ( status[1] < 128 ) ? ( pktStatus->Params.LoRa.SnrPkt = status[1] / 4 ) : ( pktStatus->Params.LoRa.SnrPkt = ( ( status[1] - 256 ) /4 ) );

            pktStatus->Params.LoRa.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.LoRa.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
            pktStatus->Params.LoRa.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.LoRa.SyncAddrStatus = status[4] & 0x07;  
            break;
            /***************************************************************************************************/
        case PACKET_TYPE_RANGING:
            pktStatus->Params.LoRa.RssiPkt = -status[0] / 2;
            ( status[1] < 128 ) ? ( pktStatus->Params.LoRa.SnrPkt = status[1] / 4 ) : ( pktStatus->Params.LoRa.SnrPkt = ( ( status[1] - 256 ) /4 ) );

            pktStatus->Params.LoRa.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.LoRa.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.LoRa.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
            pktStatus->Params.LoRa.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.LoRa.SyncAddrStatus = status[4] & 0x07;
            break;

        case PACKET_TYPE_FLRC:
            pktStatus->Params.Flrc.RssiAvg = -status[0] / 2;
            pktStatus->Params.Flrc.RssiSync = -status[1] / 2;

            pktStatus->Params.Flrc.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.Flrc.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.Flrc.TxRxStatus.RxPid = ( status[3] >> 6 ) & 0x03;
            pktStatus->Params.Flrc.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
            pktStatus->Params.Flrc.TxRxStatus.RxPidErr = ( status[3] >> 4 ) & 0x01;
            pktStatus->Params.Flrc.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.Flrc.SyncAddrStatus = status[4] & 0x07;
            break;

        case PACKET_TYPE_BLE:
            pktStatus->Params.Ble.RssiAvg = -status[0] / 2;
            pktStatus->Params.Ble.RssiSync = -status[1] / 2;

            pktStatus->Params.Ble.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
            pktStatus->Params.Ble.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            pktStatus->Params.Ble.TxRxStatus.PacketSent = status[3] & 0x01;

            pktStatus->Params.Ble.SyncAddrStatus = status[4] & 0x07;
            break;

        case PACKET_TYPE_NONE:
            // In that specific case, we set everything in the pktStatus to zeros
            // and reset the packet type accordingly
            memset( pktStatus, 0, sizeof( PacketStatus_t ) );
            pktStatus->packetType = PACKET_TYPE_NONE;
            break;
    }
}

/*
函数功能：接收数据包期间返回瞬时RSSI值
*/
int8_t SX1280GetRssiInst( void )
{
    uint8_t raw = 0;

    SX1280HalReadCommand( RADIO_GET_RSSIINST, &raw, 1 );

    return ( int8_t )( -raw / 2 );
}

/*
函数功能：设置IRQ的引脚和启用IRQ
*/
void SX1280SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
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
    SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, buf, 8 );
}

/*
函数功能：得到IRQ状态
*/
uint16_t SX1280GetIrqStatus( void )
{
    uint8_t irqStatus[2];

    SX1280HalReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
    
    return ( irqStatus[0] << 8 ) | irqStatus[1];
}

/*
函数功能：清除IRQ状态
*/
void SX1280ClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    SX1280HalWriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}

void SX1280Calibrate( CalibrationParams_t calibParam )
{
    uint8_t cal = ( calibParam.ADCBulkPEnable << 5 ) |
                  ( calibParam.ADCBulkNEnable << 4 ) |
                  ( calibParam.ADCPulseEnable << 3 ) |
                  ( calibParam.PLLEnable << 2 ) |
                  ( calibParam.RC13MEnable << 1 ) |
                  ( calibParam.RC64KEnable );

    SX1280HalWriteCommand( RADIO_CALIBRATE, &cal, 1 );
}

/*
函数功能：设置LDO 还是DCDC
*/
void SX1280SetRegulatorMode( RadioRegulatorModes_t mode )
{
    SX1280HalWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}

/*
函数功能：转换到休眠模式后，收发器寄存器的内容将丢失。设备可以利用自动恢复的 SetSaveContext（）命令。
          将无线电寄存器值的当前上下文存储到协议引擎内的数据RAM中，以便在唤醒时进行恢复。
*/
void SX1280SetSaveContext( void )
{
    SX1280HalWriteCommand( RADIO_SET_SAVECONTEXT, 0, 0 );
}

/*
函数功能：
BLE 要求收发器能够在数据包接收后125μs发回响应。这是通过发送命令SetAutoTx（）来执行的，
该命令允许收发器在数据包接收结束后的用户可编程时间（时间）发送数据包。
必须以STDBY_RC模式发出SetAutoTx（）。
*/
void SX1280SetAutoTx( uint16_t time )
{
    uint16_t compensatedTime = time - ( uint16_t )AUTO_RX_TX_OFFSET;
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( compensatedTime >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( compensatedTime & 0x00FF );
    SX1280HalWriteCommand( RADIO_SET_AUTOTX, buf, 2 );
}

/*
函数功能：
此功能修改芯片行为，使Rx或Tx操作后的状态为FS而不是STDBY（参见第57页第10.7节“收发器电路模式图形说明” ）。
此功能用于减少连续Rx和/或Tx操作之间的切换时间（请参见表10-2：所有可能转换的切换时间（TswMode））。
*/
void SX1280SetAutoFS( uint8_t enable )
{
    SX1280HalWriteCommand( RADIO_SET_AUTOFS, &enable, 1 );
}

/*
函数功能：
命令（操作码0x98）将收发器设置为长前导码模式，并且只能与LoRa模式和GFSK模式一起使用。
在此模式下，命令SetTx，SetRx和SetRxDutyCycle的行为 被修改为：
*/
void SX1280SetLongPreamble( uint8_t enable )
{
    SX1280HalWriteCommand( RADIO_SET_LONGPREAMBLE, &enable, 1 );
}

/*
函数功能：设置TX 发送的数据、数据大小 
          在SX1280SendPayload（）函数中调用
*/
void SX1280SetPayload( uint8_t *buffer, uint8_t size )
{
    SX1280HalWriteBuffer( 0x00, buffer, size );
}

/*
函数功能：得到收到数据、数据大小 
*/
uint8_t SX1280GetPayload( uint8_t *buffer, uint8_t *size , uint8_t maxSize )
{
    static uint8_t offset=0;

    SX1280GetRxBufferStatus( size, &offset );//得到数据大小 偏移量
    if( *size > maxSize )
    {
        return 1;
    }
    SX1280HalReadBuffer( offset, buffer, *size );//根据数据大小，偏移量，得到数据
    return 0;
}


/*
函数现象：11110 111100 1111000 11110000 111100000 ......
参数：txcnt2_max：不发送TX命令的最大次数
返回：1对应发送TX命令  0对应不发送TX命令
*/
uint8_t tx_enablefunc(uint8_t txcnt2_max)
{
    static uint8_t txcnt1=5,txcnt2=1;//txcnt1：发送TX命令的次数  txcnt2：不发送TX命令的次数
    static uint8_t i=0,flag_enable=0;//0对应发送TX命令  1对应不发送TX命令 
     i++;                            //计数值
    if(flag_enable==0)//发送TX命令
    {
        if(i==txcnt1)//当i=4的时候，将i清零，切换flag_enable使能值=1，不发送TX命令
        {
            flag_enable=1;i=0;txcnt2++;//当i=4时，txcnt2++，用于不发送TX命令的次数
            if(txcnt2>=txcnt2_max){txcnt2=1;}
        }
        return 1;
    }
    else//不发送TX命令
    {
        if(txcnt2==i)//当不发送TX命令的次数等于i的时候，设置为发送TX命令，清空i
        {
            flag_enable=0;i=0;
        }
        return 0;
    }   
}   

/*
函数功能：设置TX 发送的数据、数据大小、延时时间 
*/
void SX1280SendPayload( uint8_t *payload, uint8_t size, TickTime_t timeout )
{
    SX1280SetPayload( payload, size );
    HAL_GPIO_WritePin(SX1280_TX_EN_GPIO_Port, SX1280_TX_EN_Pin, GPIO_PIN_SET);//发射控制 高电平有效
#if 0
    if(tx_enablefunc())
    {
        SX1280SetTx( timeout );
    }
    else/*由于将中断中将频点下标移到任务中执行，故这里不需要将频点下标++*/
    {
        /*若在这里进行了++，在中断中++了，会造成发送TX命令的数据帧和下一个不发送TX命令的数据帧频点错误*/
        //SX1280_Freq_i=(1+SX1280_Freq_i)%6;
    }
#endif
    SX1280SetTx( timeout );
#if 0
    /*通过按键来控制不发送TX命令的次数*/
    if(TX_flag==1)//按键往右
    {
        TX_cnt1++;        
        if(TX_cnt1>=60)
        {
            SX1280SetTx( timeout );
            TX_cnt1=61;
        }
        else/*由于将中断中将频点下标移到任务中执行，故这里不需要将频点下标++*/
        {
            //SX1280_Freq_i=(1+SX1280_Freq_i)%6;
            //Radio.SetRfFrequency( (1000000*SX1280_Freq[SX1280_Freq_band][SX1280_Freq_i]) );  //频点设置
        }            
    }
    else if(TX_flag==2)
    {
       TX_cnt2++;
        if(TX_cnt2>=179)
        {
            SX1280SetTx( timeout );
            TX_cnt2=180;
        } 
        else
        {
            //SX1280_Freq_i=(1+SX1280_Freq_i)%6;
            //Radio.SetRfFrequency( (1000000*SX1280_Freq[SX1280_Freq_band][SX1280_Freq_i]) );  //频点设置
        }            
    }
    else
    {
         TX_cnt1=0;TX_cnt2=0;
        SX1280SetTx( timeout );
    }
#endif
    
}

uint8_t SX1280SetSyncWord( uint8_t syncWordIdx, uint8_t *syncWord )
{
    uint16_t addr;
    uint8_t syncwordSize = 0;

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            syncwordSize = 5;
            switch( syncWordIdx )
            {
                case 1:
                    addr = REG_LR_SYNCWORDBASEADDRESS1;
                    break;

                case 2:
                    addr = REG_LR_SYNCWORDBASEADDRESS2;
                    break;

                case 3:
                    addr = REG_LR_SYNCWORDBASEADDRESS3;
                    break;

                default:
                    return 1;
            }
            break;

        case PACKET_TYPE_FLRC:
            //对于FLRC数据包类型，SyncWord短1个字节，并且基地址移位1个字节
            syncwordSize = 4;
            switch( syncWordIdx )
            {
                case 1:
                    addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;

                case 2:
                    addr = REG_LR_SYNCWORDBASEADDRESS2 + 1;
                    break;

                case 3:
                    addr = REG_LR_SYNCWORDBASEADDRESS3 + 1;
                    break;

                default:
                    return 1;
            }
            break;

        case PACKET_TYPE_BLE:
            // For Ble packet type, only the first SyncWord is used and its
            // address is shifted by one byte
            syncwordSize = 4;
            switch( syncWordIdx )
            {
                case 1:
                    addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;

                default:
                    return 1;
            }
            break;

        default:
            return 1;
    }
    SX1280HalWriteRegisters( addr, syncWord, syncwordSize );
    return 0;
}

void SX1280SetSyncWordErrorTolerance( uint8_t ErrorBits )
{
    ErrorBits = ( SX1280HalReadRegister( REG_LR_SYNCWORDTOLERANCE ) & 0xF0 ) | ( ErrorBits & 0x0F );
    SX1280HalWriteRegister( REG_LR_SYNCWORDTOLERANCE, ErrorBits );
}

void SX1280SetCrcSeed( uint16_t seed )
{
    uint8_t val[2];

    val[0] = ( uint8_t )( seed >> 8 ) & 0xFF;
    val[1] = ( uint8_t )( seed  & 0xFF );

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
        case PACKET_TYPE_FLRC:
            SX1280HalWriteRegisters( REG_LR_CRCSEEDBASEADDR, val, 2 );
            break;

        default:
            break;
    }
}

void SX1280SetBleAccessAddress( uint32_t accessAddress )
{
    SX1280HalWriteRegister( REG_LR_BLE_ACCESS_ADDRESS, ( accessAddress >> 24 ) & 0x000000FF );
    SX1280HalWriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 1, ( accessAddress >> 16 ) & 0x000000FF );
    SX1280HalWriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 2, ( accessAddress >> 8 ) & 0x000000FF );
    SX1280HalWriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 3, accessAddress & 0x000000FF );
}

void SX1280SetBleAdvertizerAccessAddress( void )
{
    SX1280SetBleAccessAddress( BLE_ADVERTIZER_ACCESS_ADDRESS );
}

void SX1280SetCrcPolynomial( uint16_t polynomial )
{
    uint8_t val[2];

    val[0] = ( uint8_t )( polynomial >> 8 ) & 0xFF;
    val[1] = ( uint8_t )( polynomial  & 0xFF );

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
        case PACKET_TYPE_FLRC:
            SX1280HalWriteRegisters( REG_LR_CRCPOLYBASEADDR, val, 2 );
            break;

        default:
            break;
    }
}

void SX1280SetWhiteningSeed( uint8_t seed )
{
    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
        case PACKET_TYPE_FLRC:
        case PACKET_TYPE_BLE:
            SX1280HalWriteRegister( REG_LR_WHITSEEDBASEADDR, seed );
            break;

        default:
            break;
    }
}

void SX1280SetRangingIdLength( RadioRangingIdCheckLengths_t length )
{
    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280HalWriteRegister( REG_LR_RANGINGIDCHECKLENGTH, ( ( ( ( uint8_t )length ) & 0x03 ) << 6 ) | ( SX1280HalReadRegister( REG_LR_RANGINGIDCHECKLENGTH ) & 0x3F ) );
            break;

        default:
            break;
    }
}

/*
设置设备测距地址
*/
void SX1280SetDeviceRangingAddress( uint32_t address )
{
    uint8_t addrArray[] = { address >> 24, address >> 16, address >> 8, address };

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280HalWriteRegisters( REG_LR_DEVICERANGINGADDR, addrArray, 4 );
            break;

        default:
            break;
    }
}

/*
设置测距请求地址
*/
void SX1280SetRangingRequestAddress( uint32_t address )
{
    uint8_t addrArray[] = { address >> 24, address >> 16, address >> 8, address };

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280HalWriteRegisters( REG_LR_REQUESTRANGINGADDR, addrArray, 4 );
            break;

        default:
            break;
    }
}

double SX1280GetRangingResult( RadioRangingResultTypes_t resultType )
{
    uint32_t valLsb = 0;
    double val = 0.0;

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280SetStandby( STDBY_XOSC );
            SX1280HalWriteRegister( 0x97F, SX1280HalReadRegister( 0x97F ) | ( 1 << 1 ) ); // enable LORA modem clock
            SX1280HalWriteRegister( REG_LR_RANGINGRESULTCONFIG, ( SX1280HalReadRegister( REG_LR_RANGINGRESULTCONFIG ) & MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )resultType ) & 0x03 ) << 4 ) );
            valLsb = ( ( SX1280HalReadRegister( REG_LR_RANGINGRESULTBASEADDR ) << 16 ) | ( SX1280HalReadRegister( REG_LR_RANGINGRESULTBASEADDR + 1 ) << 8 ) | ( SX1280HalReadRegister( REG_LR_RANGINGRESULTBASEADDR + 2 ) ) );
            SX1280SetStandby( STDBY_RC );

            // Convertion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280
            switch( resultType )
            {
                case RANGING_RESULT_RAW:
                    // Convert the ranging LSB to distance in meter
                    val = ( double )SX1280complement2( valLsb, 24 ) / ( double )SX1280GetLoRaBandwidth( ) * 36621.09375;
                    break;

                case RANGING_RESULT_AVERAGED:
                case RANGING_RESULT_DEBIASED:
                case RANGING_RESULT_FILTERED:
                    val = ( double )valLsb * 20.0 / 100.0;
                    break;

                default:
                    val = 0.0;
            }
            break;

        default:
            break;
    }
    return val;
}

void SX1280SetRangingCalibration( uint16_t cal )
{
    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_RANGING:
            SX1280HalWriteRegister( REG_LR_RANGINGRERXTXDELAYCAL, ( uint8_t )( ( cal >> 8 ) & 0xFF ) );
            SX1280HalWriteRegister( REG_LR_RANGINGRERXTXDELAYCAL + 1, ( uint8_t )( ( cal ) & 0xFF ) );
            break;

        default:
            break;
    }
}

void SX1280RangingClearFilterResult( void )
{
    uint8_t regVal = SX1280HalReadRegister( REG_LR_RANGINGRESULTCLEARREG );

    // To clear result, set bit 5 to 1 then to 0
    SX1280HalWriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal | ( 1 << 5 ) );
    SX1280HalWriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal & ( ~( 1 << 5 ) ) );
}

void SX1280RangingSetFilterNumSamples( uint8_t num )
{
    // Silently set 8 as minimum value
    SX1280HalWriteRegister( REG_LR_RANGINGFILTERWINDOWSIZE, ( num < DEFAULT_RANGING_FILTER_SIZE ) ? DEFAULT_RANGING_FILTER_SIZE : num );
}

//int8_t SX1280ParseHexFileLine( char* line )
//{
//    uint16_t addr;
//    uint16_t n;
//    uint8_t code;
//    uint8_t bytes[256];

//    if( SX1280GetHexFileLineFields( line, bytes, &addr, &n, &code ) != 0 )
//    {
//        if( code == 0 )
//        {
//            SX1280HalWriteRegisters( addr, bytes, n );
//        }
//        if( code == 1 )
//        { // end of file
//            //return 2;
//        }
//        if( code == 2 )
//        { // begin of file
//            //return 3;
//        }
//    }
//    else
//    {
//        return 0;
//    }
//    return 1;
//}

void SX1280SetRangingRole( RadioRangingRoles_t role )
{
    uint8_t buf[1];

    buf[0] = role;
    SX1280HalWriteCommand( RADIO_SET_RANGING_ROLE, &buf[0], 1 );
}

//int8_t SX1280GetHexFileLineFields( char* line, uint8_t *bytes, uint16_t *addr, uint16_t *num, uint8_t *code )
//{
//    uint16_t sum, len, cksum;
//    char *ptr;
//
//    *num = 0;
//    if( line[0] != ':' )
//    {
//        return 0;
//    }
//    if( strlen( line ) < 11 )
//    {
//        return 0;
//    }
//    ptr = line + 1;
//    if( !sscanf( ptr, "%02hx", &len ) )
//    {
//        return 0;
//    }
//    ptr += 2;
//    if( strlen( line ) < ( 11 + ( len * 2 ) ) )
//    {
//        return 0;
//    }
//    if( !sscanf( ptr, "%04hx", addr ) )
//    {
//        return 0;
//    }
//    ptr += 4;
//    if( !sscanf( ptr, "%02hhx", code ) )
//    {
//        return 0;
//    }
//    ptr += 2;
//    sum = ( len & 255 ) + ( ( *addr >> 8 ) & 255 ) + ( *addr & 255 ) + ( ( *code >> 8 ) & 255 ) + ( *code & 255 );
//    while( *num != len )
//    {
//        if( !sscanf( ptr, "%02hhx", &bytes[*num] ) )
//        {
//            return 0;
//        }
//        ptr += 2;
//        sum += bytes[*num] & 255;
//        ( *num )++;
//        if( *num >= 256 )
//        {
//            return 0;
//        }
//    }
//    if( !sscanf( ptr, "%02hx", &cksum ) )
//    {
//        return 0;
//    }
//    if( ( ( sum & 255 ) + ( cksum & 255 ) ) & 255 )
//    {
//        return 0; // checksum error
//    }
//
//    return 1;
//}

/*
函数功能：得到频率误差
*/
double SX1280GetFrequencyError( )
{
    uint8_t efeRaw[3] = {0};
    uint32_t efe = 0;
    double efeHz = 0.0;

    switch( SX1280GetPacketType( ) )
    {
        case PACKET_TYPE_LORA:
        case PACKET_TYPE_RANGING:
            efeRaw[0] = SX1280HalReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
            efeRaw[1] = SX1280HalReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
            efeRaw[2] = SX1280HalReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
            efe = ( efeRaw[0]<<16 ) | ( efeRaw[1]<<8 ) | efeRaw[2];
            efe &= REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

            efeHz = 1.55 * ( double )SX1280complement2( efe, 20 ) / ( 1600.0 / ( double )SX1280GetLoRaBandwidth( ) * 1000.0 );
            break;

        case PACKET_TYPE_NONE:
        case PACKET_TYPE_BLE:
        case PACKET_TYPE_FLRC:
        case PACKET_TYPE_GFSK:
            break;
    }

    return efeHz;
}

/*
函数功能：设置轮询模式
*/
void SX1280SetPollingMode( void )
{
    PollingMode = true;
}

int32_t SX1280GetLoRaBandwidth( )
{
    int32_t bwValue = 0;

    switch( LoRaBandwidth )
    {
        case LORA_BW_0200:
            bwValue = 203125;
            break;

        case LORA_BW_0400:
            bwValue = 406250;
            break;

        case LORA_BW_0800:
            bwValue = 812500;
            break;

        case LORA_BW_1600:
            bwValue = 1625000;
            break;

        default:
            bwValue = 0;
    }
    return bwValue;
}

void SX1280SetInterruptMode( void )
{
    PollingMode = false;
}

void SX1280OnDioIrq( void )
{
    /*
     * 激活轮询模式（polling mode）后，将由应用程序调用    
     * ProcessIrqs( ). 否则，驱动程序会在无线电中断时自动调用ProcessIrqs（）,在stm32f1xx_it.c文件的EXTI3_IRQHandler中断中（206行）
     */
    if( PollingMode == true )
    {
        IrqState = true;
    }
    else
    {
        SX1280ProcessIrqs( );
    }
}

/*
函数功能：根据IRQ判断芯片是TXDone RXDone txTimeout.....
*/
void SX1280ProcessIrqs( void )
{
    RadioPacketTypes_t packetType = PACKET_TYPE_NONE;//数据包类型
    
    if( SX1280GetOpMode( ) == MODE_SLEEP )//得到芯片模式 sleep fs tx rx...
    {
        return; // DIO glitch on V2b :-)
    }
#if 0
    if( PollingMode == true )//轮询模式
    {
        if( IrqState == true )
        {
            __disable_irq( );
            IrqState = false;
            __enable_irq( );
        }
        else
        {
            return;
        }
    }
 #endif 
    packetType = SX1280GetPacketType( );//数据包类型
    uint16_t irqRegs = SX1280GetIrqStatus( );//读取IRQ寄存器
    SX1280ClearIrqStatus( IRQ_RADIO_ALL );//清除IRQ
   
    switch( packetType )
    {
        case PACKET_TYPE_GFSK:
        case PACKET_TYPE_FLRC:
        case PACKET_TYPE_BLE:
            switch( OperatingMode )
            {
                case MODE_RX:
                    if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
                    {
                        if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
                        {
                            if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxError != NULL ) )
                            {
                                RadioCallbacks->rxError( IRQ_CRC_ERROR_CODE );
                            }
                        }
                        else if( ( irqRegs & IRQ_SYNCWORD_ERROR ) == IRQ_SYNCWORD_ERROR )
                        {
                            if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxError != NULL ) )
                            {
                                RadioCallbacks->rxError( IRQ_SYNCWORD_ERROR_CODE );
                            }
                        }
                        else
                        {
                            if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxDone != NULL ) )
                            {
                                RadioCallbacks->rxDone( );
                            }
                        }
                    }
                    if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxSyncWordDone != NULL ) )
                        {
                            RadioCallbacks->rxSyncWordDone( );
                        }
                    }
                    if( ( irqRegs & IRQ_SYNCWORD_ERROR ) == IRQ_SYNCWORD_ERROR )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxError != NULL ) )
                        {
                            RadioCallbacks->rxError( IRQ_SYNCWORD_ERROR_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxTimeout != NULL ) )
                        {
                            RadioCallbacks->rxTimeout( );
                        }
                    }
                    break;
                case MODE_TX:
                    if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->txDone != NULL ) )
                        {
                            RadioCallbacks->txDone( );
                        }
                    }
                    if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->txTimeout != NULL ) )
                        {
                            RadioCallbacks->txTimeout( );
                        }
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        case PACKET_TYPE_LORA:
            switch( OperatingMode )
            {
                case MODE_RX:
                    if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )//RX_IRQ 0x0002
                    {
                        if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )//CRC错误  0x0040
                        {
                            if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxError != NULL ) )
                            {
                                RadioCallbacks->rxError( IRQ_CRC_ERROR_CODE );//RX CRC错误
                               #if (Slave_Debug_PA13==0)    //使能引脚 
                                HAL_GPIO_WritePin(SX1280_RX_EN_GPIO_Port, SX1280_RX_EN_Pin, GPIO_PIN_RESET);//接收控制关闭
                              #endif
                            }
                        }
                        else
                        {
                            if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxDone != NULL ) )
                            {
                                RadioCallbacks->rxDone( );//RX 完成
                               #if (Slave_Debug_PA13==0)    //使能引脚 
                                HAL_GPIO_WritePin(SX1280_RX_EN_GPIO_Port, SX1280_RX_EN_Pin, GPIO_PIN_RESET);//接收控制关闭
                              #endif
                            }
                        }
                    }
                    if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxHeaderDone != NULL ) )
                        {
                            RadioCallbacks->rxHeaderDone( );//主函数中将这个的回调函数置为NULL
                        }
                    }
                    if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxError != NULL ) )
                        {
                            RadioCallbacks->rxError( IRQ_HEADER_ERROR_CODE );//RX 头错误
                           #if (Slave_Debug_PA13==0)    //使能引脚 
                            HAL_GPIO_WritePin(SX1280_RX_EN_GPIO_Port, SX1280_RX_EN_Pin, GPIO_PIN_RESET);//接收控制关闭
                          #endif
                        }
                    }
                    if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxTimeout != NULL ) )
                        {
                            RadioCallbacks->rxTimeout( );//RX 超时
                        }
                    }
                    if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_DISCARDED ) == IRQ_RANGING_SLAVE_REQUEST_DISCARDED )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxError != NULL ) )
                        {
                            RadioCallbacks->rxError( IRQ_RANGING_ON_LORA_ERROR_CODE );//RX 测距错误
                        }
                    }
                    break;
                case MODE_TX:
                  
                    if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->txDone != NULL ) )
                        {
                            RadioCallbacks->txDone( );//TX 完成
                             HAL_GPIO_WritePin(SX1280_TX_EN_GPIO_Port, SX1280_TX_EN_Pin, GPIO_PIN_RESET);//发射控制 关闭
                           #if (Slave_Debug_PA13==0)    //使能引脚 
                            HAL_GPIO_WritePin(SX1280_RX_EN_GPIO_Port, SX1280_RX_EN_Pin, GPIO_PIN_SET);//接收控制开启
                          #endif
                        }
                    }
                    if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->txTimeout != NULL ) )
                        {
                            RadioCallbacks->txTimeout( );//TX 超时
                            HAL_GPIO_WritePin(SX1280_TX_EN_GPIO_Port, SX1280_TX_EN_Pin, GPIO_PIN_RESET);//发射控制 关闭
                           #if (Slave_Debug_PA13==0)    //使能引脚 
                            HAL_GPIO_WritePin(SX1280_RX_EN_GPIO_Port, SX1280_RX_EN_Pin, GPIO_PIN_SET);//接收控制开启
                          #endif
                        }
                    }
                    break;
                case MODE_CAD:
                    if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
                    {
                        if( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED )
                        {
                            if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->cadDone != NULL ) )
                            {
                                RadioCallbacks->cadDone( true );
                            }
                        }
                        else
                        {
                            if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->cadDone != NULL ) )
                            {
                                RadioCallbacks->cadDone( false );
                            }
                        }
                    }
                    else if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxTimeout != NULL ) )
                        {
                            RadioCallbacks->rxTimeout( );
                        }
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        case PACKET_TYPE_RANGING:
            switch( OperatingMode )
            {
                // MODE_RX indicates an IRQ on the Slave side
                case MODE_RX:
                    if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_DISCARDED ) == IRQ_RANGING_SLAVE_REQUEST_DISCARDED )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rangingDone != NULL ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_SLAVE_ERROR_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_VALID ) == IRQ_RANGING_SLAVE_REQUEST_VALID )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rangingDone != NULL ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_SLAVE_VALID_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RANGING_SLAVE_RESPONSE_DONE ) == IRQ_RANGING_SLAVE_RESPONSE_DONE )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rangingDone != NULL ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_SLAVE_VALID_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rangingDone != NULL ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_SLAVE_ERROR_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxHeaderDone != NULL ) )
                        {
                            RadioCallbacks->rxHeaderDone( );
                        }
                    }
                    if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rxError != NULL ) )
                        {
                            RadioCallbacks->rxError( IRQ_HEADER_ERROR_CODE );
                        }
                    }
                    break;
                // MODE_TX indicates an IRQ on the Master side
                case MODE_TX:
                    if( ( irqRegs & IRQ_RANGING_MASTER_RESULT_TIMEOUT ) == IRQ_RANGING_MASTER_RESULT_TIMEOUT )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rangingDone != NULL ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_MASTER_ERROR_CODE );
                        }
                    }
                    if( ( irqRegs & IRQ_RANGING_MASTER_RESULT_VALID ) == IRQ_RANGING_MASTER_RESULT_VALID )
                    {
                        if( ( RadioCallbacks != NULL ) && ( RadioCallbacks->rangingDone != NULL ) )
                        {
                            RadioCallbacks->rangingDone( IRQ_RANGING_MASTER_VALID_CODE );
                        }
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        default:
            // Unexpected IRQ: silently returns
            break;
    }
}
