/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
*/
//#include "stm32f0xx.h"
#include "main.h"
#include <string.h>  
//#include "delay.h"
#include "sx1280_hal.h"
#include "sx1280_radio.h"
#include "gpio.h"
#include "spi.h"
//#include "cmsis_os.h"
/*!
 * \brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE   255    //0xFFF

#define IRQ_HIGH_PRIORITY     0

/***********************SX1280  NSS  RESET  SCK引脚*******************************/

#define RADIO_NSS_L     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET)
#define RADIO_NSS_H     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET)

#define RADIO_nRESET_L  HAL_GPIO_WritePin(SX1280_RESET_GPIO_Port,SX1280_RESET_Pin,GPIO_PIN_RESET)
#define RADIO_nRESET_H  HAL_GPIO_WritePin(SX1280_RESET_GPIO_Port,SX1280_RESET_Pin,GPIO_PIN_SET)


#define SX1280_SPI_WR(pTxData, pRxData, Size)	HAL_SPI_MY_TransmitReceive_DMA(&hspi1, pTxData,pRxData,Size)


/***************************************************************************************/

/*!
 * Radio driver 驱动程序结构初始化
 */
const struct Radio_s Radio =
{
    SX1280Init,                 //初始化
    SX1280HalReset,             //复位
    SX1280GetStatus,            //得到芯片状态
    SX1280HalWriteCommand,      //写指令        参数：指令 数据  大小
    SX1280HalReadCommand,       //读指令        参数：指令 数据  大小
    SX1280HalWriteRegisters,    //写多个寄存器  参数：地址 数据 大小
    SX1280HalWriteRegister,     //写一个寄存器  参数：地址 数据
    SX1280HalReadRegisters,     //读多个寄存器  参数：地址 数据 大小
    SX1280HalReadRegister,      //读一个寄存器  参数：地址 数据
    SX1280HalWriteBuffer,       //写多个字节缓存区  参数：地址 数据 大小
    SX1280HalReadBuffer,        //读多个字节缓存区  参数：地址 数据 大小
    SX1280HalGetDioStatus,      //得到DIO状态       返回值：DIO电平<<1|BUSY电平
    SX1280GetFirmwareVersion,   //获取芯片信息
    SX1280SetRegulatorMode,     //设置LDO 还是DCDC
    SX1280SetStandby,           //设置Standby 模式
    SX1280SetPacketType,        //设置数据包类型
    SX1280SetModulationParams,  //配置无线电的调制参数（数据包类型(lora) 扩频因子 带宽 编码率）
    SX1280SetPacketParams,      //设置数据包参数（前导码 标头 有效负载大小 CRC IQ...）
    SX1280SetRfFrequency,       //设置频点
    SX1280SetBufferBaseAddresses,   //设置基地址
    SX1280SetTxParams,          //设置TX输出功率 和 Tx斜坡时间
    SX1280SetDioIrqParams,      //设置IRQ的引脚和启用IRQ
    SX1280SetSyncWord,          //设置同步字
    SX1280SetRx,                //设置RX 模式
    SX1280GetPayload,           //得到收到数据、数据大小 
    SX1280SendPayload,          //设置TX 发送的数据、数据大小、延时时间 
    SX1280SetRangingRole,
    SX1280SetPollingMode,       //设置轮询模式
    SX1280SetInterruptMode,
    SX1280SetRegistersDefault,
    SX1280GetOpMode,            //得到芯片模式
    SX1280SetSleep,             //设置sleep 模式
    SX1280SetFs,                //设置FS 模式
    SX1280SetTx,                //设置TX 模式
    SX1280SetRxDutyCycle,       //收发器设置为嗅探模式，以便它定期查找新数据包（占空比操作）
    SX1280SetCad,
    SX1280SetTxContinuousWave,
    SX1280SetTxContinuousPreamble,
    SX1280GetPacketType,        //得到数据包类型
    SX1280SetCadParams,
    SX1280GetRxBufferStatus,    //此命令返回最后接收的 数据包的长度 和 接收的第一个字节的地址。
    SX1280GetPacketStatus,      //得到数据包信息
    SX1280GetRssiInst,          //接收数据包期间返回瞬时RSSI值
    SX1280GetIrqStatus,         //得到IRQ状态
    SX1280ClearIrqStatus,       //清除IRQ状态
    SX1280Calibrate,
    SX1280SetSaveContext,
    SX1280SetAutoTx,
    SX1280SetAutoFS,
    SX1280SetLongPreamble,
    SX1280SetPayload,
    SX1280SetSyncWordErrorTolerance,
    SX1280SetCrcSeed,
    SX1280SetBleAccessAddress,
    SX1280SetBleAdvertizerAccessAddress,
    SX1280SetCrcPolynomial,
    SX1280SetWhiteningSeed,
    SX1280SetRangingIdLength,
    SX1280SetDeviceRangingAddress,
    SX1280SetRangingRequestAddress,
    SX1280GetRangingResult,
    SX1280SetRangingCalibration,
    SX1280RangingClearFilterResult,
    SX1280RangingSetFilterNumSamples,
    SX1280GetFrequencyError,
};

static uint8_t halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};


/*
函数功能：读写一个字节
*/
uint8_t SX1280_SimSendByte( uint8_t output )
{
   
    uint8_t input = 0;

    SX1280_SPI_WR(&output,&input,1);

    return input ;
}

/*
函数功能：写字节
参数：写入的数据  大小
*/
 
void SpiIn( uint8_t *txBuffer, uint16_t size )
{
   uint8_t rxbuff[64];
   SX1280_SPI_WR(txBuffer,rxbuff,size);
}

/*
函数功能：读字节
参数：写入的数据  读取的数据 大小
*/
void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size )
{
    SX1280_SPI_WR(txBuffer,rxBuffer,size); 
}

/*!
 * \brief 用于阻止执行等待无线电繁忙引脚上的低电平状态。基本上用于SPI通信
 */
void SX1280HalWaitOnBusy( void )
{	 
   while( HAL_GPIO_ReadPin(SX1280_BUSY_GPIO_Port,SX1280_BUSY_Pin ) == 1 ); 	
}

void SX1280HalInit( DioIrqHandler **irqHandlers )
{
    SX1280HalReset( );//复位
    SX1280HalIoIrqInit( irqHandlers );
}

void SX1280HalIoIrqInit( DioIrqHandler **irqHandlers )
{
    /*设置中断引脚IRQ、 优先级、并执行SX1280OnDioIrq函数，里面是根据读取IRQ 判断是什么中断，由于工程中在gpio.c中对中断引脚进行了配置，故屏蔽掉*/
    //GpioSetIrq( RADIO_DIOx_PORT, RADIO_DIOx_PIN, IRQ_HIGH_PRIORITY, irqHandlers[0] );
    SX1280OnDioIrq();
}

/*
函数功能：复位
*/
void SX1280HalReset( void )
{
    HAL_Delay(100);
    RADIO_nRESET_L;
    HAL_Delay(100);
    RADIO_nRESET_H;
    HAL_Delay(100);
    RADIO_nRESET_L;
    HAL_Delay(100);
    RADIO_nRESET_H;
    HAL_Delay(100);	  
}

/*
函数功能：清除指令RAM会将0x00s写入每个字节。
*/
void SX1280HalClearInstructionRam( void )
{
    // Clearing the instruction RAM is writing 0x00s on every bytes of the
    // instruction RAM
    uint16_t halSize = 3 + IRAM_SIZE;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( IRAM_START_ADDRESS >> 8 ) & 0x00FF;
    halTxBuffer[2] = IRAM_START_ADDRESS & 0x00FF;
    for( uint16_t index = 0; index < IRAM_SIZE; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }
    SX1280HalWaitOnBusy( );

    SpiIn( halTxBuffer, halSize );

    SX1280HalWaitOnBusy( );
}


void SX120HalWakeup( void )
{
    __disable_irq( );


    uint16_t halSize = 2;
    halTxBuffer[0] = RADIO_GET_STATUS;
    halTxBuffer[1] = 0x00;
    SpiIn( halTxBuffer, halSize );

    // Wait for chip to be ready.
    SX1280HalWaitOnBusy( );

    __enable_irq( );
}

/*
函数功能：写指令
参数：指令 数据  大小
*/
void SX1280HalWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize  = size + 1;
    SX1280HalWaitOnBusy( );


    halTxBuffer[0] = command;
    memcpy( halTxBuffer + 1, ( uint8_t * )buffer, size * sizeof( uint8_t ) );
    
    SpiIn( halTxBuffer, halSize );


    if( command != RADIO_SET_SLEEP )
    {
        SX1280HalWaitOnBusy( );
    }
}

/*
函数功能：读指令
参数：指令 数据  大小
*/
void SX1280HalReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = 2 + size;
    halTxBuffer[0] = command;
    halTxBuffer[1] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[2+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    SpiInOut( halTxBuffer, halRxBuffer, halSize );

    memcpy( buffer, halRxBuffer + 2, size );

    SX1280HalWaitOnBusy( );
}

/*
函数功能：写多个寄存器
参数：地址 数据 大小
*/
void SX1280HalWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    memcpy( halTxBuffer + 3, buffer, size );

    SX1280HalWaitOnBusy( );


    SpiIn( halTxBuffer, halSize );
   

    SX1280HalWaitOnBusy( );
}

/*
函数功能：写一个寄存器
参数：地址 数据
*/
void SX1280HalWriteRegister( uint16_t address, uint8_t value )
{
    SX1280HalWriteRegisters( address, &value, 1 );
}

/*
函数功能：读多个寄存器
参数：地址 数据 大小
*/
void SX1280HalReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = 4 + size;
    halTxBuffer[0] = RADIO_READ_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    halTxBuffer[3] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[4+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    SpiInOut( halTxBuffer, halRxBuffer, halSize );
    
    memcpy( buffer, halRxBuffer + 4, size );

    SX1280HalWaitOnBusy( );
}

/*
函数功能：读一个寄存器
参数：地址 数据
*/
uint8_t SX1280HalReadRegister( uint16_t address )
{
    uint8_t data;

    SX1280HalReadRegisters( address, &data, 1 );

    return data;
}

/*
函数功能：写多个字节缓存区
参数：地址 数据 大小
*/
void SX1280HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint16_t halSize = size + 2;
    halTxBuffer[0] = RADIO_WRITE_BUFFER;
    halTxBuffer[1] = ( offset ) >> 8;
    memcpy( halTxBuffer + 2, buffer, size );

    SX1280HalWaitOnBusy( );

    SpiIn( halTxBuffer, halSize );

    SX1280HalWaitOnBusy( );
}

/*
函数功能：读多个字节缓存区
参数：地址 数据 大小
*/
void SX1280HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_READ_BUFFER;
    halTxBuffer[1] = offset;
    halTxBuffer[2] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

    SX1280HalWaitOnBusy( );

    SpiInOut( halTxBuffer, halRxBuffer, halSize );
    
    memcpy( buffer, halRxBuffer + 3, size );

    SX1280HalWaitOnBusy( );
}

/*
函数功能：得到DIO状态
返回值：DIO电平<<1|BUSY电平
*/
uint8_t SX1280HalGetDioStatus( void )
{
	
    return (  HAL_GPIO_ReadPin(A7106_IRQ_GPIO_Port,A7106_IRQ_Pin) << 1 ) | (  HAL_GPIO_ReadPin(SX1280_BUSY_GPIO_Port,SX1280_BUSY_Pin) << 0 );
}

