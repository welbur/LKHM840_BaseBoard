// #include "Arduino.h"

// #if not(defined(MBED_H) || defined(__SAM3X8E__) || defined(DISABLE_SPI_SERIALTRANSFER)) // These boards are/will not be supported by SPITransfer.h

#include "SPITransfer.h"
#include "SPITransfer_C.h"

/*!
 *    @brief  Create an SPI device with the given CS pin and settings
 *    @param  cspin The arduino pin number to use for chip select
 *    @param  freq The SPI clock frequency to use, defaults to 1MHz
 *    @param  dataOrder The SPI data order to use for bits within each byte,
 * defaults to SPI_BITORDER_MSBFIRST
 *    @param  dataMode The SPI mode to use, defaults to SPI_MODE0
 *    @param  theSPI The SPI bus to use, defaults to &theSPI
 */
#if 1
SPITransfer::SPITransfer(SlaveBoardHandler_t *slavebH, SPI_HandleTypeDef *theSPI, bool master)
{
	_slavebH = slavebH;
	_spi = theSPI;
	_master = master;
	_cs = _slavebH->BoardID;
	begin(false);
	_begun = false;
}
#endif

/*
 void SPITransfer::begin(SPIClass &_port, configST configs, const uint8_t &_SS)
 Description:
 ------------
  * Advanced initializer for the SPITransfer Class
 Inputs:
 -------
  * const SPIClass &_port - SPI port to communicate over
  * const configST configs - Struct that holds config
  * const uint8_t &_SS - SPI buslave select pin used
  values for all possible initialization parameters
 Return:
 -------
  * void
*/
void SPITransfer::begin(const configST configs)
{
	packet.begin(configs);
	_begun = true;
}

/*
 void SPITransfer::begin(SPIClass &_port, const uint8_t &_SS, const bool _debug, Stream &_debugPort)
 Description:
 ------------
  * Simple initializer for the SPITransfer Class
 Inputs:
 -------
  * const Stream &_port - SPI port to communicate over
  * const uint8_t &_SS - SPI buslave select pin used
  * const bool _debug - Whether or not to print error messages
  * const Stream &_debugPort - Serial port to print error messages
 Return:
 -------
  * void
*/
void SPITransfer::begin(const bool _debug)
{
	// 初始化spi//_spi->begin();
	// 设置spi为主模式//_spi->setSlaveMode(_slave);
	packet.begin(_debug);
	_begun = true;
}

#if 0
/*!
 *    @brief  Manually begin a transaction (calls beginTransaction if hardware
 * SPI)
 */
void SPITransfer::beginTransaction(void) 
{
    //初始化spi//_spi->beginTransaction(*_spiSetting);
}

/*!
 *    @brief  Manually end a transaction (calls endTransaction if hardware SPI)
 */
void SPITransfer::endTransaction(void) 
{
    _spi->endTransaction();
}
#endif

/**
 * @brief  Master_writeACKto_Slave
 * @param  cs: 指向Slave板的cs引脚
 * @note   Master板 发送同步信号给Slave板.
 * @retval 发送成功 返回 1； 发送失败 返回 0
 */
bool SPITransfer::Master_writeSyncto_Slave(uint8_t txData)
{
	return (bool)MSP_SPI_write(_spi, &txData, 1);
}

/**
 * @brief  Master_SyncWith_Slave
 * @param  rxData: ack数据
 * @note   slave板 读取slave板发送过来的ack信号.
 * @retval 发送成功 返回 1； 发送失败 返回 0
 */
bool SPITransfer::Master_SyncWith_Slave(uint8_t rxData)
{
	uint8_t rxack;
	uint32_t msTickstart = xTaskGetTickCount();
	do
	{
		if (!MSP_SPI_read(_spi, &rxack, 1))
		{
		} 
		LOGI("rxack : %d\r\n", rxack);
		if (rxack == rxData)
		{
			return true;
		}
	} while ((xTaskGetTickCount() - msTickstart) < 10); //{0xAC, 0xA0, 0xB0, 0x30, 0xE1};
	//LOGI("timeout ...msTickstart : %ld\r\n", (HAL_GetTick() - msTickstart));
	return false;
}

uint8_t SPITransfer::Master_writeDATAto_Slave_withPacket(const uint16_t &messageLen, const uint8_t boardID)
{
	uint8_t numBytesIncl = packet.constructPacket(messageLen, boardID);
	MSP_SPI_write(_spi, packet.preamble, sizeof(packet.preamble));
	MSP_SPI_write(_spi, packet.txBuff, numBytesIncl);
	MSP_SPI_write(_spi, packet.postamble, sizeof(packet.postamble));
	return numBytesIncl;
}

void SPITransfer::Master_readDATAfrom_Slave_withPacket()
{
	uint32_t msTickstart = xTaskGetTickCount(); // HAL_GetTick();
	uint8_t recChar = 0xF0;
	bytesRead = 0;
	//  	uint8_t rxi = 0;    //
	//	uint8_t spiRxData[100];
	do
	{
		if (!MSP_SPI_read(_spi, &recChar, 1))
		{
			_slavebH->spiTransState = SpiTrans_Err;
			return;
		}
		//		if (rxi > 100) rxi = 0;
		//		spiRxData[rxi++] 		= recChar;   //
		bytesRead = packet.parse(recChar);
		status = packet.status;
		if (status != CONTINUE)
		{
			//LOGI("\r\nstatus : %d, bytesRead : %d\r\n", status, bytesRead);
			if (status < 0)
			{
				reset();
				_slavebH->spiTransState = SpiTrans_S2M_RxData_Err;
				return;
			} //{sTransState[cBoard] = SpiTrans_S2M_RxData_Err; reset();}
			_slavebH->spiTransState = SpiTrans_S2M_RxData_End;
#if 0 // 打印测试信息
  			LOGI("read spi data len : %d\r\n", rxi);
  			for (int i = 0; i < rxi; i++) {
    			LOGI("%d, ", spiRxData[i]);
  			}
  			LOGI("-------------%ld\r\n", xTaskGetTickCount());
#endif
			//_slavebH->spiRx_uartTx_u8regs_size = rxi;
			return; // sTransState[cBoard] = SpiTrans_S2M_RxData_End;
		}
	} while ((xTaskGetTickCount() - msTickstart) < sTrans_TimeOut); // while(recChar != 129); //0xAA);

#if 0 // 打印测试信息
  	LOGI("read spi data len : %d\r\n", rxi);
  	for (int i = 0; i < rxi; i++) {
    	LOGI("%d, ", spiRxData[i]);
  	}
  	LOGI("-------------%ld\r\n", xTaskGetTickCount());
  	LOGI("status : %d\r\n", status);
  	rxi = 0;
#endif
#if 1
	/* 3------判断是否为超时退出？或者是完成读取数据------------ */
	// if (sTransState[cBoard] == SpiTrans_S2M_RxData_End) break;
	if ((HAL_GetTick() - msTickstart) > sTrans_TimeOut)
	{
		LOGE("spi trans timeout\r\n");
	}
#endif
	_slavebH->spiTransState = SpiTrans_TimeOut;
	return;
}

/**
 * @brief  Master Synchronization with Slave.
 * @param  currentBoard_TypeDef 当前触发的板子号
 * @retval None                                            如果同步成功，返回true
 */
void SPITransfer::Master_Spi1_Transfer(uint8_t TxRxFlag, uint8_t boardID)
{
	// uint8_t rxbuf[100];
	// for (uint8_t i = 0 ; i < sizeof(rxbuf) ; i++) { rxbuf[i] = 0; }
	/*****如果没收到slave板发过来的ack信号， master板就重新发送一次ack信号给slave板，循环3次都失败的话，退出报错*****/

#if 1
	if (TxRxFlag == RxFlag)
	{
		//LOGI("start read data from slave \r\n");
		SPI1_CS_ENABLE(boardID);
#if 1
		/* 1------主控板发送 ACK signal 给 slave板 */
		if (!Master_writeSyncto_Slave(SPI_MASTERRead_ACK))
		{
			LOGE("tx ack error\r\n");
			return;
		}
#endif
		/* 2------从slave板接收数据------------ */
		// LOGI("start read data from slave \r\n");
		Master_readDATAfrom_Slave_withPacket(); // 1-----接收数据
		SPI1_CS_DISABLE(boardID);
		if (_slavebH->spiTransState != SpiTrans_S2M_RxData_End)
		{
			LOGE("read rxdata err\r\n");
			return;
		}

		for (int i = 0; i < (packet.bytesRead); i++)
		{
			_slavebH->spiRx_uartTx_u8regs[i] = packet.rxBuff[i];
			// LOGI("%02X ", packet.rxBuff[i]);
		}
		_slavebH->spiRx_uartTx_u8regs_size = packet.bytesRead;
		// LOGI(".......%d\r\n", packet.bytesRead);
	}
	else if (TxRxFlag == TxFlag)
	{
		SPI1_CS_ENABLE(boardID);
		//uint32_t txTickstart = xTaskGetTickCount(); 
		//uint8_t rxack;
		//LOGI("rxflag...........%d\r\n", HAL_GPIO_ReadPin(SPI1_DQB1_CS_Port, SPI1_DQB2_CS));
		if (!Master_SyncWith_Slave(SPI_SLAVE_ACK)) {
			LOGE("read ack from slave error.................................\r\n");
			return;
		}

		// SPI1_CS_DISABLE(boardID);
		/* 2------往slave板发送数据------------ */
		uint8_t sendSize = 0;
		packet.txObj(_slavebH->spiTx_uartRx_Buffer, sendSize);
		sendSize = _slavebH->spiTx_uartRx_Buffer_Size;
		// SPI1_CS_ENABLE(boardID);
		Master_writeDATAto_Slave_withPacket(sendSize, boardID);
		SPI1_CS_DISABLE(boardID);
	}
	else
	{
		LOGE("txrxflag error \r\n");
	}
#endif
	/* 3------接收完数据后，master板发送end信号给slave板------------ */
	//	if (!MSP_SPI_write(_spi, _cs, &SPI_Trans_END, 1)) { LOGI("write end to slave err\r\n"); _slavebH->spiTransState = SpiTrans_Err; return; }
	_slavebH->spiTransState = SpiTrans_End;

	return;
}

/*
 uint8_t SPITransfer::currentPacketID()
 Description:
 ------------
  * Returns the ID of the last parsed packet
 Inputs:
 -------
  * void
 Return:
 -------
  * uint8_t - ID of the last parsed packet
*/
uint8_t SPITransfer::currentPacketID()
{
	return packet.currentPacketID();
}

/*
 void SerialTransfer::reset()
 Description:
 ------------
  * Clears out the tx, and rx buffers, plus resets
  the "bytes read" variable, finite state machine, etc
 Inputs:
 -------
  * void
 Return:
 -------
  * void
*/
void SPITransfer::reset()
{
	uint8_t txdata = 0xFF;
	MSP_SPI_write(_spi, &txdata, 1); //_spi->transfer(0xFF);
	packet.reset();
	status = packet.status;
	LOGE("spi transfer reset \r\n");
}

/*
 * 用于c调用
 */
extern void *SPITransfer_C_New(SlaveBoardHandler_t *slavebH, SPI_HandleTypeDef *theSPI, uint8_t master)
{
	return new SPITransfer(slavebH, theSPI, (bool)master);
}
extern void SPITransfer_C_Master_Spi1_Transfer(void *SpiTrans, uint8_t TxRxFlag, BoardID_TypeDef boardID)
{
	SPITransfer *sTrans = (SPITransfer *)SpiTrans;
	sTrans->Master_Spi1_Transfer(TxRxFlag, (uint8_t)boardID);
	delete sTrans;
	return;
}

// sendSize = packet.txObj(test_TxBuff, sendSize);  		//1-------封装数据
// extern void SPITransfer_C_TxObj(void *SpiTrans, uint8_t TxRxFlag, BoardID_TypeDef boardID)
//{
//	uint8_t sendSize = 0;
//	sendSize = packet.txObj(test_TxBuff, sendSize);  		//1-------封装数据
//	return;
// }

// #endif // not (defined(MBED_H) || defined(__SAM3X8E__))
