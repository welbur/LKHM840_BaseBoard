#include "BackPanelTrans.h" //baseboard
#ifdef LKHM840BaseB
#define BackPanelTransPort hspi1
#else
#define BackPanelTransPort hspi2
#endif

/* 定义全局变量 */
uint8_t _printDebug = 1; // 默认打印信息

#ifdef LKHM840PowerB
uint8_t _sBoardID = 0; // 只作为从模式的时候才会用到
#endif
uint16_t _spiITTimeOut = 0; // SPI IT消息超时时间
uint16_t _SYNCTimeOut = 0; // 同步信号超时时间
uint16_t _waitCSTimeOut = 0;
uint16_t _readACKTimeOut = 0;
uint16_t _readDataTimeOut = 0;

BackPanelTransStatus_TypeDef BPTrans_Init(BackPanelTransHandler_t *bpTransH) //(SPITransHandler_t *stransH)
{
	_printDebug 		= bpTransH->printDebug;

#ifdef LKHM840PowerB
	_sBoardID 			= bpTransH->slaveBoardID;
#endif
	_spiITTimeOut 		= bpTransH->spiITTimeOut;
	_SYNCTimeOut 		= bpTransH->SYNCTimeOut;
	_waitCSTimeOut 		= bpTransH->waitCSTimeOut;
	_readACKTimeOut 	= bpTransH->readACKTimeOut;
	_readDataTimeOut 	= bpTransH->readDataTimeOut;
	if ((BackPanelTransPort.Instance == NULL) | \
		(_spiITTimeOut == 0) | \
		(_SYNCTimeOut == 0) | \
		(_waitCSTimeOut == 0) | \
		(_readACKTimeOut == 0) | \
		(_readDataTimeOut == 0))
	{
		if (_printDebug)
			LOG("BackPanelTrans_Init error!\r\n");
		return bpTrans_Init_Err;
	}

	if (_printDebug)
		LOG("BackPanelTrans_Init complete!\r\n");
	return bpTrans_OK;
}

/**
 * @brief  SPI IT模式时的 超时退出.
 * @param  None
 * @retval None
 */
BackPanelTransStatus_TypeDef waitSPISTATEready(void)
{
	uint32_t msTickstart = xTaskGetTickCount();
	while(HAL_SPI_GetState(&BackPanelTransPort) != HAL_SPI_STATE_READY) 
	{
		//LOG("spi state is ready? ..%ld\r\n", (xTaskGetTickCount() - msTickstart));
		if ((xTaskGetTickCount() - msTickstart) > _spiITTimeOut) return bpTrans_SPIIT_Err;
	}
	return bpTrans_OK;
}

#ifdef LKHM840BaseB
/**
 * @brief  Master Synchronization with Slave.
 * @param  None
 * @retval None
 */
BackPanelTransStatus_TypeDef Master_Synchro(void)
{
	uint32_t msTickstart = xTaskGetTickCount();
	uint8_t txackbytes = SPI_MASTER_SYNBYTE, rxackbytes = 0x00;
	do
	{
		/* Call SPI write function to send command to slave */
		if (HAL_SPI_TransmitReceive_IT(&BackPanelTransPort, (uint8_t *)&txackbytes, (uint8_t *)&rxackbytes, 1) != HAL_OK)
			return bpTrans_SYNC_Err;
		if (waitSPISTATEready() != bpTrans_OK)	return bpTrans_SPIIT_Err;
		if (xTaskGetTickCount() - msTickstart > _SYNCTimeOut) 
			return bpTrans_SYNC_TimeOut;
	} while (rxackbytes != SPI_SLAVE_SYNBYTE);
	return bpTrans_OK;
}
#else
/**
 * @brief  Slave synchronization with Master
 * @param  None
 * @retval None
 */
BackPanelTransStatus_TypeDef Slave_Synchro(void)
{
	uint32_t msTickstart = xTaskGetTickCount();
	uint8_t txackbyte = SPI_SLAVE_SYNBYTE, rxackbyte = 0x00;
	do
	{
		if (HAL_SPI_TransmitReceive(&BackPanelTransPort, (uint8_t *)&txackbyte, (uint8_t *)&rxackbyte, 1, 10) != HAL_OK)
			return bpTrans_SYNC_Err;
		if (xTaskGetTickCount() - msTickstart > _SYNCTimeOut) 
			return bpTrans_SYNC_TimeOut;
	} while (rxackbyte != SPI_MASTER_SYNBYTE);
	return bpTrans_OK;
}
#endif

/**
 * @brief  BackPanel_WriteACK
 * @param  ACKValue: ack信号的数值
 * @note   Master板 发送同步信号给Slave板.  SPI_MASTER_ACK = 0xAC,
 * @note   Slave板 发送同步信号给Master板.	SPI_SLAVE_ACK  = 0x53,
 * @retval 发送成功 返回 1； 发送失败 返回 0
 */
BackPanelTransStatus_TypeDef BackPanel_WriteACK(BackPanelTrans_ACK_TypeDef ACKValue)
{
	return MSP_SPI_write(&BackPanelTransPort, &(ACKValue), 1);
}

/**
 * @brief  BackPanel_ReadACK
 * @param  ACKValue: 读取到的ack数值 与 ACKValue 比较，判断是否一致
 * @note   Master板 读取Slave板发送过来的ack信号.
 * @note   Slave板 读取Master板发送过来的ack信号.
 * @retval 读取到正确的ack值 返回 1； 错误 返回 0
 */
BackPanelTransStatus_TypeDef BackPanel_ReadACK(BackPanelTrans_ACK_TypeDef ACKValue)
{
	uint8_t rxack = 0;
	uint32_t msTickstart = xTaskGetTickCount();
	do
	{
		if (!MSP_SPI_read(&BackPanelTransPort, &rxack, 1))
		{
		}
		LOG("rxack : %02X\r\n", rxack);

		if (rxack == ACKValue)
		{
			return bpTrans_OK;
		}
	} while ((xTaskGetTickCount() - msTickstart) < _readACKTimeOut); //{0xAC, 0xA0, 0xB0, 0x30, 0xE1};
	if (_printDebug)
		Addto_osPrintf("timeout ...msTickstart : %ld\r\n", (xTaskGetTickCount() - msTickstart));
	return bpTrans_SPI_Err;
}

/**
 * @brief  BackPanel_WriteDATA_withPacket
 * @param  numBytesIncl: 需要发送的数据的长度
 * @note   封装后，再发送数据.
 * @retval 发送成功 返回 1； 错误 返回 0
 */
BackPanelTransStatus_TypeDef BackPanel_WriteDATA_withPacket(const uint8_t numBytesIncl)
{
	// numBytesIncl = PacketConstructPacket(messageLen, boardID);
	// LOG("numBytesIncl : %d\r\n", numBytesIncl);
	// for (int i =0; i < sizeof(pktHandle.preamble); i++) LOG(" %d ", pktHandle.preamble[i]);
	for (int i = 0; i < numBytesIncl; i++)
		Addto_osPrintf(" %d ", pktHandle.PtxBuff[i]);
	// for (int i=0; i<sizeof(pktHandle.postamble);i++) 	LOG(" %d ", pktHandle.postamble[i]);
	Addto_osPrintf("\r\n");
	MSP_SPI_write(&BackPanelTransPort, pktHandle.preamble, sizeof(pktHandle.preamble));
	MSP_SPI_write(&BackPanelTransPort, pktHandle.PtxBuff, numBytesIncl);
	MSP_SPI_write(&BackPanelTransPort, pktHandle.postamble, sizeof(pktHandle.postamble));
	return bpTrans_OK;
}

/**
 * @brief  BackPanel_readDATA_withPacket
 * @param  none
 * @retval 发送成功 返回 1； 错误 返回 0
 */
BackPanelTransStatus_TypeDef BackPanel_readDATA_withPacket()
{
	uint32_t msTickstart = xTaskGetTickCount(); // HAL_GetTick();
	uint8_t recChar = 0xF0;
	do
	{
		//if (waitSPISTATEready() != bpTrans_OK)	return bpTrans_SPIIT_Err;		//while(HAL_SPI_GetState(&BackPanelTransPort) != HAL_SPI_STATE_READY) {}
		if (HAL_SPI_Receive_IT(&BackPanelTransPort, &recChar, 1) != HAL_OK)
		{
			if (_printDebug) LOG("BackPanel_read one DATA error\r\n");
			return bpTrans_SPI_Err;
		}
		if (waitSPISTATEready() != bpTrans_OK)	return bpTrans_SPIIT_Err;		//while(HAL_SPI_GetState(&BackPanelTransPort) != HAL_SPI_STATE_READY) {}
		if (_printDebug) LOG("%d, ", recChar);
		PacketParse(&recChar, true); // bytesRead = PacketParse(recChar, true);
		// status = packet.status;
		// LOG("time : %ld, status : %d\r\n", (xTaskGetTickCount() - msTickstart), pktHandle.status);
		if (pktHandle.status != CONTINUE)
		{
			if (_printDebug) LOG("\r\nstatus : %d, bytesRead : %d\r\n", pktHandle.status, pktHandle.bytesRead);
			if (pktHandle.status < 0)
			{
				BackPanel_reset();
				if (_printDebug) LOG("BackPanel_readData error\r\n");
				return bpTrans_Data_Err;
			}
			return bpTrans_OK;
		}
	} while ((xTaskGetTickCount() - msTickstart) < _readDataTimeOut); // sTrans_TimeOut); // while(recChar != 129); //0xAA);

	if (_printDebug) LOG("BackPanel_readData timeout\r\n");

	return bpTrans_Data_TimeOut;
}

#ifdef LKHM840PowerB
/*****************************************************slave to master start********************************************************************************************/
#if 0
/* 等待spi2的 cs信号的值 */
BackPanelTransStatus_TypeDef waitSPI2CSequal(uint8_t cs_v) 
{ 
	uint32_t txTickstart = xTaskGetTickCount();
#if (cs_v == 0)
	while (MSP_SPI2_CS_STATUS())
#else
	while (!MSP_SPI2_CS_STATUS())
#endif
	{
		if (xTaskGetTickCount() - txTickstart > 10)
		{
			//LOG("wait CS = %d timeout.....\r\n", cs_v);
			return bpTrans_WaitCS_Err;
		} 
	}
	return bpTrans_Continue;
}
#endif

/**
 * @brief  BackPanelTrans_Slave_writeDataTo_Master .
 * @note   使用spi2
 * @param  writeData 需要发送的数据
 * @param  writeDataLen 数据长度
 * @retval BackPanelTransStatus_TypeDef
 */
BackPanelTransStatus_TypeDef BackPanelTrans_Slave_writeDataTo_Master(uint8_t *writeData, uint16_t writeDataLen)
{
	BackPanelTransStatus_TypeDef bpstatus;
	uint32_t txTickstart = xTaskGetTickCount();
	uint8_t test_TxBuff[] = {11, 22, 33, 104, 101, 44, 55, 66, 77, 88, 99, 132, 126, 0, 255, 12};
// LOG("writedata : ");
// for (int i=0; i<writeDataLen;i++) LOG("%d ", writeData[i]);
// LOG("\r\n");
/*****读取master板发过来的ack信号，失败的话，退出报错*****/
// LOGI("start  BackPanelTrans_Slave_writeDataTo_Master %d\r\n", MSP_SPI2_CS_STATUS());
/* 1------主控板发送过来的cs信号 */
#if 1
	while (MSP_SPI2_CS_STATUS())
	{
		if (xTaskGetTickCount() - txTickstart > _waitCSTimeOut)
		{
			if (_printDebug)
				Addto_osPrintf("wait CS = 0 timeout.....\r\n");
			return bpTrans_WaitCS_Err;
		}
	}
#endif
	/* 2------主控板发送 ACK signal 给 slave板 */
	bpstatus = BackPanel_ReadACK(SPI_MASTER_ACK);
	if (!bpstatus)
	{
		if (_printDebug)
			Addto_osPrintf("read ack from master error.................................\r\n");
		return bpstatus;
	}
	/* 3------往master板发送数据------------ */
	uint16_t sendSize = 0;
	// PacketTxObj(test_TxBuff, sendSize, sizeof(test_TxBuff));	//1-------封装数据
	PacketTxObj(writeData, sendSize, writeDataLen); // sendSize = packet.txObj(test_TxBuff, sendSize);
	uint8_t numBytesIncl = PacketConstructPacket(writeDataLen, _sBoardID);
	BackPanel_WriteDATA_withPacket(numBytesIncl); // 2-------发送数据
#if 0
	while (!MSP_SPI2_CS_STATUS()) 
	{
		if (xTaskGetTickCount() - txTickstart > _waitCSTimeOut)
		{
			if (_printDebug) Addto_osPrintf("wait CS = 1 timeout.....\r\n");
			return bpTrans_WaitCS_Err;
		} 
	}
#endif
	if (_printDebug)
		LOG("end BackPanelTrans_Slave_writeDataTo_Master ...%ld\r\n", xTaskGetTickCount());

	return bpTrans_OK;
}

/**
 * @brief  BackPanelTrans_Slave_readDataFrom_Master .
 * @note   使用spi2
 * @param  readData : slave板读取到的数据
 * @param  readDataLen ： slave板读取到的数据长度
 * @retval BackPanelTransStatus_TypeDef
 */
BackPanelTransStatus_TypeDef BackPanelTrans_Slave_readDataFrom_Master(uint8_t *readData, uint16_t *readDataLen)
{
	BackPanelTransStatus_TypeDef bpstatus;
	// uint32_t rxTickstart = xTaskGetTickCount();
	if (MSP_SPI2_CS_STATUS())
	{
		return bpTrans_Continue;
	}

	/* 1------slave板 发送 ACK signal 给 master板 */
	bpstatus = BackPanel_WriteACK(SPI_SLAVE_ACK);
	if (!bpstatus)
	{
		if (_printDebug)
			Addto_osPrintf("slave tx ack error\r\n");
		return bpstatus;
	}
	/* 2------接收master板发送过来的数据------------ */
	// while (MSP_SPI2_CS_STATUS()) {}
	bpstatus = BackPanel_readDATA_withPacket();
	// while (!MSP_SPI2_CS_STATUS())
	//{
	//	if (xTaskGetTickCount() - rxTickstart > 2) return;
	// }
	for (int i = 0; i < (pktHandle.bytesRead); i++)
	{
		readData[i] = pktHandle.PrxBuff[i];
		if (_printDebug)
			Addto_osPrintf("%02X ", readData[i]);
	}
	*readDataLen = pktHandle.bytesRead;
	if (_printDebug)
		Addto_osPrintf("\r\n......receive SPI_MASTERWrite_ACK!.....%d\r\n", *readDataLen);
	return bpTrans_OK;
}

/*****************************************************slave board end********************************************************************************************/

#else

/****************************************************      用于master board start       ************************************************************/
/**
 * @brief  BackPanelTrans_Master_readDataFrom_Slave.
 * @param  currentID: master板发送给Slave板的板ID
 * @param  readData : master板读取到的数据
 * @param  readDataLen ： master板读取到的数据长度
 * @retval
 */
BackPanelTransStatus_TypeDef BackPanelTrans_Master_readDataFrom_Slave(BoardID_TypeDef currentID, uint8_t *readData, uint16_t *readDataLen)
{
	BackPanelTransStatus_TypeDef bpstatus;
	//while(HAL_SPI_GetState(&BackPanelTransPort) != HAL_SPI_STATE_READY) {}
	if (_printDebug) LOG("start read data from power board %d\r\n", currentID);
	
	if (waitSPISTATEready() != bpTrans_OK)	return bpTrans_SPIIT_Err;	//等待SPI可用
	SPI1_CS_ENABLE(currentID);
	/* 1------主控板发送 同步信号sync signal 给 slave板 */
	bpstatus = Master_Synchro();	//bpstatus = BackPanel_WriteACK(SPI_MASTER_ACK);
	if (waitSPISTATEready() != bpTrans_OK)	return bpTrans_SPIIT_Err;	//等待SPI可用
	if (bpstatus != bpTrans_OK)
	{
		if (_printDebug) LOG("Master_Synchro error\r\n");
		return bpstatus;
	}
	/* 2------从slave板接收数据------------ */
	// LOGI("start read data from slave ......\r\n");
	bpstatus = BackPanel_readDATA_withPacket(); // 1-----接收数据
	if (waitSPISTATEready() != bpTrans_OK)	return bpTrans_SPIIT_Err;	//等待SPI可用
	if (_printDebug) LOG("end read data from slave ......\r\n");
	SPI1_CS_DISABLE(currentID);

#if 1
	for (int i = 0; i < (pktHandle.bytesRead); i++)
	{
		readData[i] = pktHandle.PrxBuff[i];
		if (_printDebug) LOG("%d ", readData[i]);
	}
	*readDataLen = pktHandle.bytesRead;
	if (_printDebug) LOG("\r\n.......%d\r\n", *readDataLen);
#endif

	return bpstatus;
}

/**
 * @brief  BackPanelTrans_Master_readDataFrom_Slave.
 * @param  currentID: master板发送给Slave板的板ID
 * @param  writeData : master板发送给slave板的数据
 * @param  writeDataLen ： master板发送给slave板的数据长度
 * @retval
 */
BackPanelTransStatus_TypeDef BackPanelTrans_Master_writeDataTo_Slave(BoardID_TypeDef currentID, uint8_t *writeData, uint16_t writeDataLen)
{
	BackPanelTransStatus_TypeDef bpstatus;
	if (_printDebug)
		Addto_osPrintf("currentChipID : %d\r\n", currentID);
	if (_printDebug)
		Addto_osPrintf("u8regs_size : %d\r\n", writeDataLen);

	if (_printDebug)
	{
		Addto_osPrintf("u8regs data :");
		for (int i = 0; i < writeDataLen; i++)
		{
			Addto_osPrintf("%02X ", writeData[i]);
		}
		Addto_osPrintf("\r\n");
	}
	SPI1_CS_ENABLE(currentID); // SPI1_CS_DISABLE(stransH->currentChipID);
	/* master板从slave板读取ack信号 */
	bpstatus = BackPanel_ReadACK(SPI_ACK_BYTES); //
	if (!bpstatus)
	{
		if (_printDebug)
			Addto_osPrintf("read ack from slave error.................................\r\n");
		SPI1_CS_DISABLE(currentID);
		return bpstatus;
	}
	/* 2------往slave板发送数据------------ */
	uint8_t sendSize = 0;
	// PacketTxObj(PLCCmd, sendSize, 15);	//PacketTxObj(stransH->spiTx_uartRx_Buffer, sendSize);
	PacketTxObj(writeData, sendSize, writeDataLen);
	// sendSize = writeDataLen;			//sendSize = stransH->spiTx_uartRx_Buffer_Size;
	uint8_t numBytesIncl = PacketConstructPacket(writeDataLen, currentID);
	BackPanel_WriteDATA_withPacket(numBytesIncl);
	SPI1_CS_DISABLE(currentID);
	if (_printDebug)
		Addto_osPrintf("end Master_writeDataTo_Slave ...%ld\r\n", xTaskGetTickCount());
	return bpTrans_OK;
}

/****************************************************      用于master board end       *****************************************************************/
#endif

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
void BackPanel_reset()
{
	uint8_t txdata = 0xFF;
	MSP_SPI_write(&BackPanelTransPort, &txdata, 1); //_spi->transfer(0xFF);
	PacketReset();
	// status = packet.status;
	// LOG("spi transfer reset \r\n");
}
