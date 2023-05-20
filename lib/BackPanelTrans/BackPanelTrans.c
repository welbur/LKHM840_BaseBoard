// #include "Arduino.h"

// #if not(defined(MBED_H) || defined(__SAM3X8E__) || defined(DISABLE_SPI_SERIALTRANSFER)) // These boards are/will not be supported by SPITransfer.h

//#include "BackPanelTrans.h"
#include "BackPanelTrans_C.h"

//PacketHandler_t PktHandle;

//extern PacketHandler_t pktHandle;

#if 0
void SPITrans_Init(void)		//(SPITransHandler_t *stransH)
{
	PacketInit();
	CRC_Init(0x9B, 8);
	LOG("test....%d\r\n", pktHandle.preamble[0]);
}
#endif

/**
 * @brief  Master_writeACKto_Slave
 * @param  cs: 指向Slave板的cs引脚
 * @note   Master板 发送同步信号给Slave板.
 * @retval 发送成功 返回 1； 发送失败 返回 0
 */
uint8_t SPITransfer_Master_writeSyncto_Slave(SPITransHandler_t *stransH, SPITrans_ACK_TypeDef ACKValue)
{
	return MSP_SPI_write(stransH->spiPort, &(ACKValue), 1);
}

/**
 * @brief  Master_SyncWith_Slave
 * @param  rxData: ack数据
 * @note   slave板 读取slave板发送过来的ack信号.
 * @retval 发送成功 返回 1； 发送失败 返回 0
 */
uint8_t SPITransfer_Master_SyncWith_Slave(SPITransHandler_t *stransH, SPITrans_ACK_TypeDef ACKValue)
{
	uint8_t rxack;
	uint32_t msTickstart = xTaskGetTickCount();
	//LOG("readACKTimeOut : %d\r\n", stransH->readACKTimeOut);
	do
	{
		if (!MSP_SPI_read(stransH->spiPort, &rxack, 1))
		{
		} 
		//LOG("rxack : %d\r\n", rxack);

		if (rxack == ACKValue)
		{
			return 1;
		}
	} while ((xTaskGetTickCount() - msTickstart) < stransH->readACKTimeOut); //{0xAC, 0xA0, 0xB0, 0x30, 0xE1};
	Addto_osPrintf("timeout ...msTickstart : %ld\r\n", (xTaskGetTickCount() - msTickstart));
	return 0;
}

uint8_t SPITransfer_Master_writeDATAto_Slave_withPacket(SPITransHandler_t *stransH, const uint16_t messageLen, const uint8_t boardID)
{
	uint8_t numBytesIncl = PacketConstructPacket(messageLen, boardID);
	//LOG("numBytesIncl : %d\r\n", numBytesIncl);
	//for (int i =0; i < sizeof(pktHandle.preamble); i++) LOG(" %d ", pktHandle.preamble[i]);
	//for (int i=0; i<numBytesIncl;i++) 					LOG(" %02X ", pktHandle.PtxBuff[i]);
	//for (int i=0; i<sizeof(pktHandle.postamble);i++) 	LOG(" %d ", pktHandle.postamble[i]);

	MSP_SPI_write(stransH->spiPort, pktHandle.preamble, sizeof(pktHandle.preamble));
	MSP_SPI_write(stransH->spiPort, pktHandle.PtxBuff, numBytesIncl);
	MSP_SPI_write(stransH->spiPort, pktHandle.postamble, sizeof(pktHandle.postamble));
	return numBytesIncl;
}

void SPITransfer_Master_readDATAfrom_Slave_withPacket(SPITransHandler_t *stransH)
{
	uint32_t msTickstart = xTaskGetTickCount(); // HAL_GetTick();
	uint8_t recChar = 0xF0;
	do
	{
		if (!MSP_SPI_read(stransH->spiPort, &recChar, 1))
		{
			stransH->spiTransState = SpiTrans_Err;
			return;
		}
		//Addto_osPrintf("%d, ", recChar);
		PacketParse(&recChar, true);	//bytesRead = PacketParse(recChar, true);
		//status = packet.status;
		//LOG("time : %ld, status : %d\r\n", (xTaskGetTickCount() - msTickstart), pktHandle.status);
		if (pktHandle.status != CONTINUE)
		{
			Addto_osPrintf("\r\nstatus : %d, bytesRead : %d\r\n", pktHandle.status, pktHandle.bytesRead);
			if (pktHandle.status < 0)
			{
				SPITransfer_reset(stransH);
				stransH->spiTransState = SpiTrans_S2M_RxData_Err;
				return;
			} //{sTransState[cBoard] = SpiTrans_S2M_RxData_Err; reset();}
			stransH->spiTransState = SpiTrans_S2M_RxData_End;
#if 0 // 打印测试信息
  			LOGI("read spi data len : %d\r\n", rxi);
  			for (int i = 0; i < rxi; i++) {
    			LOGI("%d, ", spiRxData[i]);
  			}
  			LOGI("-------------%ld\r\n", xTaskGetTickCount());
#endif
			//_slavebH->spiRx_uartTx_u8regs_size = rxi;
			//LOG("return.....\r\n");
			return; // sTransState[cBoard] = SpiTrans_S2M_RxData_End;
		}
	} while ((xTaskGetTickCount() - msTickstart) < stransH->readDataTimeOut);//sTrans_TimeOut); // while(recChar != 129); //0xAA);

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
	if ((xTaskGetTickCount() - msTickstart) > stransH->readDataTimeOut)
	{
		//strcat(LOG_MSG[1], "spi trans timeout\r\n"); 	
		Addto_osPrintf("spi trans timeout\r\n");
	}
#endif
	stransH->spiTransState = SpiTrans_TimeOut;
	return;
}

/**
 * @brief  Master Synchronization with Slave.
 * @param  currentBoard_TypeDef 当前触发的板子号
 * @retval None                                            如果同步成功，返回true
 */
void SPITransfer_Master_Spi1_Transfer(SPITransHandler_t *stransH)	//(uint8_t TxRxFlag, uint8_t boardID)
{
	/*****如果没收到slave板发过来的ack信号， master板就重新发送一次ack信号给slave板，循环3次都失败的话，退出报错*****/
#if 1
	if (stransH->SPITransDir == Master_readDataFrom_Slave)
	{
		Addto_osPrintf("start read data from power board %d\r\n", stransH->currentPBoardID);
		SPI1_CS_ENABLE(stransH->currentPBoardID);
		//LOG("spi1 cs set\r\n");
#if 1
		/* 1------主控板发送 ACK signal 给 slave板 */
		if (!SPITransfer_Master_writeSyncto_Slave(stransH, SPI_MASTERRead_ACK))
		{
			Addto_osPrintf("tx ack error\r\n");
			return;
		}
#endif
		/* 2------从slave板接收数据------------ */
		//LOGI("start read data from slave ......\r\n");
		SPITransfer_Master_readDATAfrom_Slave_withPacket(stransH); // 1-----接收数据
		Addto_osPrintf("end read data from slave ......\r\n");
		SPI1_CS_DISABLE(stransH->currentPBoardID);
		if (stransH->spiTransState != SpiTrans_S2M_RxData_End)
		{
			Addto_osPrintf("read rxdata err\r\n");
			return;
		}

#if 1
		for (int i = 0; i < (pktHandle.bytesRead); i++)
		{
			stransH->readDataFromPowerBoard_u8regs[i] = pktHandle.PrxBuff[i];
			Addto_osPrintf("%d ", stransH->readDataFromPowerBoard_u8regs[i]);
		}
		stransH->readDataFromPowerBoard_u8regs_size = pktHandle.bytesRead;
		Addto_osPrintf(".......%d\r\n", pktHandle.bytesRead);
#endif
	}
	else if (stransH->SPITransDir == Master_writeDataTo_Slave)
	{
		//uint8_t *PLCCmd;
		//*PLCCmd = stransH->PLCCmd_u8regs_p;
		//LOG("Master_writeDataTo_Slave\r\n");
		Addto_osPrintf("currentPBoardID : %d\r\n", stransH->currentPBoardID);
		Addto_osPrintf("u8regs_size : %d\r\n", stransH->PLCCmd_u8regs_size);
		
		Addto_osPrintf("u8regs data :");
		for  (int i = 0; i < stransH->PLCCmd_u8regs_size; i++)
		{
			Addto_osPrintf("%02X ", stransH->PLCCmd_u8regs[i]);
		}
		Addto_osPrintf("\r\n");
		SPI1_CS_ENABLE(stransH->currentPBoardID);	//SPI1_CS_DISABLE(stransH->currentPBoardID);
		//Addto_osPrintf("board_id 0 cs value = %d \r\n", HAL_GPIO_ReadPin(PowerB_SPI1_CS1_Port, PowerB_SPI1_CS1));
		//Addto_osPrintf("board_id 1 cs value = %d \r\n", HAL_GPIO_ReadPin(PowerB_SPI1_CS2_Port, PowerB_SPI1_CS2));
		//Addto_osPrintf("board_id 2 cs value = %d \r\n", HAL_GPIO_ReadPin(PowerB_SPI1_CS3_Port, PowerB_SPI1_CS3));
		//Addto_osPrintf("board_id 3 cs value = %d \r\n", HAL_GPIO_ReadPin(PowerB_SPI1_CS4_Port, PowerB_SPI1_CS4));
		#if 1
		if (!SPITransfer_Master_SyncWith_Slave(stransH, SPI_SLAVE_ACK)) {
			Addto_osPrintf("read ack from slave error.................................\r\n");
			SPI1_CS_DISABLE(stransH->currentPBoardID);
			return;
		}
		#endif
		// SPI1_CS_DISABLE(boardID);
		/* 2------往slave板发送数据------------ */
		uint8_t sendSize = 0;
		//PacketTxObj(PLCCmd, sendSize, 15);	//PacketTxObj(stransH->spiTx_uartRx_Buffer, sendSize);
		PacketTxObj(stransH->PLCCmd_u8regs, sendSize, stransH->PLCCmd_u8regs_size);
		sendSize = stransH->PLCCmd_u8regs_size;			//sendSize = stransH->spiTx_uartRx_Buffer_Size;
		SPITransfer_Master_writeDATAto_Slave_withPacket(stransH, sendSize, stransH->currentPBoardID);
		SPI1_CS_DISABLE(stransH->currentPBoardID);
		Addto_osPrintf("end Master_writeDataTo_Slave ...%ld\r\n", xTaskGetTickCount());
	}
	else
	{
		Addto_osPrintf("txrxflag error \r\n");
	}
#endif
	/* 3------接收完数据后，master板发送end信号给slave板------------ */
	//	if (!MSP_SPI_write(_spi, _cs, &SPI_Trans_END, 1)) { LOGI("write end to slave err\r\n"); _slavebH->spiTransState = SpiTrans_Err; return; }
	stransH->spiTransState = SpiTrans_End;

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
//uint8_t SPITransfer_currentPacketID()
//{
//	return packet.currentPacketID();
//}

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
void SPITransfer_reset(SPITransHandler_t *stransH)
{
	uint8_t txdata = 0xFF;
	MSP_SPI_write(stransH->spiPort, &txdata, 1); //_spi->transfer(0xFF);
	PacketReset();
	//status = packet.status;
	//LOG("spi transfer reset \r\n");
}

/*
 * 用于c调用
 */
#if 0
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
#endif
// sendSize = packet.txObj(test_TxBuff, sendSize);  		//1-------封装数据
// extern void SPITransfer_C_TxObj(void *SpiTrans, uint8_t TxRxFlag, BoardID_TypeDef boardID)
//{
//	uint8_t sendSize = 0;
//	sendSize = packet.txObj(test_TxBuff, sendSize);  		//1-------封装数据
//	return;
// }

// #endif // not (defined(MBED_H) || defined(__SAM3X8E__))
