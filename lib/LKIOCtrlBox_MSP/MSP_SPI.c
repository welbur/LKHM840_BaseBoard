/**
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  * @file       BSP_spi.c/h
  * @brief      ��HAL��SPI�������ж��η�װ������Ӧ���������е���
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V3.0.0     2020.7.14     	              	1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  */
#include "MSP_SPI.h"

/* transfer state */
__IO uint32_t sTxRxFlag;
// uint32_t wTransferState = TRANSFER_WAIT;
// #include "spi.h"

/**
 * @brief          ��װSPI6д����
 * @param[in]     	data ����������
 * @retval         �յ�������
 */
uint8_t SPI1_WriteData(uint8_t *data, uint16_t size)
{
	printf("transmit spi1 data....\r\n");
	return HAL_SPI_Transmit(&hspi1, data, size, 1000);
}

/**
 * @brief          ��װSPI2��д������Ƭ��SPI Flashʹ�ã�
 * @param[in]     	TxData ����������
 * @retval         RxData �յ�������
 */
uint8_t SPI2_ReadWriteByte(uint8_t TxData)
{
	uint8_t Rxdata[5];
	HAL_SPI_Receive(&hspi2, Rxdata, 5, 1000);
	printf("spi2 read data : %d, %d, %d, %d, %d\r\n", Rxdata[0], Rxdata[1], Rxdata[2], Rxdata[3], Rxdata[4]);
	return Rxdata[0]; // �����յ�������
}

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* SPI1 init function */
void MX_SPI1_Init(void)
{
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI1_MODE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES; // SPI_DIRECTION_1LINE;                         //SPI_DIRECTION_2LINES
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;				  // SPI_POLARITY_LOW
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;					  // SPI_PHASE_1EDGE
	hspi1.Init.NSS = SPI_NSS_SOFT;							  // SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // SPI_BAUDRATEPRESCALER_256
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10; // 7
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		printf("error spi");		//Error_Handler();
	}
}
/* SPI2 init function */
void MX_SPI2_Init(void)
{
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI2_MODE;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES; // SPI_DIRECTION_1LINE;                   //SPI_DIRECTION_2LINES
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW; // SPI_POLARITY_LOW
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;	   // SPI_PHASE_1EDGE
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // SPI_BAUDRATEPRESCALER_256
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		printf("error spi");
		// Error_Handler();
	}
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (spiHandle->Instance == SPI1)
	{
		/* SPI1 clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**SPI1 GPIO Configuration
		PA4     ------> SPI1_NSS
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;				// GPIO_PULLUP;     //GPIO_NOPULL
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM; // GPIO_SPEED_FREQ_LOW;	//GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		/* NVIC for SPI */
		HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(SPI1_IRQn);
	}
	else if (spiHandle->Instance == SPI2)
	{
		/* SPI2 clock enable */
		__HAL_RCC_SPI2_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**SPI2 GPIO Configuration
		PB13     ------> SPI2_SCK
		PB14     ------> SPI2_MISO
		PB15     ------> SPI2_MOSI
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		/* NVIC for SPI */
		HAL_NVIC_SetPriority(SPI2_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(SPI2_IRQn);
	}
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{
	if (spiHandle->Instance == SPI1)
	{
		/* Peripheral clock disable */
		//__HAL_RCC_SPI1_CLK_DISABLE();
		__HAL_RCC_SPI1_FORCE_RESET();
		__HAL_RCC_SPI1_RELEASE_RESET();
		/**SPI1 GPIO Configuration
		PA4     ------> SPI1_NSS
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	}
	else if (spiHandle->Instance == SPI2)
	{
		/* Peripheral clock disable */
		//__HAL_RCC_SPI2_CLK_DISABLE();
		__HAL_RCC_SPI2_FORCE_RESET();
		__HAL_RCC_SPI2_RELEASE_RESET();
		/**SPI2 GPIO Configuration
		PB13     ------> SPI2_SCK
		PB14     ------> SPI2_MISO
		PB15     ------> SPI2_MOSI
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
		HAL_NVIC_DisableIRQ(SPI2_IRQn);
	}
}

void SPITransfer_Init(void)
{
	/*******************************************       先初始化需要用到的GPIO引脚      *************************************/
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pins : PB0 PB1 PB2 PB3 PB4 PB5 PB6 PB7
	 *        PB0~PB7定义为spi1 cs的引脚
	 */
	SPI1_DIB1_CS_CLK_ENABLE();
	GPIO_InitStruct.Pin = SPI1_DIB1_CS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_DIB1_CS_Port, &GPIO_InitStruct);
	SPI1_DIB2_CS_CLK_ENABLE();
	GPIO_InitStruct.Pin = SPI1_DIB2_CS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_DIB2_CS_Port, &GPIO_InitStruct);
	SPI1_DIB3_CS_CLK_ENABLE();
	GPIO_InitStruct.Pin = SPI1_DIB3_CS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI1_DIB3_CS_Port, &GPIO_InitStruct);
	SPI1_DIB4_CS_CLK_ENABLE();
	GPIO_InitStruct.Pin = SPI1_DIB4_CS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI1_DIB4_CS_Port, &GPIO_InitStruct);
	SPI1_DQB1_CS_CLK_ENABLE();
	GPIO_InitStruct.Pin = SPI1_DQB1_CS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI1_DQB1_CS_Port, &GPIO_InitStruct);
	SPI1_DQB2_CS_CLK_ENABLE();
	GPIO_InitStruct.Pin = SPI1_DQB2_CS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI1_DQB2_CS_Port, &GPIO_InitStruct);
	SPI1_RS485B_CS_CLK_ENABLE();
	GPIO_InitStruct.Pin = SPI1_RS485B_CS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI1_RS485B_CS_Port, &GPIO_InitStruct);
	SPI1_MENUB_CS_CLK_ENABLE();
	GPIO_InitStruct.Pin = SPI1_MENUB_CS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI1_MENUB_CS_Port, &GPIO_InitStruct);
	/*******************************************       将所有cs引脚都默认设为高电平     *************************************/
	HAL_GPIO_WritePin(SPI1_DIB1_CS_Port, SPI1_DIB1_CS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_DIB2_CS_Port, SPI1_DIB2_CS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_DIB3_CS_Port, SPI1_DIB3_CS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_DIB4_CS_Port, SPI1_DIB4_CS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_DQB1_CS_Port, SPI1_DQB1_CS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_DQB2_CS_Port, SPI1_DQB2_CS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_RS485B_CS_Port, SPI1_RS485B_CS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_MENUB_CS_Port, SPI1_MENUB_CS, GPIO_PIN_SET);
}

/**
 * @brief  MSP_SPI_write
 * @param  spiHandle: 指向哪个spi号
 * @param  cs: 指向Slave板的cs引脚
 * @param  txData: 指向需要发送数据的指针
 * @param  txLen: 需要发送数据的长度
 * @note   Master板 通过spi1发送数据给Slave板.
 * @retval 发送成功 返回 1； 发送失败 返回 0
 */

#if 1
uint8_t MSP_SPI_write(SPI_HandleTypeDef *spiHandle, uint8_t *txData, uint16_t txLen)
{
	for (uint16_t i = 0; i < txLen; i++)
	{
		if (HAL_SPI_Transmit_IT(spiHandle, (txData + i), 1) != HAL_OK)
			return 0;
	}
	return 1;
}
#endif
/**
 * @brief  MSP_SPI_read
 * @param  spiHandle: 指向哪个spi号
 * @param  cs: 指向Slave板的cs引脚
 * @param  rxData: 指向需要发送数据的指针
 * @param  rxLen: 需要发送数据的长度
 * @note   通过spi号接收指定数量的数据.
 * @retval 发送成功 返回 1； 发送失败 返回 0
 */
#if 1
uint8_t MSP_SPI_read(SPI_HandleTypeDef *spiHandle, uint8_t *rxData, uint16_t rxLen)
{
	for (uint16_t i = 0; i < rxLen; i++)
	{
		if (HAL_SPI_Receive(spiHandle, rxData, 1, 10) != HAL_OK)
		{
			// printf("spi MSP_SPI_read timeout....\r\n");
			return 0;
		}
		while (HAL_SPI_GetState(spiHandle) != HAL_SPI_STATE_READY)
		{
		}
	}
	return 1;
}
#endif


/**
 * @brief  TxRx Transfer completed callback.
 * @param  hspi: SPI handle
 * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	printf("spi rx callback\r\n");
	sTxRxFlag = SpiRx_COMPLETE;
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// printf("spi tx callback\r\n");
	sTxRxFlag = SpiTx_COMPLETE;
}

/**
 * @brief  SPI error callbacks.
 * @param  hspi: SPI handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	printf("spi error callback\r\n");
	sTxRxFlag = SpiTxRx_ERROR;
	HAL_SPI_MspDeInit(hspi);
	HAL_SPI_MspInit(hspi);
}
