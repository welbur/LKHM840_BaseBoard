#ifndef _MSP_SPI_H_
#define _MSP_SPI_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "SPITransfer_C.h"
#include "LOG.h"
//#include "SlaveBoardConfig.h"
//#include "MSP_GPIO.h"
//#include "../../include/spi.h"


/* USER CODE BEGIN Private defines */
#define SPI1_MODE SPI_MODE_MASTER         //设置SPI1为主模式
#define SPI2_MODE SPI_MODE_SLAVE          //设置SPI2为从模式
#define SET_SPIMODE_MASTER				1
#define SET_SPIMODE_SLAVE				0


/* USER CODE END Private defines */




enum {
	SpiTxRx_WAIT,
	SpiTx_COMPLETE,
	SpiRx_COMPLETE,
	SpiTxRx_ERROR
};


extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle);

void SPITransfer_Init(void);

uint8_t MSP_SPI_write(SPI_HandleTypeDef* spiHandle, uint8_t *txData, uint16_t txLen);
uint8_t MSP_SPI_read(SPI_HandleTypeDef* spiHandle, uint8_t *rxData, uint16_t rxLen);

uint8_t SPI1_WriteData(uint8_t *data,uint16_t size);
uint8_t SPI2_ReadWriteByte(uint8_t TxData);


#ifdef __cplusplus
}
#endif

#endif
