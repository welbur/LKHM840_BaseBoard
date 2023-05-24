#ifndef _MSP_SPI_H_
#define _MSP_SPI_H_

#ifdef __cplusplus
 extern "C" {
#endif

//baseboard

#include "stm32f4xx_hal.h"
//#include "stdio.h"
//#include "SPITransfer_C.h"
#include "LOG.h"
#include "PinConfig.h"
//#include "main.h"

/* USER CODE BEGIN Private defines */
#define SPI1_MODE SPI_MODE_MASTER         //设置SPI1为主模式
#define SPI2_MODE SPI_MODE_SLAVE          //设置SPI2为从模式
#define SET_SPIMODE_MASTER				1
#define SET_SPIMODE_SLAVE				0

#ifdef LKHM840BaseB
#define SPI1_CS_ENABLE(__BoardID__)       do{if((__BoardID__) == PowerBoard_1)    HAL_GPIO_WritePin(PowerB_SPI1_CS1_Port,PowerB_SPI1_CS1,GPIO_PIN_RESET); else \
                                        	if((__BoardID__) == PowerBoard_2)    HAL_GPIO_WritePin(PowerB_SPI1_CS2_Port,PowerB_SPI1_CS2,GPIO_PIN_RESET); else \
                                           	if((__BoardID__) == PowerBoard_3)    HAL_GPIO_WritePin(PowerB_SPI1_CS3_Port,PowerB_SPI1_CS3,GPIO_PIN_RESET); else \
                                           	if((__BoardID__) == PowerBoard_4)    HAL_GPIO_WritePin(PowerB_SPI1_CS4_Port,PowerB_SPI1_CS4,GPIO_PIN_RESET); \
                                        }while(0)
#define SPI1_CS_DISABLE(__BoardID__)      do{if((__BoardID__) == PowerBoard_1)    HAL_GPIO_WritePin(PowerB_SPI1_CS1_Port,PowerB_SPI1_CS1,GPIO_PIN_SET); else \
                                           	if((__BoardID__) == PowerBoard_2)    HAL_GPIO_WritePin(PowerB_SPI1_CS2_Port,PowerB_SPI1_CS2,GPIO_PIN_SET); else \
                                           	if((__BoardID__) == PowerBoard_3)    HAL_GPIO_WritePin(PowerB_SPI1_CS3_Port,PowerB_SPI1_CS3,GPIO_PIN_SET); else \
                                           	if((__BoardID__) == PowerBoard_4)    HAL_GPIO_WritePin(PowerB_SPI1_CS4_Port,PowerB_SPI1_CS4,GPIO_PIN_SET); \
                                        }while(0)
#endif

/*
enum {
	SpiTxRx_WAIT,
	SpiTx_COMPLETE,
	SpiRx_COMPLETE,
	SpiTxRx_ERROR
};*/


extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle);

void SPITransfer_GPIO_Init(void);

uint8_t MSP_SPI_write(SPI_HandleTypeDef* spiHandle, uint8_t *txData, uint16_t txLen);
uint8_t MSP_SPI_read(SPI_HandleTypeDef* spiHandle, uint8_t *rxData, uint16_t rxLen);
#ifdef LKHM840PowerB
uint8_t MSP_SPI2_CS_STATUS();
#endif

uint8_t SPI1_WriteData(uint8_t *data,uint16_t size);
uint8_t SPI2_ReadWriteByte(uint8_t TxData);


#ifdef __cplusplus
}
#endif

#endif
