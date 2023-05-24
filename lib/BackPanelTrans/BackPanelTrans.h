
#ifndef _BACKPANELTRANS_H_
#define _BACKPANELTRANS_H_

#ifdef __cplusplus
extern "C"
{
#endif

//baseboard

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "PinConfig.h"
#include "LOG.h"
#include "Packet.h"
#include "MSP_SPI.h"
// #include "main.h"


#ifdef LKHM840PowerB
#define ChipNum		8
#else
#define PowerBoardNum 4	   // 定义DI板子的数量 4块
#endif

typedef enum
{
	SPI_SLAVE_ACK 			= 0x53,
	SPI_MASTER_ACK 			= 0xAC,
	//SPI_MASTERWrite_ACK 	= 0xCA,
	SPI_Trans_END 			= 0xED,
}BackPanelTrans_ACK_TypeDef;

typedef enum
{
	bpTrans_Init_Err 			= -7,
	bpTrans_WaitCS_Err			= -6,
	bpTrans_ACK_TimeOut 		= -5,
	bpTrans_ACK_Err				= -4,
	bpTrans_Data_TimeOut		= -3,
	bpTrans_Data_Err			= -2,
	bpTrans_Wait 				= -1,
	bpTrans_SPI_Err				= 0,
	bpTrans_Continue,
	bpTrans_OK
} BackPanelTransStatus_TypeDef;

#ifdef LKHM840PowerB
typedef enum
{
	CHIPID_1 = 0, //= 0b00000001,
	CHIPID_2,	  //= 0b00000010,	//2,
	CHIPID_3,	  //= 0b00000100,
	CHIPID_4,	  //= 0b00001000,
	CHIPID_5,
	CHIPID_6,
	CHIPID_7,
	CHIPID_8,
} CHIPID_TypeDef;
#else
typedef enum
{
	PowerBoard_1 = 0, //= 0b00000001,
	PowerBoard_2,	  //= 0b00000010,	//2,
	PowerBoard_3,	  //= 0b00000100,
	PowerBoard_4,	  //= 0b00001000,
} BoardID_TypeDef;
#endif


/**
 * @struct SPITransHandler_t
 * @brief
 * SPITrans handler structure
 * 用于 spi trans的初始化参数
 */
typedef struct
{
	uint8_t 				printDebug;
#ifdef LKHM840PowerB
	uint8_t				 	slaveBoardID;		//只有作为从模式的时候才会用到
#endif
	uint16_t				waitCSTimeOut;
	uint16_t				readACKTimeOut;
	uint16_t				readDataTimeOut;
}BackPanelTransHandler_t;


#ifdef LKHM840PowerB
#define SPI1_CS_ENABLE(__ChipID__)       do{if((__ChipID__) == CHIPID_1)    HAL_GPIO_WritePin(CHIP_SPI1_CS1_Port,CHIP_SPI1_CS1,GPIO_PIN_RESET); else \
                                        	if((__ChipID__) == CHIPID_2)    HAL_GPIO_WritePin(CHIP_SPI1_CS2_Port,CHIP_SPI1_CS2,GPIO_PIN_RESET); else \
                                           	if((__ChipID__) == CHIPID_3)    HAL_GPIO_WritePin(CHIP_SPI1_CS3_Port,CHIP_SPI1_CS3,GPIO_PIN_RESET); else \
											if((__ChipID__) == CHIPID_4)    HAL_GPIO_WritePin(CHIP_SPI1_CS4_Port,CHIP_SPI1_CS4,GPIO_PIN_RESET); else \
											if((__ChipID__) == CHIPID_5)    HAL_GPIO_WritePin(CHIP_SPI1_CS5_Port,CHIP_SPI1_CS5,GPIO_PIN_RESET); else \
											if((__ChipID__) == CHIPID_6)    HAL_GPIO_WritePin(CHIP_SPI1_CS6_Port,CHIP_SPI1_CS6,GPIO_PIN_RESET); else \
											if((__ChipID__) == CHIPID_7)    HAL_GPIO_WritePin(CHIP_SPI1_CS7_Port,CHIP_SPI1_CS7,GPIO_PIN_RESET); else \
                                           	if((__ChipID__) == CHIPID_8)    HAL_GPIO_WritePin(CHIP_SPI1_CS8_Port,CHIP_SPI1_CS8,GPIO_PIN_RESET); \
                                        }while(0)
#define SPI1_CS_DISABLE(__ChipID__)      do{if((__ChipID__) == CHIPID_1)    HAL_GPIO_WritePin(CHIP_SPI1_CS1_Port,CHIP_SPI1_CS1,GPIO_PIN_SET); else \
                                           	if((__ChipID__) == CHIPID_2)    HAL_GPIO_WritePin(CHIP_SPI1_CS2_Port,CHIP_SPI1_CS2,GPIO_PIN_SET); else \
                                           	if((__ChipID__) == CHIPID_3)    HAL_GPIO_WritePin(CHIP_SPI1_CS3_Port,CHIP_SPI1_CS3,GPIO_PIN_SET); else \
                                           	if((__ChipID__) == CHIPID_4)    HAL_GPIO_WritePin(CHIP_SPI1_CS4_Port,CHIP_SPI1_CS4,GPIO_PIN_SET); else \
											if((__ChipID__) == CHIPID_5)    HAL_GPIO_WritePin(CHIP_SPI1_CS5_Port,CHIP_SPI1_CS5,GPIO_PIN_SET); else \
											if((__ChipID__) == CHIPID_6)    HAL_GPIO_WritePin(CHIP_SPI1_CS6_Port,CHIP_SPI1_CS6,GPIO_PIN_SET); else \
											if((__ChipID__) == CHIPID_7)    HAL_GPIO_WritePin(CHIP_SPI1_CS7_Port,CHIP_SPI1_CS7,GPIO_PIN_SET); else \
											if((__ChipID__) == CHIPID_8)    HAL_GPIO_WritePin(CHIP_SPI1_CS8_Port,CHIP_SPI1_CS8,GPIO_PIN_SET); \
                                        }while(0)
#else
#define SPI1_CS_ENABLE(__BoardID__)      do{if((__BoardID__) == PowerBoard_1)    HAL_GPIO_WritePin(PowerB_SPI1_CS1_Port,PowerB_SPI1_CS1,GPIO_PIN_RESET); else \
                                        	if((__BoardID__) == PowerBoard_2)    HAL_GPIO_WritePin(PowerB_SPI1_CS2_Port,PowerB_SPI1_CS2,GPIO_PIN_RESET); else \
                                           	if((__BoardID__) == PowerBoard_3)    HAL_GPIO_WritePin(PowerB_SPI1_CS3_Port,PowerB_SPI1_CS3,GPIO_PIN_RESET); else \
                                           	if((__BoardID__) == PowerBoard_4)    HAL_GPIO_WritePin(PowerB_SPI1_CS4_Port,PowerB_SPI1_CS4,GPIO_PIN_RESET); 	  \
                                        }while(0)
#define SPI1_CS_DISABLE(__BoardID__)     do{if((__BoardID__) == PowerBoard_1)    HAL_GPIO_WritePin(PowerB_SPI1_CS1_Port,PowerB_SPI1_CS1,GPIO_PIN_SET); else \
                                           	if((__BoardID__) == PowerBoard_2)    HAL_GPIO_WritePin(PowerB_SPI1_CS2_Port,PowerB_SPI1_CS2,GPIO_PIN_SET); else \
                                           	if((__BoardID__) == PowerBoard_3)    HAL_GPIO_WritePin(PowerB_SPI1_CS3_Port,PowerB_SPI1_CS3,GPIO_PIN_SET); else \
                                           	if((__BoardID__) == PowerBoard_4)    HAL_GPIO_WritePin(PowerB_SPI1_CS4_Port,PowerB_SPI1_CS4,GPIO_PIN_SET); 	 	\
                                        }while(0)
#endif


BackPanelTransStatus_TypeDef BPTrans_Init(BackPanelTransHandler_t *bpTransH);
BackPanelTransStatus_TypeDef BackPanel_WriteACK(BackPanelTrans_ACK_TypeDef ACKValue);
BackPanelTransStatus_TypeDef BackPanel_ReadACK(BackPanelTrans_ACK_TypeDef ACKValue);
BackPanelTransStatus_TypeDef BackPanel_WriteDATA_withPacket(const uint8_t numBytesIncl);
BackPanelTransStatus_TypeDef BackPanel_readDATA_withPacket();

#ifdef LKHM840PowerB
//BackPanelTransStatus_TypeDef waitSPI2CSequal(uint8_t cs_v);
BackPanelTransStatus_TypeDef BackPanelTrans_Slave_writeDataTo_Master(uint8_t *writeData, uint16_t writeDataLen);
BackPanelTransStatus_TypeDef BackPanelTrans_Slave_readDataFrom_Master(uint8_t *readData, uint16_t *readDataLen);
#else
BackPanelTransStatus_TypeDef BackPanelTrans_Master_readDataFrom_Slave(BoardID_TypeDef currentID, uint8_t *readData, uint16_t *readDataLen);
BackPanelTransStatus_TypeDef BackPanelTrans_Master_writeDataTo_Slave(BoardID_TypeDef currentID, uint8_t *writeData, uint16_t writeDataLen);
#endif

void BackPanel_reset(void);


#ifdef __cplusplus
}
#endif

#endif
