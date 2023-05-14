
#ifndef _BACKPANELTRANS_C_H_
#define	_BACKPANELTRANS_C_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
//#include "FreeRTOS.h"
//#include "task.h"
#include "PinConfig.h"
#include "LOG.h"
//#include "main.h"

//#define SlaveBoard_Max                  8       //定义板子的数量 di板4块，dq板2块，rs485板1块，menu板1块
#define PowerBoardNum                   4       //定义DI板子的数量 4块
#define sTrans_TimeOut                  100       //5 = 5ms
#define sTxRx_TimeOut                   1
#define spiRxDataLen                    100

typedef enum 
{
  SpiTrans_TimeOut          = -5,
  SpiTrans_S2M_RxData_Err   = -4,
  SpiTrans_S2M_RxAck_Err    = -3,
  SpiTrans_M2S_TxAck_Err    = -2,
  SpiTrans_Err              = -1,
  SpiTrans_Wait             = 0, 
  SpiTrans_M2S_TxAck_End,
  SpiTrans_S2M_RxData_End,
  SpiTrans_WakeUp,
  SpiTrans_Continue,
  SpiTrans_End
}SpiTransStatus_TypeDef;

#if 1
typedef enum
{
  PowerBoard_1    = 0,     //= 0b00000001,
  PowerBoard_2    ,     //= 0b00000010,	//2,
  PowerBoard_3    ,     //= 0b00000100,
  PowerBoard_4    ,     //= 0b00001000,
}BoardID_TypeDef;
#endif

/**
 * @struct D_I_BoardHandler_t
 * @brief
 * DI board handler structure
 * Contains all the variables required for Modbus daemon operation
 */
typedef struct 
{
  BoardID_TypeDef           BoardID;
  uint8_t                   isBoard_Rx_En;
  uint8_t                   isBoard_Tx_En;
  SpiTransStatus_TypeDef    spiTransState;
  uint8_t                   spiRx_uartTx_Buffer[spiRxDataLen];
  uint8_t                   spiTx_uartRx_Buffer[spiRxDataLen];
  uint8_t                   spiRx_uartTx_Buffer_Size;
  uint8_t                   spiTx_uartRx_Buffer_Size;

  uint8_t                  *spiRx_uartTx_u8regs;
  uint8_t                  *spiTx_uartRx_u8regs;
  uint8_t                  spiRx_uartTx_u8regs_size;
  uint8_t                  spiTx_uartRx_u8regs_size;

  uint16_t                  *spiRx_uartTx_u16regs;
  uint16_t                  *spiTx_uartRx_u16regs;
  uint16_t                  spiRx_uartTx_u16regs_size;
  uint16_t                  spiTx_uartRx_u16regs_size;
}SlaveBoardHandler_t;


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

//extern void Addto_osPrintf(char *format, ...);
extern void *SPITransfer_C_New(SlaveBoardHandler_t *slavebH, SPI_HandleTypeDef *theSPI, uint8_t master);
extern void SPITransfer_C_Master_Spi1_Transfer(void *SpiTrans, uint8_t TxRxFlag, BoardID_TypeDef boardID);


#ifdef __cplusplus
}
#endif

#endif
