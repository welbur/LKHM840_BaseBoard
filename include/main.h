/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention      : 2023-04-11  V0.0.4
  *
  *       1、usart1用于打印测试信息
  *           PA9     ------> USART1_TX
  *           PA10    ------> USART1_RX
  *           BaudRate = 115200
  *           
  *       2、usart2用做modbus从站使用，slaveID默认为0x01
  *           PA2     ------> USART1_TX
  *           PA3     ------> USART1_RX
  *           BaudRate = 921600
  *           采用dma方式
  *           
  *       3、spi1设置为主模式，用于跟slave板通讯
  *           PA4     ------> SPI1_NSS
  *           PA5     ------> SPI1_SCK
  *           PA6     ------> SPI1_MISO
  *           PA7     ------> SPI1_MOSI
  * 
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdarg.h"

//#include <SEGGER_RTT.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MSP_GPIO.h"
#include "MSP_USART.h"
#include "MSP_SPI.h"
#include "SPITransfer_C.h"
#include "stdio.h"
#include "SlaveBoard.h"
//#include "SlaveBoardConfig.h"

#include "Modbus.h"
#include "cmsis_os.h"
#include "dma.h"
#include "LOG.h"
//#include "tftlcd.h"
//#include "w25qxx.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/


/* Private defines -----------------------------------------------------------*/
#if 0
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC

#define RED_Pin GPIO_PIN_0
#define RED_GPIO_Port GPIOC
#define GREEN_Pin GPIO_PIN_1
#define GREEN_GPIO_Port GPIOC
#define BLUE_Pin GPIO_PIN_2
#define BLUE_GPIO_Port GPIOC

#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_6
#define LCD_RST_GPIO_Port GPIOB
#define LCD_PWR_Pin GPIO_PIN_7
#define LCD_PWR_GPIO_Port GPIOB
#endif

#define USEDGPIOx_CLK_ENABLE(__INDEX__)     do{if((__INDEX__) == 0) KEY_Pin_CLK_ENABLE(); else \
                                               if((__INDEX__) == 1) DIB_INT_PIN1_CLK_ENABLE(); else \
                                               if((__INDEX__) == 2) DIB_INT_PIN2_CLK_ENABLE(); \
                                                 }while(0)


// 位操作函数
//#define SET_BIT(reg, bit)   ((reg) |= (1 << (bit)))
//#define CLEAR_BIT(reg, bit) ((reg) &= ~(1 << (bit)))
#define isBIT_SET(reg, bit)  ((reg) & (1 << (bit)))         //(reg & (1 << bit)) >> bit
//#define TOGGLE_BIT(reg, bit) ((reg) ^= (1 << (bit)))

#define ModbusDATASize            600     //modubs协议中，存放数据的寄存器长度
//#define ModbusDATA_AddrOffset     1536    //0x0600

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8;  

/*
typedef struct
{
  SpiTransStatus_TypeDef    sTransState[sTransBoard_Max];
  activeBoard_TypeDef       activeBoard;

}SlaveBoardStatus_TypeDef;
*/
#define printf LOG
extern DMA_HandleTypeDef hdma_usart1_tx;
void Error_Handler(void);
extern uint8_t USART_DMA_TX_OVER;
extern modbusHandler_t ModbusH;
extern uint16_t ModbusDATA[ModbusDATASize], ModbusDATA_Cache[ModbusDATASize];
extern SlaveBoardHandler_t SlaveBoardH[8];
//extern SlaveBoardHandler_t D_I_1_BoardH, D_I_2_BoardH, D_I_3_BoardH, D_I_4_BoardH, D_Q_1_BoardH, D_Q_2_BoardH, MENU_BoardH, RS485_BoardH;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
