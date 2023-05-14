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

#include "stdarg.h"

//#include <SEGGER_RTT.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MSP_GPIO.h"
#include "MSP_USART.h"
#include "MSP_SPI.h"
#include "BackPanelTrans_C.h"
//#include "stdio.h"
#include "PinConfig.h"

#include "Modbus.h"
#include "cmsis_os.h"
#include "dma.h"
#include "LOG.h"

#if 0
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
#endif



#define RxFlag                          1
#define TxFlag                          2

//复制一个数组到另一个数组
#define COPY_ARRAY(dest, src, len) memcpy(dest, src, (len) * sizeof((src)[0]))
// 板ID位操作函数
//#define Enable_Board(reg, bit)   ((reg) |= (1 << (bit)))
//#define Disable_Baord(reg, bit) ((reg) &= ~(1 << (bit)))
#define whichBoard_Enable(reg, bit)  ((reg) & (1 << (bit)))         //(reg & (1 << bit)) >> bit
//#define TOGGLE_BIT(reg, bit) ((reg) ^= (1 << (bit)))
#define MOD_START_BYTE                  0x7E
//#define MOD_CMD_CODE                    0x03
#define MOD_PREAMBLE_SIZE               4
extern uint8_t mod_preamble[MOD_PREAMBLE_SIZE];  //   = {MOD_START_BYTE, 0, 0, 0};
//uint8_t mod_postamble[2] = {0, STOP_BYTE};


#define ModbusDATASize            600     //modubs协议中，存放数据的寄存器长度
//#define ModbusDATA_AddrOffset     1536    //0x0600



extern DMA_HandleTypeDef hdma_usart1_tx;
void Error_Handler(void);
extern uint8_t USART_DMA_TX_OVER;
extern modbusHandler_t ModbusH;
extern uint16_t ModbusDATA[ModbusDATASize], ModbusDATA_Cache[ModbusDATASize];
extern SlaveBoardHandler_t PowerBoardH[4];

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
