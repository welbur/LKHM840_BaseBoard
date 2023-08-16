/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention      : LKHM840 BaseBoard
  *                   2023-05-20  V0.0.1
  *
  *       1、usart1用于接收PN板的modbus数据
  *           PA9     ------> USART1_TX
  *           PA10    ------> USART1_RX
  *           BaudRate = 921600
  *           
  *       2、usart2用于扩展多个LKHM840模块，将usart1数据直接透传给下一个LKHM840
  *           PA2     ------> USART2_TX
  *           PA3     ------> USART2_RX
  *           BaudRate = 921600
  *     
  *       3、usart3用于打印测试信息
  *           PC10    ------> USART3_TX
  *           PC11    ------> USART3_RX
  *           BaudRate = 921600
  *           
  *       4、spi1设置为主模式，用于跟slave板通讯
  *           PA5     ------> SPI1_SCK
  *           PA6     ------> SPI1_MISO
  *           PA7     ------> SPI1_MOSI
  *       5、CS信号、INT信号、WorkLed
  *           PB4     ------> CS1       BaseBoard -->  PowerBoard
  *           PB5     ------> CS2       BaseBoard -->  PowerBoard
  *           PB6     ------> CS3       BaseBoard -->  PowerBoard
  *           PB7     ------> CS4       BaseBoard -->  PowerBoard
  *           
  *           PC6     ------> INT1      PowerBoard --> BaseBoard
  *           PC5     ------> INT2      PowerBoard --> BaseBoard
  *           PC4     ------> INT3      PowerBoard --> BaseBoard
  *           PC3     ------> INT4      PowerBoard --> BaseBoard
  *           
  *           PC0     ------> WorkLed
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
#include "BackPanelTrans.h"
//#include "stdio.h"
#include "PinConfig.h"

#include "Modbus.h"
#include "cmsis_os.h"
#include "dma.h"
#include "LOG.h"

#include "tim.h"
#include "can.h"


//#define getSlaveBoardInfo_TimeOut	2000

#define Default_SlaveTranstoMasterMode_Trigger 		1
#define Default_SlaveTranstoMasterMode_Loop 		  2

#define Default_SlaveTranstoMasterMode_Loop_Time 	50

#define DefaultbpCMDLen                  4     //有效数据(4)
#define DefaultbpDATALen                 120   //有效数据(120)

//#define RxFlag                          1
//#define TxFlag                          2

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

#define ModbusDATASize            800     //modubs协议中，存放数据的寄存器长度


/**
 * @struct slaveboardINFOHandler_t
 * @param boardID ： 从板的ID，由硬件引脚bID0和bID1决定
 * @param SlaveTranstoMasterMode : 从板发送数据给主板的模式，有两种 触发模式 和 循环模式
 * 								触发模式：设置触发条件(电流阀值或者功率阀值等)，然后主动上报给主板
 * 								循环模式：从板读取完芯片的数据后，保存到指定寄存器，然后等待主板过来读取。
 * SPITrans handler structure
 * 用于 spi trans的初始化参数
 */
typedef struct
{
	uint8_t 			boardID;				//从板发给主板的信息1 ： 从板ID
	uint8_t				BackPanelDataLen;		//从板发给主板的信息2 ： 从办的数据长度(包括 包头+有效数据+包尾)
	uint8_t				SlaveTranstoMasterMode;	//主板发给从办的信息1 : 触发模式和循环模式
	uint16_t			LoopTimeV;				//主板发给从办的信息2 ：循环模式下的轮询时间间隔
} slaveboardINFOHandler_t;


extern DMA_HandleTypeDef hdma_usart1_tx;
extern uint8_t USART_DMA_TX_OVER;
extern modbusHandler_t ModbusH;
extern uint16_t ModbusDATA[ModbusDATASize], ModbusDATA_Cache[ModbusDATASize];
extern uint8_t PowerBoard_Trig[PowerBoardNum];   //extern SlaveBoardHandler_t PowerBoardH[4];
extern uint8_t PowerBoard_DATA[255], MasterB2PowerB_Cmd[128];


//void getSlaveBoardInfo(void);
void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
