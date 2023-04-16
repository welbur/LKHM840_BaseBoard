
#ifndef _SLAVEBOARD_H_
#define	_SLAVEBOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/**
 * @brief 定义第一块DI Board的触发信号 
 * @brief DI Board --> Master Board
 */
#if DEVBoard
#define DIB_INT_PIN1                         GPIO_PIN_3
#define DIB_INT_PIN1_EXTI_IRQn               EXTI3_IRQn    
#else
#define DIB_INT_PIN1                         GPIO_PIN_11
#define DIB_INT_PIN1_EXTI_IRQn               EXTI15_10_IRQn    
#endif
#define DIB_INT_PIN1_PORT                    GPIOC
#define DIB_INT_PIN1_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define DIB_INT_PIN1_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define DIB_INT_PIN1_EXTI_SP                 1          
/**
 * @brief 定义第二块DI Board的触发信号 
 * @brief DI Board --> Master Board
 */
#define DIB_INT_PIN2                         GPIO_PIN_4
#define DIB_INT_PIN2_PORT                    GPIOC
#define DIB_INT_PIN2_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define DIB_INT_PIN2_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define DIB_INT_PIN2_EXTI_IRQn               EXTI4_IRQn         
#define DIB_INT_PIN2_EXTI_SP                 2            
/**
 * @brief 定义第3块DI Board的触发信号 
 * @brief DI Board --> Master Board
 */
#define DIB_INT_PIN3                         GPIO_PIN_5
#define DIB_INT_PIN3_PORT                    GPIOC
#define DIB_INT_PIN3_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define DIB_INT_PIN3_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define DIB_INT_PIN3_EXTI_IRQn               EXTI9_5_IRQn          
#define DIB_INT_PIN3_EXTI_SP                 3            
/**
 * @brief 定义第4块DI Board的触发信号 
 * @brief DI Board --> Master Board
 */
#define DIB_INT_PIN4                         GPIO_PIN_6
#define DIB_INT_PIN4_PORT                    GPIOC
#define DIB_INT_PIN4_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define DIB_INT_PIN4_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define DIB_INT_PIN4_EXTI_IRQn               EXTI9_5_IRQn         
#define DIB_INT_PIN4_EXTI_SP                 4            
/**
 * @brief 定义第1块DQ Board的触发信号 
 * @brief DQ Board --> Master Board
 */
#define DQB_INT_PIN1                         GPIO_PIN_7
#define DQB_INT_PIN1_PORT                    GPIOC
#define DQB_INT_PIN1_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define DQB_INT_PIN1_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define DQB_INT_PIN1_EXTI_IRQn               EXTI9_5_IRQn          
#define DQB_INT_PIN1_EXTI_SP                 5            
/**
 * @brief 定义第2块DQ Board的触发信号 
 * @brief DQ Board --> Master Board
 */
#define DQB_INT_PIN2                         GPIO_PIN_8
#define DQB_INT_PIN2_PORT                    GPIOC
#define DQB_INT_PIN2_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define DQB_INT_PIN2_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define DQB_INT_PIN2_EXTI_IRQn               EXTI9_5_IRQn          
#define DQB_INT_PIN2_EXTI_SP                 6              
/**
 * @brief 定义RS485 Board的触发信号 
 * @brief RS485 Board --> Master Board
 */
#define RS485B_INT_PIN                         GPIO_PIN_9
#define RS485B_INT_PIN_PORT                    GPIOC
#define RS485B_INT_PIN_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define RS485B_INT_PIN_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define RS485B_INT_PIN_EXTI_IRQn               EXTI9_5_IRQn    
#define RS485B_INT_PIN_EXTI_SP                 0
/**
 * @brief 定义Menu Board的触发信号 
 * @brief MENU Board --> Master Board
 */
#define MENUB_INT_PIN                         GPIO_PIN_10
#define MENUB_INT_PIN_PORT                    GPIOC
#define MENUB_INT_PIN_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define MENUB_INT_PIN_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define MENUB_INT_PIN_EXTI_IRQn               EXTI15_10_IRQn    //未定义
#define MENUB_INT_PIN_EXTI_SP                 7

#define GPIO_EXTI_PP              2       //定义GPIO中断的主优先级

//spi1 DI_Board 1 cs引脚
#define SPI1_DIB1_CS                         GPIO_PIN_0
#define SPI1_DIB1_CS_Port                    GPIOB
#define SPI1_DIB1_CS_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_DIB1_CS_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
//spi1 DI_Board 2 cs引脚
#define SPI1_DIB2_CS                         GPIO_PIN_1
#define SPI1_DIB2_CS_Port                    GPIOB
#define SPI1_DIB2_CS_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_DIB2_CS_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
//spi1 DI_Board 3 cs引脚
#define SPI1_DIB3_CS                         GPIO_PIN_2
#define SPI1_DIB3_CS_Port                    GPIOB
#define SPI1_DIB3_CS_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_DIB3_CS_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
//spi1 DI_Board 4 cs引脚
#define SPI1_DIB4_CS                         GPIO_PIN_3
#define SPI1_DIB4_CS_Port                    GPIOB
#define SPI1_DIB4_CS_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_DIB4_CS_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
//spi1 DQ_Board 1 cs引脚
#define SPI1_DQB1_CS                         GPIO_PIN_4
#define SPI1_DQB1_CS_Port                    GPIOB
#define SPI1_DQB1_CS_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_DQB1_CS_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
//spi1 DQ_Board 2 cs引脚
#define SPI1_DQB2_CS                         GPIO_PIN_5
#define SPI1_DQB2_CS_Port                    GPIOB
#define SPI1_DQB2_CS_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_DQB2_CS_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
//spi1 RS485_Board cs引脚
#define SPI1_RS485B_CS                       GPIO_PIN_6
#define SPI1_RS485B_CS_Port                  GPIOB
#define SPI1_RS485B_CS_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_RS485B_CS_CLK_DISABLE()         __HAL_RCC_GPIOB_CLK_DISABLE()
//spi1 MENU_board 1 cs引脚
#define SPI1_MENUB_CS                        GPIO_PIN_7
#define SPI1_MENUB_CS_Port                   GPIOB
#define SPI1_MENUB_CS_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1_MENUB_CS_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#define SPI1_CS_ENABLE(__INDEX__)       do{if((__INDEX__) == DI_Board_1)    HAL_GPIO_WritePin(SPI1_DIB1_CS_Port,SPI1_DIB1_CS,GPIO_PIN_RESET); else \
                                           if((__INDEX__) == DI_Board_2)    HAL_GPIO_WritePin(SPI1_DIB2_CS_Port,SPI1_DIB2_CS,GPIO_PIN_RESET); else \
                                           if((__INDEX__) == DI_Board_3)    HAL_GPIO_WritePin(SPI1_DIB3_CS_Port,SPI1_DIB3_CS,GPIO_PIN_RESET); else \
                                           if((__INDEX__) == DI_Board_4)    HAL_GPIO_WritePin(SPI1_DIB4_CS_Port,SPI1_DIB4_CS,GPIO_PIN_RESET); else \
                                           if((__INDEX__) == DQ_Board_1)    HAL_GPIO_WritePin(SPI1_DQB1_CS_Port,SPI1_DQB1_CS,GPIO_PIN_RESET); else \
                                           if((__INDEX__) == DQ_Board_2)    HAL_GPIO_WritePin(SPI1_DQB2_CS_Port,SPI1_DQB2_CS,GPIO_PIN_RESET); else \
                                           if((__INDEX__) == RS485_Board)   HAL_GPIO_WritePin(SPI1_RS485B_CS_Port,SPI1_RS485B_CS,GPIO_PIN_RESET); else \
                                           if((__INDEX__) == MENU_Board)    HAL_GPIO_WritePin(SPI1_MENUB_CS_Port,SPI1_MENUB_CS,GPIO_PIN_RESET); \
                                        }while(0)
#define SPI1_CS_DISABLE(__INDEX__)      do{if((__INDEX__) == DI_Board_1)    HAL_GPIO_WritePin(SPI1_DIB1_CS_Port,SPI1_DIB1_CS,GPIO_PIN_SET); else \
                                           if((__INDEX__) == DI_Board_2)    HAL_GPIO_WritePin(SPI1_DIB2_CS_Port,SPI1_DIB2_CS,GPIO_PIN_SET); else \
                                           if((__INDEX__) == DI_Board_3)    HAL_GPIO_WritePin(SPI1_DIB3_CS_Port,SPI1_DIB3_CS,GPIO_PIN_SET); else \
                                           if((__INDEX__) == DI_Board_4)    HAL_GPIO_WritePin(SPI1_DIB4_CS_Port,SPI1_DIB4_CS,GPIO_PIN_SET); else \
                                           if((__INDEX__) == DQ_Board_1)    HAL_GPIO_WritePin(SPI1_DQB1_CS_Port,SPI1_DQB1_CS,GPIO_PIN_SET); else \
                                           if((__INDEX__) == DQ_Board_2)    HAL_GPIO_WritePin(SPI1_DQB2_CS_Port,SPI1_DQB2_CS,GPIO_PIN_SET); else \
                                           if((__INDEX__) == RS485_Board)   HAL_GPIO_WritePin(SPI1_RS485B_CS_Port,SPI1_RS485B_CS,GPIO_PIN_SET); else \
                                           if((__INDEX__) == MENU_Board)    HAL_GPIO_WritePin(SPI1_MENUB_CS_Port,SPI1_MENUB_CS,GPIO_PIN_SET); \
                                        }while(0)

#define SlaveBoard_Max                  8       //定义板子的数量 di板4块，dq板2块，rs485板1块，menu板1块
#define DIBoard_NO                      4       //定义DI板子的数量 4块
#define sTrans_TimeOut                  100       //5 = 5ms
#define sTxRx_TimeOut                   1
#define spiRxDataLen                    100

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

typedef enum 
{
//  NO_Board      = 0,
  DI_Board_1    = 0,     //= 0b00000001,
  DI_Board_2    ,     //= 0b00000010,	//2,
  DI_Board_3    ,     //= 0b00000100,
  DI_Board_4    ,     //= 0b00001000,
  DQ_Board_1    ,     //= 0b00010000,
  DQ_Board_2    ,     //= 0b00100000,
  RS485_Board   ,     //= 0b01000000,	//64,
  MENU_Board    ,     //= 0b10000000,	//128     
}BoardID_TypeDef;


typedef enum 
{
    Read_RegData        = 0x11,
    Read_CurrentData    = 0x22,
    Config_Para         = 0xCE,  
}slaveBoardCmdCode_TypeDef;

typedef enum 
{
    Read_DI0_DI7        = 0x01,
    Read_DI8_DI15       = 0x02,
    Read_DI16_DI23      = 0x03,
    Read_DI24_DI31      = 0x04,

}slaveBoardRegAddr_TypeDef;


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

/*  DI Board cmd
 *  cmd 由5个字节组成：boardID + CmdCode + RegAddr + para1 + para2
 *                             |   |
 *            0x11：读取的数据 __|   |
 *            0x22：配置的参数 ______|
 *  byte1 byte2 byte3 byte4 byte5
*/
//activeBoard_TypeDef             boardID;
//slaveBoardCmdCode_TypeDef       sBoardCmdCode;
//slaveBoardRegAddr_TypeDef       sBoardRegAddr;
//slaveBoardPara1_TypeDef         sBoardPara_1;
//slaveBoardPara2_TypeDef         sBoardPara_2;       //备用


#ifdef __cplusplus
}
#endif

#endif
