#ifndef _MSP_USART_H
#define _MSP_USART_H

#ifdef __cplusplus
 extern "C" {
#endif

//baseboard

#include "stm32f4xx_hal.h"
//#include "main.h"
#include "PinConfig.h"
#include "LOG.h"
#include <string.h>
#include <stdlib.h>
//#include "stdio.h"	
//#include "../../include/usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif