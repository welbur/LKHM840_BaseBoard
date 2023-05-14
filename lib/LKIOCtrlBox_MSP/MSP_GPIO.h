#ifndef _MSP_GPIO_H_
#define	_MSP_GPIO_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PinConfig.h"
//#include "SPITransfer_C.h"
#include <stdio.h>
#include "LOG.h"


#if defined(DEVBoard) || defined(DEVBoardYD) 
#define LED_R(n)			(n?HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET))
#define LED_R_TogglePin		    HAL_GPIO_TogglePin(RED_GPIO_Port,RED_Pin)	//LED_R 闪烁

#define LED_G(n)			(n?HAL_GPIO_WritePin(GREEN_GPIO_Port,GREEN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(GREEN_GPIO_Port,GREEN_Pin,GPIO_PIN_RESET))
#define LED_G_TogglePin       HAL_GPIO_TogglePin(GREEN_GPIO_Port,GREEN_Pin)	//LED_G 闪烁
#endif

#define WorkLed(n)			(n?HAL_GPIO_WritePin(WorkLed_GPIO_Port,WorkLed_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(WorkLed_GPIO_Port,WorkLed_Pin,GPIO_PIN_RESET))
#define WorkLed_TogglePin     HAL_GPIO_TogglePin(WorkLed_GPIO_Port,WorkLed_Pin)	//WorkLed 闪烁 



void MX_GPIO_Init(void);

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void EXTILine_Config(void);

#ifdef __cplusplus
}
#endif

#endif
