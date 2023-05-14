
#ifndef _PINCONFIG_H_
#define	_PINCONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
//#include "FreeRTOS.h"
//#include "task.h"
#include <string.h>


#ifdef DEVBoard                       //开发板上的按键和led引脚定义
#define KEY_Pin                         GPIO_PIN_13
#define KEY_GPIO_Port                   GPIOC
#define KEY_Pin_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define KEY_Pin_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define KEY_Pin_EXTI_IRQn               EXTI0_IRQn

#define RED_Pin                         GPIO_PIN_0
#define RED_GPIO_Port                   GPIOC
#define GREEN_Pin                       GPIO_PIN_1
#define GREEN_GPIO_Port                 GPIOC
#define BLUE_Pin                        GPIO_PIN_2
#define BLUE_GPIO_Port                  GPIOC

#define WorkLed_Pin                     BLUE_Pin
#define WorkLed_GPIO_Port               BLUE_GPIO_Port
#elif defined(DEVBoardYD)             //如果使用源地开发板
#define KEY_Pin                         GPIO_PIN_3
#define KEY_GPIO_Port                   GPIOB
#define KEY_Pin_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define KEY_Pin_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
#define KEY_Pin_EXTI_IRQn               EXTI3_IRQn

#define RED_Pin                         GPIO_PIN_13
#define RED_GPIO_Port                   GPIOA
#define GREEN_Pin                       GPIO_PIN_14
#define GREEN_GPIO_Port                 GPIOA
#define BLUE_Pin                        GPIO_PIN_15
#define BLUE_GPIO_Port                  GPIOA

#define WorkLed_Pin                     BLUE_Pin
#define WorkLed_GPIO_Port               BLUE_GPIO_Port
#else                             //实际使用的板子
#define WorkLed_Pin                     GPIO_PIN_0
#define WorkLed_GPIO_Port               GPIOC
#endif

/**
 * @brief 定义第一块Power Board的触发信号 
 * @brief Power Board 1 --> Master Board
 */
#define PowerB_INT1                         GPIO_PIN_6
#define PowerB_INT1_PORT                    GPIOC
#define PowerB_INT1_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define PowerB_INT1_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define PowerB_INT1_EXTI_IRQn               EXTI9_5_IRQn         
#define PowerB_INT1_EXTI_SP                 1    
/**
 * @brief 定义第二块Power Board的触发信号 
 * @brief Power Board 2 --> Master Board
 */
#define PowerB_INT2                         GPIO_PIN_5
#define PowerB_INT2_PORT                    GPIOC
#define PowerB_INT2_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define PowerB_INT2_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define PowerB_INT2_EXTI_IRQn               EXTI9_5_IRQn          
#define PowerB_INT2_EXTI_SP                 2      
/**
 * @brief 定义第三块Power Board的触发信号 
 * @brief Power Board 3 --> Master Board
 */
#define PowerB_INT3                         GPIO_PIN_4
#define PowerB_INT3_PORT                    GPIOC
#define PowerB_INT3_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define PowerB_INT3_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define PowerB_INT3_EXTI_IRQn               EXTI4_IRQn         
#define PowerB_INT3_EXTI_SP                 3        
/**
 * @brief 定义第四块Power Board的触发信号 
 * @brief Power Board 4 --> Master Board
 */
#define PowerB_INT4                         GPIO_PIN_3
#define PowerB_INT4_PORT                    GPIOC
#define PowerB_INT4_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define PowerB_INT4_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define PowerB_INT4_EXTI_IRQn               EXTI3_IRQn   
#define PowerB_INT4_EXTI_SP                 4   

#define GPIO_EXTI_PP              2       //定义GPIO中断的主优先级

/**
 * @brief 定义第一块Power Board的片选cs信号 
 * @brief Power Board 1 <-- Master Board
 */
#define PowerB_SPI1_CS1                         GPIO_PIN_4
#define PowerB_SPI1_CS1_Port                    GPIOB
#define PowerB_SPI1_CS1_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define PowerB_SPI1_CS1_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
/**
 * @brief 定义第二块Power Board的片选cs信号 
 * @brief Power Board 2 <-- Master Board
 */
#define PowerB_SPI1_CS2                         GPIO_PIN_5
#define PowerB_SPI1_CS2_Port                    GPIOB
#define PowerB_SPI1_CS2_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define PowerB_SPI1_CS2_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
/**
 * @brief 定义第三块Power Board的片选cs信号 
 * @brief Power Board 3 <-- Master Board
 */
#define PowerB_SPI1_CS3                         GPIO_PIN_6
#define PowerB_SPI1_CS3_Port                    GPIOB
#define PowerB_SPI1_CS3_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define PowerB_SPI1_CS3_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
/**
 * @brief 定义第四块Power Board的片选cs信号 
 * @brief Power Board 4 <-- Master Board
 */
#define PowerB_SPI1_CS4                         GPIO_PIN_7
#define PowerB_SPI1_CS4_Port                    GPIOB
#define PowerB_SPI1_CS4_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define PowerB_SPI1_CS4_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()

/**
 * @brief SPI1 的GPIO 引脚 
 *	PA5     ------> SPI1_SCK
 *	PA6     ------> SPI1_MISO
 *	PA7     ------> SPI1_MOSI
 */
#define SPI1_SCK_PIN                     GPIO_PIN_5
#define SPI1_MISO_PIN                    GPIO_PIN_6
#define SPI1_MOSI_PIN                    GPIO_PIN_7
#define SPI1_GPIO_PORT                   GPIOA
#define SPI1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPI1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

/**
 * @brief SPI2 的GPIO 引脚 
 *  PB13     ------> SPI2_SCK
 *	PB14     ------> SPI2_MISO
 *	PB15     ------> SPI2_MOSI
 */
#define SPI2_SCK_PIN                     GPIO_PIN_13
#define SPI2_MISO_PIN                    GPIO_PIN_14
#define SPI2_MOSI_PIN                    GPIO_PIN_15
#define SPI2_GPIO_PORT                   GPIOB
#define SPI2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

/**
 * @brief usart1 的GPIO 引脚 
 * PA9      ------> USART1_TX
 * PA10     ------> USART1_RX
 */
#define USART1_TX_PIN                    GPIO_PIN_9
#define USART1_RX_PIN                    GPIO_PIN_10
#define USART1_GPIO_PORT                 GPIOA
#define USART1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

/**
 * @brief usart2 的GPIO 引脚 
 * PA2     ------> USART2_TX
 * PA3     ------> USART2_RX
 */
#define USART2_TX_PIN                    GPIO_PIN_2
#define USART2_RX_PIN                    GPIO_PIN_3
#define USART2_GPIO_PORT                 GPIOA
#define USART2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

/**
 * @brief usart3 的GPIO 引脚 
 * PC10     ------> USART3_TX
 * PC11     ------> USART3_RX
 */
#define USART3_TX_PIN                    GPIO_PIN_10
#define USART3_RX_PIN                    GPIO_PIN_11
#define USART3_GPIO_PORT                 GPIOC
#define USART3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()


#ifdef __cplusplus
}
#endif

#endif
