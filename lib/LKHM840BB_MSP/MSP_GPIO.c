/**
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  * @file       BSP_GPIO.c/h baseboard
  * @brief      GPIO的二次封装
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V3.0.0     2020.7.14     	              	1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  */
#include "MSP_GPIO.h"

// uint8_t DefaultBoardID = 0;
HAL_TickFreqTypeDef TickFreq = HAL_TICK_FREQ_DEFAULT; /* 1KHz */

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 */


void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

#if defined(DEVBoard) || defined(DEVBoardYD)
	/*Configure GPIO pin : 设置按键 */
	GPIO_InitStruct.Pin = KEY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins :设置led */
	GPIO_InitStruct.Pin = RED_Pin | GREEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RED_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
#endif
	/*Configure GPIO pins :设置Workled */
	GPIO_InitStruct.Pin = WorkLed_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(WorkLed_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(WorkLed_GPIO_Port, WorkLed_Pin, GPIO_PIN_RESET);

#ifdef LKHM840PowerB
	/*Configure GPIO pins :设置powerboard to baseboard int引脚 */
	GPIO_InitStruct.Pin = PB2BB_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PB2BB_INT_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(PB2BB_INT_GPIO_Port, PB2BB_INT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : 设置boardID的配置引脚 pc3 pc4 */
	GPIO_InitStruct.Pin = bID0_Pin | bID1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(bID_GPIO_Port, &GPIO_InitStruct);

  	/*Configure spi2 cs pins : PB12 */
  	SPI2_CS_CLK_ENABLE();
  	GPIO_InitStruct.Pin = SPI2_CS;                                   
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  	GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(SPI2_CS_Port, &GPIO_InitStruct);
#else
	/*Configure GPIO pins :设置baseboard to powerboard rst引脚 用于复位powerboard */
	GPIO_InitStruct.Pin = BaseBtoPowerB_OUT1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BaseBtoPowerB_OUT1_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = BaseBtoPowerB_OUT2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BaseBtoPowerB_OUT2_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = BaseBtoPowerB_OUT3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BaseBtoPowerB_OUT3_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = BaseBtoPowerB_OUT4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BaseBtoPowerB_OUT4_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(BaseBtoPowerB_OUT1_PORT, BaseBtoPowerB_OUT1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BaseBtoPowerB_OUT2_PORT, BaseBtoPowerB_OUT2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BaseBtoPowerB_OUT3_PORT, BaseBtoPowerB_OUT3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BaseBtoPowerB_OUT4_PORT, BaseBtoPowerB_OUT4, GPIO_PIN_SET);
#endif
}

/**
 * @brief  Configures EXTI Line in interrupt mode
 * @param  None
 * @retval None
 */
void EXTILine_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

// 设置按键的中断
#if defined(DEVBoard) || defined(DEVBoardYD)
	KEY_Pin_CLK_ENABLE();
	GPIO_InitStruct.Pin = KEY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(KEY_GPIO_Port, KEY_Pin, 1);
	HAL_NVIC_SetPriority(KEY_Pin_EXTI_IRQn, 2, 10);
	HAL_NVIC_EnableIRQ(KEY_Pin_EXTI_IRQn);
#endif

#ifdef LKHM840BaseB
	/*Configure GPIO pins : PC6 PC5 PC4 PC3
	 *  PC6～PC3定义为Slave板到Master板的中断信号
	 */
	/********************************     power board 1 int     *********************************/
	PowerBtoBaseB_INT1_CLK_ENABLE();							
	GPIO_InitStruct.Pin = PowerBtoBaseB_INT1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // GPIO_MODE_IT_RISING_FALLING; //GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PowerBtoBaseB_INT1_PORT, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(PowerBtoBaseB_INT1_EXTI_IRQn, GPIO_EXTI_PP, PowerBtoBaseB_INT1_EXTI_SP);	
	HAL_NVIC_EnableIRQ(PowerBtoBaseB_INT1_EXTI_IRQn);
	/********************************     power board 2 int     *********************************/
	PowerBtoBaseB_INT2_CLK_ENABLE();							
	GPIO_InitStruct.Pin = PowerBtoBaseB_INT2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PowerBtoBaseB_INT2_PORT, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(PowerBtoBaseB_INT2_EXTI_IRQn, GPIO_EXTI_PP, PowerBtoBaseB_INT2_EXTI_SP);
	HAL_NVIC_EnableIRQ(PowerBtoBaseB_INT2_EXTI_IRQn);
	/********************************     power board 3 int     *********************************/
	PowerBtoBaseB_INT3_CLK_ENABLE();							
	GPIO_InitStruct.Pin = PowerBtoBaseB_INT3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PowerBtoBaseB_INT3_PORT, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(PowerBtoBaseB_INT3_EXTI_IRQn, GPIO_EXTI_PP, PowerBtoBaseB_INT3_EXTI_SP);
	HAL_NVIC_EnableIRQ(PowerBtoBaseB_INT3_EXTI_IRQn);
	/********************************     power board 4 int     *********************************/
#ifndef DEVBoardYD
	PowerBtoBaseB_INT4_CLK_ENABLE();						
	GPIO_InitStruct.Pin = PowerBtoBaseB_INT4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PowerBtoBaseB_INT4_PORT, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(PowerBtoBaseB_INT4_EXTI_IRQn, GPIO_EXTI_PP, PowerBtoBaseB_INT4_EXTI_SP);
	HAL_NVIC_EnableIRQ(PowerBtoBaseB_INT4_EXTI_IRQn);
#endif
#endif
}

// 配置文件在.h文件中
