/**
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  * @file       BSP_GPIO.c/h
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

//uint8_t DefaultBoardID = 0;
HAL_TickFreqTypeDef TickFreq = HAL_TICK_FREQ_DEFAULT;  /* 1KHz */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
#if 1   //(DEVBoard | DEVBoardYD)
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level 设置LED*/ 
  HAL_GPIO_WritePin(GPIOC, RED_Pin|GREEN_Pin|BLUE_Pin, GPIO_PIN_SET);
#if DEVBoard
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; //GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = RED_Pin|GREEN_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#endif

#if DEVBoardYD
  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = RED_Pin|GREEN_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_GPIO_Port, &GPIO_InitStruct);
#endif
}
#endif

/**
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void EXTILine_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

#if DEVBoardYD
    KEY_Pin_CLK_ENABLE();
    GPIO_InitStruct.Pin = KEY_Pin;                                   
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(KEY_GPIO_Port, KEY_Pin, 1);
    HAL_NVIC_SetPriority(KEY_Pin_EXTI_IRQn, 2, 10);
    HAL_NVIC_EnableIRQ(KEY_Pin_EXTI_IRQn);
#endif
    /*Configure GPIO pins : PC3 PC4 PC5 PC6 PC7 PC8 PC9 PC10
     *  PC3～PC10定义为Slave板到Master板的中断信号 
    */
    /********************************     di board 1 int     *********************************/
#if 1
    DIB_INT_PIN1_CLK_ENABLE();
    GPIO_InitStruct.Pin = DIB_INT_PIN1;                                   
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;//GPIO_MODE_IT_RISING_FALLING; //GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIB_INT_PIN1_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(DIB_INT_PIN1_EXTI_IRQn, GPIO_EXTI_PP, DIB_INT_PIN1_EXTI_SP);
    HAL_NVIC_EnableIRQ(DIB_INT_PIN1_EXTI_IRQn);
#endif
    /********************************     di board 2 int     *********************************/
    DIB_INT_PIN2_CLK_ENABLE();
    GPIO_InitStruct.Pin = DIB_INT_PIN2;                                   
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIB_INT_PIN2_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(DIB_INT_PIN2_EXTI_IRQn, GPIO_EXTI_PP, DIB_INT_PIN2_EXTI_SP);
    HAL_NVIC_EnableIRQ(DIB_INT_PIN2_EXTI_IRQn);
    /********************************     di board 3 int     *********************************/
    DIB_INT_PIN3_CLK_ENABLE();
    GPIO_InitStruct.Pin = DIB_INT_PIN3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIB_INT_PIN3_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(DIB_INT_PIN3_EXTI_IRQn, GPIO_EXTI_PP, DIB_INT_PIN3_EXTI_SP);
    HAL_NVIC_EnableIRQ(DIB_INT_PIN3_EXTI_IRQn);
    /********************************     di board 4 int     *********************************/
    DIB_INT_PIN4_CLK_ENABLE();
    GPIO_InitStruct.Pin = DIB_INT_PIN4;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIB_INT_PIN4_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(DIB_INT_PIN4_EXTI_IRQn, GPIO_EXTI_PP, DIB_INT_PIN4_EXTI_SP);
    HAL_NVIC_EnableIRQ(DIB_INT_PIN4_EXTI_IRQn);
    /********************************     dq board 1 int     *********************************/
    DQB_INT_PIN1_CLK_ENABLE();
    GPIO_InitStruct.Pin = DQB_INT_PIN1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DQB_INT_PIN1_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(DQB_INT_PIN1_EXTI_IRQn, GPIO_EXTI_PP, DQB_INT_PIN1_EXTI_SP);
    HAL_NVIC_EnableIRQ(DQB_INT_PIN1_EXTI_IRQn);
    /********************************     dq board 2 int     *********************************/
    DQB_INT_PIN2_CLK_ENABLE();
    GPIO_InitStruct.Pin = DQB_INT_PIN2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DQB_INT_PIN2_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(DQB_INT_PIN2_EXTI_IRQn, GPIO_EXTI_PP, DQB_INT_PIN2_EXTI_SP);
    HAL_NVIC_EnableIRQ(DQB_INT_PIN2_EXTI_IRQn);
    /********************************     rs485 board int     *********************************/
    RS485B_INT_PIN_CLK_ENABLE();
    GPIO_InitStruct.Pin = RS485B_INT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(RS485B_INT_PIN_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(RS485B_INT_PIN_EXTI_IRQn, GPIO_EXTI_PP, RS485B_INT_PIN_EXTI_SP);
    HAL_NVIC_EnableIRQ(RS485B_INT_PIN_EXTI_IRQn);
    /********************************     menu board int     *********************************/
    MENUB_INT_PIN_CLK_ENABLE();
    GPIO_InitStruct.Pin = MENUB_INT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MENUB_INT_PIN_PORT, &GPIO_InitStruct);
    /* Enable and set EXTI Line0 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(MENUB_INT_PIN_EXTI_IRQn, GPIO_EXTI_PP, MENUB_INT_PIN_EXTI_SP);
    HAL_NVIC_EnableIRQ(MENUB_INT_PIN_EXTI_IRQn);
}


//配置文件在.h文件中


