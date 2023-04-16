/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "SEGGER_RTT.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//uint8_t USART_DMA_TX_OVER = 1;
#ifdef UartPrintf
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif


/*modbus相关参数*/
modbusHandler_t ModbusH;
uint16_t ModbusDATA[ModbusDATASize], ModbusDATA_Cache[ModbusDATASize];

/*Slave Board相关参数*/
SlaveBoardHandler_t D_I_1_BoardH, D_I_2_BoardH, D_I_3_BoardH, D_I_4_BoardH, D_Q_1_BoardH, D_Q_2_BoardH, MENU_BoardH, RS485_BoardH;
uint8_t DI1_DATA[128], DI2_DATA[128], DI3_DATA[128], DI4_DATA[128], DQ1_DATA[128], DQ2_DATA[128], MENU_DATA[128], RS485_DATA[128];
SlaveBoardHandler_t SlaveBoardH[8];
/* Private function prototypes -----------------------------------------------*/


void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void DI_Board_Init(void);
/* Private user code ---------------------------------------------------------*/
/**/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();

  EXTILine_Config();
  SPITransfer_Init();

#if 1
	/* Modbus 从站初始化Slave initialization */
	ModbusH.uModbusType = MB_SLAVE;
	ModbusH.port = &huart2;
	ModbusH.u8id = 1; // slave ID,  For master it must be 0
	ModbusH.u16timeOut = 1000;
	ModbusH.EN_Port = NULL; // No RS485   //ModbusH.EN_Port = NULL;
	ModbusH.EN_Pin = 0;
	// ModbusH2.EN_Port = LD2_GPIO_Port; // RS485 Enable
	// ModbusH2.EN_Pin = LD2_Pin; // RS485 Enable
	ModbusH.u16regs = ModbusDATA;
	ModbusH.u16regsize = sizeof(ModbusDATA) / sizeof(ModbusDATA[0]);
	ModbusH.xTypeHW = USART_HW_DMA;
#endif
	// Initialize Modbus library	
  ModbusInit(&ModbusH);
	// Start capturing traffic on serial Port
	ModbusStart(&ModbusH);
	/***********/
  uint8_t tt = 10;
  LOGE("start Modbud......%d......%d\r\n", tt, tt);
  LOGI("start Modbud\r\n");
  LOGW("start Modbud\r\n");

  /*Slave板状态 初始化*/
  D_I_1_BoardH.BoardID = DI_Board_1;
  D_I_1_BoardH.isBoard_Rx_En = 0;
  D_I_1_BoardH.spiRx_uartTx_u8regs = DI1_DATA;
  D_I_1_BoardH.spiRx_uartTx_u8regs_size = sizeof(DI1_DATA) / sizeof(DI1_DATA[0]);
  D_I_1_BoardH.spiTransState = SpiTrans_Wait;
  SlaveBoardH[DI_Board_1] = D_I_1_BoardH;

  D_I_2_BoardH.BoardID = DI_Board_2;
  D_I_2_BoardH.isBoard_Rx_En = 0;
  D_I_2_BoardH.spiRx_uartTx_u8regs = DI2_DATA;
  D_I_2_BoardH.spiRx_uartTx_u8regs_size = sizeof(DI2_DATA) / sizeof(DI2_DATA[0]);
  D_I_2_BoardH.spiTransState = SpiTrans_Wait;
  SlaveBoardH[DI_Board_2] = D_I_2_BoardH;

  D_Q_1_BoardH.BoardID = DQ_Board_1;
  D_Q_1_BoardH.isBoard_Rx_En = 0;
  D_Q_1_BoardH.spiRx_uartTx_u8regs = DQ1_DATA;  //spiTx_uartRx_u8regs
  D_Q_1_BoardH.spiRx_uartTx_u8regs_size = sizeof(DQ1_DATA) / sizeof(DQ1_DATA[0]);
  D_Q_1_BoardH.spiTransState = SpiTrans_Wait;
  SlaveBoardH[DQ_Board_1] = D_Q_1_BoardH;

  D_Q_2_BoardH.BoardID = DQ_Board_2;
  D_Q_2_BoardH.isBoard_Rx_En = 0;
  D_Q_2_BoardH.spiRx_uartTx_u8regs = DQ2_DATA;  //spiTx_uartRx_u8regs
  D_Q_2_BoardH.spiRx_uartTx_u8regs_size = sizeof(DQ2_DATA) / sizeof(DQ2_DATA[0]);
  D_Q_2_BoardH.spiTransState = SpiTrans_Wait;
  SlaveBoardH[DQ_Board_2] = D_Q_2_BoardH;
  //SlaveBoardStatus.sTransState = SpiTrans_Wait;

  /* Infinite loop */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
//  DI_Board_Init();
  /* Start scheduler */
  osKernelStart();

  

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
#if 1
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

#ifdef UartPrintf
PUTCHAR_PROTOTYPE
{
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  //HAL_UART_Transmit_IT(&huart1, (uint8_t *)&ch, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 2);

  return ch;
}
#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
uint8_t ledg_v = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) 
  {
    case KEY_Pin:
      SlaveBoardH[DI_Board_1].isBoard_Rx_En = 1;
//      SlaveBoardH[DI_Board_2].isBoardEnable = 1;
//      SlaveBoardH[DI_Board_3].isBoardEnable = 1;
//      SlaveBoardH[DI_Board_4].isBoardEnable = 1;
//      SlaveBoardH[DQ_Board_1].isBoardEnable = 1;
      LOGI("DEV button........\r\n");
      ledg_v = 1 - ledg_v;
      LED_R(ledg_v);
      break;
#if 1 //DEVBoard
    case DIB_INT_PIN1:
      SlaveBoardH[DI_Board_1].isBoard_Rx_En = 1;//D_I_1_BoardH.isBoardEnable = 1;
      break;
#endif
    case DIB_INT_PIN2:
      SlaveBoardH[DI_Board_2].isBoard_Rx_En = 1;
      break;
    case DQB_INT_PIN1:
      SlaveBoardH[DQ_Board_1].isBoard_Rx_En = 1;
      //LOGI("DQ board 1 int pin........%d\r\n", SlaveBoardStatus.activeBoard);
      break;
    default:
      LOGE("int gpio pin not found!");
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
