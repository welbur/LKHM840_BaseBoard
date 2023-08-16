/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "BackPanelTrans.h"
#include "Modbus.h"
#include "LOG.h"

#include "CO_app_STM32.h"
#include "can.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define lowByte(w) ((w)&0xff)
#define highByte(w) ((w) >> 8)
#define getSlaveBoardInfo_TimeOut 3000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* 定义CANopenNode数据 */
#define COB_ID 0x600
#define NODE_ID 0x01
#define DATA_LENGTH 8

/* 定义要发送的数据 */
uint8_t candata[DATA_LENGTH] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

CANopenNodeSTM32 canOpenNodeSTM32;
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// #define MSG_COUNT 3
// #define MSG_LENGTH 128

// char LOG_MSG[MSG_LENGTH];
// SPITransHandler_t SPITransHandler;

uint8_t _sboardFlag = 0x00; // 用于判断有哪几块板子
uint8_t _backpanelDataLen = 0;

osMutexId_t mutex;
// Semaphore for backpanel data
osSemaphoreId_t BackPanelSemaphore; // 用于背板协议数据的信号量

// 事件标志ID
osEventFlagsId_t getSlaveBoardInfo_flagsID;

uint32_t RPBD_All_Countt = 0;
uint32_t RPBD_Act_Countt = 0;
uint8_t mod_preamble[MOD_PREAMBLE_SIZE] = {MOD_START_BYTE, 0, 0, 0};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name = "defaultTask",
	.stack_size = 32 * 4,
	.priority = (osPriority_t)osPriorityLow,
};

/* Definitions for canopen */
osThreadId_t CANopen_TaskHandle;
const osThreadAttr_t CANopenTask_attributes = {
	.name = "CANopenTask",
	.priority = (osPriority_t)osPriorityHigh,
	.stack_size = 128 * 4};

osThreadId_t osPrintLOG_TaskHandle;
const osThreadAttr_t osPrintLOGTask_attributes = {
	.name = "osPrintLOGTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityLow1,
};

#if 0
osThreadId_t getSlaveBoardInfo_TaskHandle;
const osThreadAttr_t getSlaveBoardInfoTask_attributes = {
	.name = "getSlaveBoardInfoTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};

/* 1-----Main Board read power board data ,and saved to buffer, wait plc read */
osThreadId_t ReadPowerBoardData_TransTaskHandle;
const osThreadAttr_t ReadPowerBoardData_TransTask_attributes = {
	.name = "ReadPowerBoardData_TransTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityBelowNormal1, // osPriorityHigh
};
/* 2-----Main Board read modbus data from pn board, and write to Power Board TransTask */
osThreadId_t MainBoard2PowerBoard_TransTaskHandle;
const osThreadAttr_t MainBoard2PowerBoard_TransTask_attributes = {
	.name = "MainBoard2PowerBoard_TransTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityBelowNormal, // osPriorityHigh
};
#endif

// Semaphore to access the spi handler
const osSemaphoreAttr_t BackPanelSphr_attributes = {
	.name = "BackPanelSphr"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void CANopenTask_Init(void);
void BackPanelTransTask_Init(void);

void StartDefaultTask(void *argument);
void osPrintLOG(void *argument);
void getSlaveBoardInfo(void *argument);
void CANopen_task(void *argument);

void Start_Master_BackPanelTrans_Task(void *argument);
void Start_MainBoard2PowerBoard_TransTask(void *argument);
void waitBackPanelTaskDone(void);

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	mutex = osMutexNew(NULL);
	// 创建事件标志
	// getSlaveBoardInfo_flagsID = osEventFlagsNew(NULL);
	//getSlaveBoardInfo_TaskHandle = osThreadNew(getSlaveBoardInfo, NULL, &getSlaveBoardInfoTask_attributes);
	LOG("freerots init ok \r\n");
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
//	CANopen_TaskHandle = osThreadNew(CANopen_task, NULL, &CANopenTask_attributes);
	// osPrintLOG_TaskHandle = osThreadNew(osPrintLOG, NULL, &osPrintLOGTask_attributes);
}

void CANopenTask_Init(void)
{
	//CANopenNodeSTM32 canOpenNodeSTM32;
	canOpenNodeSTM32.CANHandle = &hcan1;
	canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
	canOpenNodeSTM32.timerHandle = &htim14;
	canOpenNodeSTM32.desiredNodeID = 21;
	canOpenNodeSTM32.baudrate = 500;
	//HAL_TIM_Base_Start_IT(&htim14);
	//canopen_app_init(&canOpenNodeSTM32);
	HAL_TIM_Base_Start_IT(&htim14);
	LOG("CANopenSTM32 Init OK! \r\n");
	//CANopen_TaskHandle = osThreadNew(CANopen_task, NULL, &CANopenTask_attributes);
}



/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartDefaultTask(void *argument)
{
	// while((osEventFlagsWait(getSlaveBoardInfo_flagsID, 0x0001, osFlagsWaitAny, osWaitForever) && 0x0001) == 0)
	// LOG("start default task \r\n");
	/* Infinite loop */
	for (;;)
	{
		//osMutexAcquire(mutex, osWaitForever); // 打印调试信息用

		LED_R_TogglePin;
		//LOG("default task \r\n");
		//osMutexRelease(mutex);
		osDelay(1000);
	}
}


/**
 * @brief Function implementing the canopen thread.
 * @param argument: Not used
 * @retval None
 */
void CANopen_task(void *argument)
{
	uint32_t msTickstart = HAL_GetTick();
	
	/* Infinite loop */
	for (;;)
	{
		osMutexAcquire(mutex, osWaitForever);
		canopen_app_process();
		// Sleep for 1ms, you can decrease it if required, in the canopen_app_process we will double check to make sure 1ms passed
		//vTaskDelay(pdMS_TO_TICKS(1));
		
		if (HAL_GetTick() - msTickstart > 5000)
		{
			LOG("can open task \r\n");
			msTickstart = HAL_GetTick();
		}
			
		osMutexRelease(mutex);
		osDelay(1);
	}
}

