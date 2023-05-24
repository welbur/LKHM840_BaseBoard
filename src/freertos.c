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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//#define MSG_COUNT 3
//#define MSG_LENGTH 128

//char LOG_MSG[MSG_LENGTH];
//SPITransHandler_t SPITransHandler;

osMutexId_t mutex;


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

osThreadId_t osPrintLOG_TaskHandle;
const osThreadAttr_t osPrintLOGTask_attributes = {
	.name = "osPrintLOGTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t)osPriorityLow1,
};

/* 1-----Main Board read power board data ,and saved to buffer, wait plc read */
osThreadId_t ReadPowerBoardData_TransTaskHandle;
const osThreadAttr_t ReadPowerBoardData_TransTask_attributes = {
	.name = "ReadPowerBoardData_TransTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t)osPriorityBelowNormal1, // osPriorityHigh
};
/* 2-----Main Board read modbus data from pn board, and write to Power Board TransTask */
osThreadId_t MainBoard2PowerBoard_TransTaskHandle;
const osThreadAttr_t MainBoard2PowerBoard_TransTask_attributes = {
	.name = "MainBoard2PowerBoard_TransTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t)osPriorityBelowNormal, // osPriorityHigh
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void osPrintLOG(void *argument);
void Start_ReadPowerBoardData_TransTask(void *argument);
void Start_MainBoard2PowerBoard_TransTask(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void BackPanelTrans_Init(void);

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	mutex = osMutexNew(NULL);

	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	osPrintLOG_TaskHandle = osThreadNew(osPrintLOG, NULL, &osPrintLOGTask_attributes);
}

void BackPanelTrans_Init(void)
{
	BackPanelTransHandler_t BackPanelTransHandler;
	BackPanelTransHandler.printDebug 		= 1;
	BackPanelTransHandler.readACKTimeOut 	= 5;
	BackPanelTransHandler.readDataTimeOut 	= 50;
	BackPanelTransHandler.waitCSTimeOut 	= 5;
	PacketInit();
	CRC_Init(0x9B);
	BPTrans_Init(&BackPanelTransHandler);
#if 1
	ReadPowerBoardData_TransTaskHandle = osThreadNew(Start_ReadPowerBoardData_TransTask, NULL, &ReadPowerBoardData_TransTask_attributes);
	MainBoard2PowerBoard_TransTaskHandle = osThreadNew(Start_MainBoard2PowerBoard_TransTask, NULL, &MainBoard2PowerBoard_TransTask_attributes);
#endif
}

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartDefaultTask(void *argument)
{
	/* Infinite loop */
	for (;;)
	{
		osMutexAcquire(mutex, osWaitForever);	//打印调试信息用
		if (PowerBoard_Trig[PowerBoard_1])
		{
			// PowerBoard_Trig[PowerBoard_1] = 0;
			#if defined(DEVBoardYD) || defined(DEVBoard)
			LED_G_TogglePin;
			#endif
		}
		else
		{
			WorkLed_TogglePin;
			//strcat(LOG_MSG[Default_LOG], "test print time \r\n");		//LOGI("test print time\r\n");
		}
		osMutexRelease(mutex);
		osDelay(1000);
	}
}

/**
 * @brief  将需要打印的数据都合并到一个buffer里面，然后再一起打印出来。
 * @param  argument: Not used
 * @retval None
 */
void osPrintLOG(void *argument)
{
	for (;;)
	{
		osMutexAcquire(mutex, osWaitForever);
		if (LOG_MSG[0] != 0)
		{	
			//if (LOG_MSG >= MSG_LENGTH)
			LOG("~~~~~~~~~~RTOS Print LOG~~~~~~~~~~\r\n%s", LOG_MSG);
			LOG("\r\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
			memset(LOG_MSG, 0, MSG_LENGTH);
		}
		osMutexRelease(mutex);
		osDelay(10);
	}
}

/****************   1-----Main Board read power board data ,and saved to buffer, wait plc read   ********************
 * @brief 每隔指定时间读取各个power板的数据，并保存，等待plc过来读取.
 * @param argument: Not used
 * @retval None
 */
void Start_ReadPowerBoardData_TransTask(void *argument)
{
//	SPITransHandler_t SPITransHandler;
//	SPITransHandler.spiPort = &hspi1;
//	SPITransHandler. SPIMODE = SPIMasterMode;
//	SPITransHandler.readACKTimeOut = 10;
//	SPITransHandler.readDataTimeOut = 100;
//	SPITransHandler.SPITransDir = Master_readDataFrom_Slave;
//	SPITransHandler.readDataFromPowerBoard_u8regs = PowerBoard_DATA;
//	SPITransHandler.readDataFromPowerBoard_u8regs_size = sizeof(PowerBoard_DATA) / sizeof(PowerBoard_DATA[0]);

	/* Infinite loop */
	for (;;)
	{
		osMutexAcquire(mutex, osWaitForever);	//打印调试信息用
		//SPITransHandler.spiTransState = SpiTrans_Wait;
		uint8_t re_arr_size = MOD_PREAMBLE_SIZE; // 暂用数组前面的4个元素，作为包头使用
		uint8_t re_arr[128];

		/*1-------------------------有触发信号后，读取相关slave板的所有数据*/
		for (int i = 0; i < PowerBoardNum; i++)
		{
			if (PowerBoard_Trig[i])
			{
				uint16_t rDataLen;
				uint8_t *rData = NULL;
				LOG("start time : %ld\r\n", xTaskGetTickCount());
				BackPanelTransStatus_TypeDef bpstatus = bpTrans_Wait;
				//HAL_SPI_MspInit(&hspi1);
				bpstatus = BackPanelTrans_Master_readDataFrom_Slave(i, rData, &rDataLen);
				//HAL_SPI_MspDeInit(&hspi1);
				LOG("current board %d status : ..%d..\r\n", i, bpstatus);
				PowerBoard_Trig[i] = 0;		//PowerBoardH[i].isBoard_Rx_En = 0;
				if (bpstatus == bpTrans_OK)
				{
					/*从spi通道读到数据后，把slave板从1-8所有的数据都读出来后，合并在一起，然后再发给modbus主机(pn板)*/
					COPY_ARRAY(re_arr + re_arr_size, &rData, rDataLen);
					re_arr_size += rDataLen;
				}
			}
		}
		
		if (re_arr_size != MOD_PREAMBLE_SIZE)
		{	
			/*增加包头*/
			re_arr[0] = mod_preamble[0], re_arr[1] = mod_preamble[1];
			re_arr[2] = re_arr_size - MOD_PREAMBLE_SIZE; // 有效数据的长度
			re_arr[3] = MB_FC_READ_REGISTERS;			 // 功能指令码

			Addto_osPrintf("rec111 data : ");
			for (int j = 0; j < re_arr_size; j++)
				Addto_osPrintf("%02X ", re_arr[j]);	
			Addto_osPrintf("\r\n.......%d~~~ end time : %ld --------%ld\r\n", re_arr_size, xTaskGetTickCount(), ++RPBD_Act_Countt);

			/*放到modbus里面去发送数据*/
			ModbusH.spiRx_uartTx_u8regs = re_arr;
			ModbusH.spiRx_uartTx_u8regs_size = re_arr_size;
			//spiRxUartTxBuffer(&ModbusH);
			ModbusH.spiRx_uartTx_u8regs_size = 0;
		}
		osMutexRelease(mutex);
		osDelay(5);
	}
}

/****************   2-----Main Board read modbus data from pn board, and write to Power Board TransTask   ********************
 * @brief 等待pn板发过来的modbus指令，然后将数据发送给指定的Power板 
 * @brief modbus指令格式 ： __________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________	
 * 						  |			 			Power Board 1					   |							Power Board 2							|						Power Board 3					       |							Power Board 4							|	 
 * 						  |	pboardID	DataLen		Data(ChannelID+Name+Value)     |	pboardID	DataLen			Data(ChannelID+Name+Value)			|	pboardID		DataLen		Data(ChannelID+Name+Value)     |	pboardID		DataLen			Data(ChannelID+Name+Value) 		|
 * 						  |	   01  		   4		         01     AA  11 22 	   |	   02      	   8     	   01  AA  11 22  02  BB  11 22 	    |	   03      		   4		      01  AA   11 22 	       |	   04      		   8     	   01  AA  11 22  02  BB  11 22		|
 * 						  |________________________________________________________|____________________________________________________________________|______________________________________________________________|____________________________________________________________________|
 * @param argument: Not used
 * @retval None
 */
#if 1
void Start_MainBoard2PowerBoard_TransTask(void *argument)
{
//	SPITransHandler_t SPITransHandler;
//	SPITransHandler.spiPort = &hspi1;
//	SPITransHandler. SPIMODE = SPIMasterMode;
//	SPITransHandler.readACKTimeOut = 50;
//	SPITransHandler.readDataTimeOut = 100;
//	SPITransHandler.SPITransDir = Master_writeDataTo_Slave;

	/* Infinite loop */
	for (;;)
	{
		osMutexAcquire(mutex, osWaitForever);	//打印调试信息用
		if (xSemaphoreTake(ModbusH.ModBusSphrHandle, portMAX_DELAY) == pdTRUE)
		{
			if (ModbusH.FCStatus[MB_FC_WRITE_MULTIPLE_REGISTERS] != 0)// && (ModbusH.spiRx_uartTx_u8regs_size == 0)) 
			{
				Addto_osPrintf("start Start_MainBoard2PowerBoard_TransTask : %ld\r\n", xTaskGetTickCount());
				ModbusH.FCStatus[MB_FC_WRITE_MULTIPLE_REGISTERS] = 0;
				
				for (uint16_t i = 0 ; i < ModbusH.current_u16regs_num ; )
				{
					/* 解析出modbus数据，然后传给各个powerboard */
					BoardID_TypeDef boardID 		= highByte(ModbusDATA[ModbData_PowerBoard_Addr+i])-1;
					if ((boardID > PowerBoardNum) || (boardID < 1)) break;
					uint16_t PLCCmdBufferLen 				= lowByte(ModbusDATA[ModbData_PowerBoard_Addr+i]);
					uint8_t PLCCmdBuffer[PLCCmdBufferLen];
					i++;
					for (int j = 0; j < (PLCCmdBufferLen / 2); j++)
					{
						PLCCmdBuffer[j*2] 		= highByte(ModbusDATA[ModbData_PowerBoard_Addr+i]);
						PLCCmdBuffer[j*2+1] 	= lowByte(ModbusDATA[ModbData_PowerBoard_Addr+i]);
						i++;
					}
					BackPanelTrans_Master_writeDataTo_Slave(boardID, PLCCmdBuffer, PLCCmdBufferLen);
					//osDelay(10);
				}
			}
			xSemaphoreGive(ModbusH.ModBusSphrHandle);
		}
		osMutexRelease(mutex);
		osDelay(1);
	}
}
#endif

/****************   3-----DI Board 3     ********************
 * @brief Function implementing the DI Board 3 SpiTrans thread.
 * @param argument: Not used
 * @retval None
 */
void Start_DIB_3_SPITransTask(void *argument)
{
	/* Infinite loop */
	for (;;)
	{
		continue;
		// osDelay(1000);
	}
}
/****************   4-----DI Board 4     ********************
 * @brief Function implementing the DI Board 4 SpiTrans thread.
 * @param argument: Not used
 * @retval None
 */
void Start_DIB_4_SPITransTask(void *argument)
{
	/* Infinite loop */
	for (;;)
	{
		osDelay(1000);
	}
}
/****************   5-----DQ Board 1     ********************
 * @brief Function implementing the DQ Board 1 SpiTrans thread.
 * @param argument: Not used
 * @retval None
 */
void Start_DQB_1_SPITransTask(void *argument)
{
	/* Infinite loop */
	for (;;)
	{
		osDelay(1000);
	}
}
/****************   6-----DQ Board 2     ********************
 * @brief Function implementing the DQ Board 2 SpiTrans thread.
 * @param argument: Not used
 * @retval None
 */
void Start_DQB_2_SPITransTask(void *argument)
{
	/* Infinite loop */
	for (;;)
	{
		osDelay(1000);
	}
}
/****************   7-----Menu Board     ********************
 * @brief Function implementing the Menu Board SpiTrans thread.
 * @param argument: Not used
 * @retval None
 */
void Start_MenuB_SPITransTask(void *argument)
{
	/* Infinite loop */
	for (;;)
	{
		osDelay(1000);
	}
}
/****************   8-----RS485 Board     ********************
 * @brief Function implementing the RS485 Board SpiTrans thread.
 * @param argument: Not used
 * @retval None
 */
void Start_RS485B_SPITransTask(void *argument)
{
	/* Infinite loop */
	for (;;)
	{
		osDelay(1000);
	}
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
