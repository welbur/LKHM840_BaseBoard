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
#include "SPITransfer_C.h"
#include "Modbus.h"

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
uint32_t countt = 0;
uint8_t mod_preamble[MOD_PREAMBLE_SIZE] = {MOD_START_BYTE, 0, 0, 0};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name = "defaultTask",
	.stack_size = 32 * 4,
	.priority = (osPriority_t)osPriorityLow,
};
#if 1
/* 1----Definitions for Start Slave Board SPITransTask */
osThreadId_t DIBoard_TransTaskHandle;
const osThreadAttr_t DIBoard_TransTask_attributes = {
	.name = "DIBoard_TransTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t)osPriorityBelowNormal1, // osPriorityHigh
};
#endif
osThreadId_t DQBoard_TransTaskHandle;
const osThreadAttr_t DQBoard_TransTask_attributes = {
	.name = "DQBoard_TransTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t)osPriorityBelowNormal, // osPriorityHigh
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Start_DIBoard_TransTask(void *argument);
void Start_DQBoard_TransTask(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	/* creation of work led Task */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	/* creation of TaskmyTaskSlave */
#if 1
	DIBoard_TransTaskHandle = osThreadNew(Start_DIBoard_TransTask, NULL, &DIBoard_TransTask_attributes);
	DQBoard_TransTaskHandle = osThreadNew(Start_DQBoard_TransTask, NULL, &DQBoard_TransTask_attributes);
#endif

	/* USER CODE END Init */
	/* USER CODE BEGIN Header */
	/**
	 ******************************************************************************
	 * File Name          :
	 * Description        :
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

	/**
	 * @}
	 */

	/**
	 * @}
	 */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;)
	{
		if (SlaveBoardH[DI_Board_1].isBoard_Rx_En == 1)
		{
			// SlaveBoardH[DI_Board_1].isBoardEnable = 0;
			LED_G_TogglePin;
		}
		else
		{
			LED_B_TogglePin;
			//LOGI("test print time");
		}
		#if 0
		uint32_t msTickstart = xTaskGetTickCount();
		uint32_t printno = 0;
		do
		{
			LOGI("test print time......%ld......%ld\r\n", printno++, xTaskGetTickCount());
		} while ((xTaskGetTickCount() - msTickstart) < 1); 
		#endif
		osDelay(1000);
	}
	/* USER CODE END StartDefaultTask */
}

/****************   1-----DI Board      ********************
 * @brief Function implementing the DI Board SpiTrans thread.
 * @param argument: Not used
 * @retval None
 */
void Start_DIBoard_TransTask(void *argument)
{
	/* Infinite loop */
	for (;;)
	{
		uint8_t re_arr_size = MOD_PREAMBLE_SIZE; // 暂用数组前面的4个元素，作为包头使用
		uint8_t re_arr[128];
		/*1-------------------------有触发信号后，读取相关slave板的所有数据*/
		for (int i = 0; i < DIBoard_NO; i++)
		{
			if (SlaveBoardH[i].isBoard_Rx_En)
			{
				void *sTrans = SPITransfer_C_New(&SlaveBoardH[i], &hspi1, SET_SPIMODE_MASTER);
				SPITransfer_C_Master_Spi1_Transfer(sTrans, RxFlag, SlaveBoardH[i].BoardID);
				LOGI("current board %d status : .....%d......~~~~~~~~~~~~\r\n", SlaveBoardH[i].BoardID, SlaveBoardH[i].spiTransState);
				SlaveBoardH[i].isBoard_Rx_En = 0;
				if (SlaveBoardH[i].spiTransState == SpiTrans_End)
				{
					/*从spi通道读到数据后，把slave板从1-8所有的数据都读出来后，合并在一起，然后再发给modbus主机(pn板)*/
					COPY_ARRAY(re_arr + re_arr_size, SlaveBoardH[i].spiRx_uartTx_u8regs, SlaveBoardH[i].spiRx_uartTx_u8regs_size);
					re_arr_size += SlaveBoardH[i].spiRx_uartTx_u8regs_size;
				}
			}
		}
#if 1
		if (re_arr_size != MOD_PREAMBLE_SIZE)
		{
			LOG("rec111 data : ");
			for (int j = 0; j < re_arr_size; j++)
				LOG("%02X ", re_arr[j]);
			LOG("\r\n.......%d~~~~~~~~~~~~~%ld-------------%ld\r\n", re_arr_size, HAL_GetTick(), ++countt);
			/*增加包头*/
			re_arr[0] = mod_preamble[0], re_arr[1] = mod_preamble[1];
			re_arr[2] = re_arr_size - MOD_PREAMBLE_SIZE; // 有效数据的长度
			re_arr[3] = MB_FC_READ_REGISTERS;			 // 功能指令码
			/*放到modbus里面去发送数据*/
			ModbusH.spiRx_uartTx_u8regs = re_arr;
			ModbusH.spiRx_uartTx_u8regs_size = re_arr_size;
			spiRxUartTxBuffer(&ModbusH);
			ModbusH.spiRx_uartTx_u8regs_size = 0;
		}
#endif
		osDelay(5);
	}
}

/****************   2-----DQ Board      ********************
 * @brief 等待modbus指令，然后将数据发送给DQ板
 * @param argument: Not used
 * @retval None
 */
void Start_DQBoard_TransTask(void *argument)
{
	/* Infinite loop */
	for (;;)
	{
		if (xSemaphoreTake(ModbusH.ModBusSphrHandle, portMAX_DELAY) == pdTRUE)
		{
			if (ModbusH.FCStatus[MB_FC_WRITE_MULTIPLE_COILS] != 0)// && (ModbusH.spiRx_uartTx_u8regs_size == 0)) 
			{
				//uint8_t dbuf[50];
				ModbusH.FCStatus[MB_FC_WRITE_MULTIPLE_COILS] = 0;
				for (int dqi = 0; dqi < 2; dqi++) 	//for (int dqi = 4; dqi < 6; dqi++) 
				{
					for (uint16_t i = 0; i < ModbusH.FCAddrHandle.FC15_u16regsno; i++) 
					{
						//dbuf[i*2] = highByte(ModbusDATA[ModbusH.FCAddrHandle.FC15_u16StartCoil+i]);
						//dbuf[i*2+1] = lowByte(ModbusDATA[ModbusH.FCAddrHandle.FC15_u16StartCoil+i]);
						//SlaveBoardH[dqi].spiTx_uartRx_u8regs[i*2] = dbuf[i*2];
						//SlaveBoardH[dqi].spiTx_uartRx_u8regs[i*2+1] = dbuf[i*2+1];
						SlaveBoardH[dqi].spiTx_uartRx_Buffer[i*2] = highByte(ModbusDATA[ModbusH.FCAddrHandle.FC15_u16StartCoil+i]);
						SlaveBoardH[dqi].spiTx_uartRx_Buffer[i*2+1] = lowByte(ModbusDATA[ModbusH.FCAddrHandle.FC15_u16StartCoil+i]);
					}
					SlaveBoardH[dqi].spiTx_uartRx_Buffer_Size = ModbusH.FCAddrHandle.FC15_u16regsno * 2;
					void *sTrans = SPITransfer_C_New(&SlaveBoardH[dqi], &hspi1, SET_SPIMODE_MASTER);
					SPITransfer_C_Master_Spi1_Transfer(sTrans, TxFlag, SlaveBoardH[dqi].BoardID);
					LOGI("end dqboard trans task..............boardID : %d.......\r\n", SlaveBoardH[dqi].BoardID);
				}
				LOGI("\r\n");
			}
			xSemaphoreGive(ModbusH.ModBusSphrHandle);
		}
		osDelay(1);
		//continue;
	}
}
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
