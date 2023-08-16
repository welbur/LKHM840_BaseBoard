/*
 * BackPanelTrans.h
 *
 * Created: 2023-08-11
 *  Author: WU555
 */

#ifndef BACKPANELTRANS_H_
#define BACKPANELTRANS_H_

#include "stm32f4xx_hal.h"
#include "PowerBoardConfig.h"
#include "PinConfig.h"
#include "LOG.h"
#include "FrameFormat.h"
#include "CRC.h"
#include "MSP_USART.h"
#include "MSP_TIM.h"

#define word(a, b) (((uint16_t)(a) << 8) | (b))		//2个uint8_t合并为一个uint16_t
#define lowByte(x) ((uint8_t)(x))					//读取uint16_t中的低八位
#define highByte(x) ((uint8_t)((x) >> 8))		//读取uint16_t中的高八位

#define MAX_BUFFER	255



/**
 * @struct BackPanelHandler_t
 * @brief
 * BackPanel Trans handler structure
 * Contains all the variables required for Modbus daemon operation
 */
typedef struct
{

	UART_HandleTypeDef *port; //HAL Serial Port handler
	uint8_t BoardID; //该板子的ID号，用于背板传输用

 	uint8_t RecvBuf[MAX_BUFFER];
 	uint8_t SendBuf[MAX_BUFFER];
	uint8_t RecvBufSize;
	uint8_t SendBufSize;
	uint8_t RecvBufCnt;
	uint8_t SendBufCnt;
	uint8_t TxEnd;

	uint8_t LoadData[MAX_BUFFER];	//用于保存各路负载的数据（包括负载有效电流值、有效功率等）
	uint16_t LoadDataLen;

	uint16_t t35chars;
	uint32_t RecvLastTime;

	uint32_t token_LastTime;

	uint8_t token_currentBoardID;
	uint8_t token_nextBoard_ID;
	uint16_t token_validTime;
}
BackPanelHandler_t;

extern BackPanelHandler_t *_backPanelH;

uint8_t BackPanelProtocol_Init(BackPanelHandler_t *backpH);
void BackPanelProtocol_Process(void);
void BackPanelProtocol_SendFrame(uint8_t msgtype, uint8_t *data, uint16_t len);
void BackPanelProtocol_ReceiveFrame(void);
void BackPanelProtocol_Analyze(uint8_t msgType, uint8_t *data, uint16_t len);
void TransData(uint8_t boardid);
uint16_t AddTokenInfo(uint8_t tokeninfo[]);
uint8_t CheckTokenisValid(void);
void fillTransData(uint8_t tdata[], uint16_t tdatalen);
void getPowerCtrlData(uint8_t *pcdata);

int16_t contains(uint8_t arr[], uint16_t N, uint16_t target);

#endif /* BACKPANELTRANS_H_ */