/*
 * BackPanelTrans.c
 *
 * Created: 2023.08.11
 *  Author: WU555
 * Description:	1、私有背板协议
 * 				2、多主模式，获取到令牌的从机才能主动给BaseBoard发送数据
 * 				3、采用RS485， 波特率：921600
 */

#include "BackPanelTrans.h"

uint8_t resultTrue  =		1;
uint8_t resultFalse =		0;

BackPanelHandler_t *_backPanelH;
uint8_t _PowerCtrlPara[255];
uint16_t _PowerCtrlParaLen;

uint8_t BackPanelProtocol_Init(BackPanelHandler_t *backpH)
{
	/* 获取该板子的ID号，由2个引脚定义 */
	//backpH->BoardID = getBoardID();
	if (backpH->BoardID == 0 || backpH->BoardID > 5)
	{
		LOG_error("get board id error!\r\n");
		return 0;
	}

	/*初始化令牌参数*/
	backpH->token_currentBoardID = backpH->BoardID;
	backpH->token_nextBoard_ID = backpH->BoardID + 1;
	backpH->token_validTime = 0;
	backpH->token_LastTime = 0;

	/* 初始化接收缓冲区 和发送缓冲区*/
	backpH->RecvBufCnt = 0;
	backpH->SendBufCnt = 0;
	memset(backpH->RecvBuf, 0, backpH->RecvBufSize);
	memset(backpH->SendBuf, 0, backpH->SendBufSize);

	/* 设置t35chars时间 */
	uint32_t baud = huart1.Init.BaudRate;
	if (baud <= 19200)
	{
		backpH->t35chars = 3500000 * 11 / baud;
	}
	else
	{
		backpH->t35chars = 1750; // 1750us
	}
	backpH->RecvLastTime = 0;

	// Receive data from serial port for Modbus using interrupt
	if (HAL_UART_Receive_IT(backpH->port, &backpH->RecvBuf[backpH->RecvBufCnt], 1) == HAL_OK) {
		backpH->RecvBufCnt++;
	}
	else {
		LOG_error("Receive data error!\r\n");
		return 0;
	}

	_backPanelH = backpH;

	// 待补充
	return 1;
}

/*背板协议传输的线程*/
void BackPanelProtocol_Process(void)
{
	BackPanelProtocol_ReceiveFrame();
}

void BackPanelProtocol_SendFrame(uint8_t msgtype, uint8_t *data, uint16_t len) {
	_backPanelH->TxEnd = 0;	//标记为 未完成发送
	size_t idx = 0;
	if (len + FRAME_BASELEN > (uint16_t)sizeof(_backPanelH->SendBuf))
	{
		len = sizeof(_backPanelH->SendBuf) - FRAME_BASELEN;
	}

	// 帧头
	_backPanelH->SendBuf[idx++] = FRAME_START_1;
	_backPanelH->SendBuf[idx++] = FRAME_START_2;

	// 内容长度
	memcpy(_backPanelH->SendBuf + idx, &len, 2);
	idx += 2;

	// 消息类型
	_backPanelH->SendBuf[idx++] = msgtype;

	// 数据
	memcpy(_backPanelH->SendBuf + idx, data, len);
	idx += len;

	// 校验
	uint16_t crc = CRC16_CCITT(_backPanelH->SendBuf, idx);
	memcpy(_backPanelH->SendBuf + idx, &crc, 2);
	idx += 2;


	LOG_info("write data to BaseBoard : ");
	for (size_t i = 0; i < idx; i++) {
		LOG("%d ", _backPanelH->SendBuf[i]);
	}
	LOG("\r\n");

	_backPanelH->RecvBufCnt = idx;
	// transfer buffer to serial line IT
	HAL_UART_Transmit_IT(_backPanelH->port, _backPanelH->SendBuf, idx);

}

void BackPanelProtocol_ReceiveFrame(void)
{
	if (_backPanelH->RecvBufCnt <= 0 || (HAL_getTick_us() - _backPanelH->RecvLastTime < _backPanelH->t35chars))
	{
		return;
	}

	/*读取串口接收到的数据，测试用*/
	LOG_info("read data : ");
	for (size_t i = 0; i < _backPanelH->RecvBufCnt; i++)
	{
		LOG("%d ", _backPanelH->RecvBuf[_backPanelH->RecvBufCnt]);
	}
	LOG("\r\n");

	/*查找包头位置，找不到的话，就退出*/
	uint16_t FrameStart = word(FRAME_START_1, FRAME_START_2); // FrameStart_1 << 8 | FrameStart_2;
	int16_t fs_a = contains(_backPanelH->RecvBuf, _backPanelH->RecvBufCnt, FrameStart);
	if (fs_a == -1)
	{
		LOG_error("FrameStart error!\r\n");
		return;
	}

	/*判断数据长度*/
	size_t inLen = word(_backPanelH->RecvBuf[fs_a + 3], _backPanelH->RecvBuf[fs_a + 2]) + FRAME_BASELEN; //_inBuffIdx;

	if (_backPanelH->RecvBufCnt < inLen)
	{
		LOG_error("read Data Len is too low! error!\r\n");
		return;
	}
	_backPanelH->RecvBufCnt = 0;

	/*校验crc*/
	uint16_t crc16 = CRC16_CCITT(_backPanelH->RecvBuf, inLen - 2);
	if (_backPanelH->RecvBuf[inLen - 1] != highByte(crc16) || _backPanelH->RecvBuf[inLen - 2] != lowByte(crc16))
	{
		LOG_error("crc error!\r\n");
		return;
	}

	// 消息类型
	uint8_t msgType = _backPanelH->RecvBuf[fs_a + 4]; // - MsgTypeBack; //0x81 - 0x80 = 0x01

	// 消息处理
	BackPanelProtocol_Analyze(msgType, _backPanelH->RecvBuf + 5, inLen - 7);
}

void BackPanelProtocol_Analyze(uint8_t msgType, uint8_t *data, uint16_t len) {
	// msgType = msgType - Recall_BaseMsgType; //0x81 - 0x80 = 0x01
	uint8_t datacache[255];
	memcpy(datacache, data, len);

	switch (msgType)
	{
	case ResetPowerBoard_MsgType:
		LOG_info("MsgType is 0x80 : ready to Restart PowerBoard !\r\n");
		break;
	case StartPowerBoard_MsgType:
		LOG_info("MsgType is 0x81 : ready to Start PowerBoard ! \r\n");
		break;
	case SendCMD_MsgType:
		LOG_info("MsgType is 0x82 : read cmd from BaseBoard !\r\n");
		if (data[0] != _backPanelH->BoardID) {
			LOG_info("BoardID is not match !\r\n");
			return;
		}
		_PowerCtrlParaLen = len - 1;
		memcpy(_PowerCtrlPara, data + 1, _PowerCtrlParaLen);
		//uint8_t result = 1;
		BackPanelProtocol_SendFrame(SendCMD_MsgType + Recall_BaseMsgType, &resultTrue, 1);
		break;
	case TransToken_MsgType:
		LOG_info("MsgType is 0x03 : goto check token is valid !\r\n");
		if (data[1] != _backPanelH->BoardID) {
			LOG_info("token is not match !\r\n");
			return;
		}
		
		/* 保存token有效时间，并开始计时，超时后，令牌会丢弃 */
		_backPanelH->token_validTime = word(data[3], data[2]);
		_backPanelH->token_LastTime = HAL_getTick_us();
		
		/* 解析token指令后面的是什么指令 */
		msgType = data[4];
		memcpy(datacache, data + 5, len - 5);
		//break;
	case TransData_MsgType:
		LOG_info("MsgType is 0x84 : goto trans data to BaseBoard !\r\n");
		if (datacache[0] != _backPanelH->BoardID || datacache[0] != 0xFF) {
			LOG_info("BoardID is not match(TransData) !\r\n");
			return;
		}
		TransData(datacache[0]);
		break;
	case ReadStatus_MsgType:
		LOG_info("MsgType is 0x85 : goto send status to BaseBoard !\r\n");
		break;
	default:
		LOG_error("MsgType Fault!");
	}
}


/* 发送数据给BaseBoard板 */
void TransData(uint8_t boardid) {
	uint8_t Data[255];
	uint16_t idx = 0;
	/* 板id如果为0xFF，说明时跟token指令复用的，需要添加token信息一起发送 */
	if (boardid == 0xFF) {
		uint8_t tinfo[10];
		uint16_t tinfoLen = AddTokenInfo(tinfo);
		memcpy(Data + idx, tinfo, tinfoLen);
		idx += tinfoLen;

		uint8_t td_msgtype = TransData_MsgType + Recall_BaseMsgType;
		memcpy(Data + idx, &td_msgtype, 1);
		idx += 1;

		memcpy(Data + idx, _backPanelH->LoadData, _backPanelH->LoadDataLen);
		idx += _backPanelH->LoadDataLen;
		if (CheckTokenisValid()) {
			BackPanelProtocol_SendFrame(TransToken_MsgType, Data, idx);
		}
	} else {
		memcpy(Data + idx, _backPanelH->LoadData, _backPanelH->LoadDataLen);
		idx += _backPanelH->LoadDataLen;
		BackPanelProtocol_SendFrame(TransData_MsgType + Recall_BaseMsgType, Data, idx);
	}

}

uint16_t AddTokenInfo(uint8_t tokeninfo[]) {
	uint16_t idx = 0;
	memcpy(tokeninfo + idx, &_backPanelH->token_currentBoardID, 1);
	idx += 1;

    memcpy(tokeninfo + idx, &_backPanelH->token_nextBoard_ID, 1);
    idx += 1;

	//计算剩余的有效时间，并传递给下一个板子
	_backPanelH->token_validTime = _backPanelH->token_validTime - (HAL_getTick_us() - _backPanelH->token_LastTime);
	if (_backPanelH->token_validTime < 0) _backPanelH->token_validTime = 0;
    memcpy(tokeninfo + idx, &_backPanelH->token_validTime, 2);
    idx += 2;
	return idx;
}

/* 确认令牌是否有效 */
uint8_t CheckTokenisValid(void) {
	if ((_backPanelH->token_validTime != 0) && 
		(HAL_getTick_us() - _backPanelH->token_LastTime < _backPanelH->token_validTime)) {
			LOG_info("token is valid, set mode to master! \r\n");
			return 1;
			//_backPanelH->BPTransMode = BPTrans_Master;
		}
	return 0;
}

void mergeData(uint8_t tdata[], uint16_t tdatalen, uint8_t isdone) {
	
}

/* 填充需要发送给BaseBoard板的数据 */
void fillTransData(uint8_t tdata[], uint16_t tdatalen) {
	memcpy(_backPanelH->LoadData, tdata, tdatalen);
	_backPanelH->LoadDataLen = tdatalen;
}

/* 将功率控制的相关参数传递给 PowerControl中 */
void getPowerCtrlData(uint8_t *pcdata) {
	for (size_t i = 0; i < _PowerCtrlParaLen; i++) {
		pcdata[i] = _PowerCtrlPara[i];
	}
}

int16_t contains(uint8_t arr[], uint16_t len, uint16_t target)
{
	uint8_t highByte = target >> 8;
	uint8_t lowByte = target & 0xFF;
	for (size_t i = 0; i < len - 1; i++)
	{
		if (arr[i] == highByte && arr[i + 1] == lowByte)
		{
			return i;
		}
	}
	return -1;
}

