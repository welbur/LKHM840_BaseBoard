/*
 * BackPanelTrans.h
 *
 * Created: 2023-08-11
 *  Author: WU555
 */

#include "BackPanelTrans.h"

/**
 * @brief
 * This is the callback for HAL interrupts of UART TX used by Modbus library.
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {

	if (_backPanelH->port == UartHandle)
	{
		// notify the end of TX
		_backPanelH->TxEnd = 1;
	}
}

#if 1
/**
 * @brief
 * This is the callback for HAL interrupt of UART RX
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {

	/* BackPanel Trans RTU RX callback BEGIN */

	if (_backPanelH->port == UartHandle)
	{
		if (HAL_UART_Receive_IT(_backPanelH->port, &_backPanelH->RecvBuf[_backPanelH->RecvBufCnt], 1) == HAL_OK)
		{
			_backPanelH->RecvBufCnt++;
		}
		else
		{
			LOG_error("BackPanelTrans receive it error!");
		}
		_backPanelH->RecvLastTime = HAL_getTick_us();
	}

	/* BackPanel Trans RTU RX callback END */
}
#endif
