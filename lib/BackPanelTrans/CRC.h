/*
 * CRC.c
 *
 * Created: 2023.08.11
 *  Author: WU555
 */

#ifndef CRC_H_
#define CRC_H_

#include "stm32f4xx_hal.h"

#define CRC_INIT 0 // CCITT初始CRC为全0

uint16_t CRC16_CCITT(const void *obj, unsigned int length);
void crc16(uint8_t *data, uint8_t length, uint8_t *crc);

#endif /* CRC_H_ */