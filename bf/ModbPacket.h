/*
01111110 00000000 11111111 00000000 00000000 00000000 ... 00000000 10000001
|      | |      | |      | |      | |      | |      | | | |      | |______|__Stop byte
|      | |      | |      | |      | |      | |      | | | |______|___________8-bit CRC
|      | |      | |      | |      | |      | |      | |_|____________________Rest of payload
|      | |      | |      | |      | |      | |______|________________________2nd payload byte
|      | |      | |      | |      | |______|_________________________________1st payload byte
|      | |      | |      | |______|__________________________________________# Nums of payload bytes
|      | |      | |______|___________________________________________________COBS Overhead byte
|      | |______|____________________________________________________________Packet ID (0 by default)
|______|_____________________________________________________________________Start byte (constant)
*/
#ifndef _MODBPACKET_H_
#define	_MODBPACKET_H_

#ifdef __cplusplus
extern "C" {
#endif


//#include "stm32f4xx_hal.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include <string.h>
//#include <math.h>
//#include "LOG.h"
#include "Modbus.h"
												//ModbusData size 0x0320(800个)


/**
 * @struct FC_StartAddr_t
 * @brief
 * 
 * 
 */
typedef struct
{
    uint16_t FC15_RS485Board_u16StartCoil;     
    uint16_t FC15_RS485Board_u16regsno;     
    uint16_t FC16_u16StartAddr; 
    uint16_t FC16_u16regsno;
}
FC_Addr_t;


//uint8_t Modbus_ConstructPacket(const uint16_t messageLen, const uint8_t packetID);
uint8_t Modbus_FC15_ParsePacket(uint16_t *u16regs, uint16_t u16regsaddr, uint16_t u16regsLen);
uint8_t Modbus_FC16_ParsePacket(uint16_t *u16regs, uint16_t u16regsaddr, uint16_t u16regsLen);
uint16_t Modbus_ConvertTo_DataAddress(uint16_t u16regsaddr, uint16_t u16regsnum, uint8_t mb_fc);
//uint8_t PacketCurrentPacketID();
//void PacketCalcOverhead(uint8_t arr[], const uint8_t len);
//int16_t PacketFindLast(uint8_t arr[], const uint8_t len);
//void PacketStuffPacket(uint8_t arr[], const uint8_t len);
//void PacketUnpackPacket(uint8_t arr[]);

#ifdef __cplusplus
}
#endif

#endif	/* _MODBPACKET_H_ */