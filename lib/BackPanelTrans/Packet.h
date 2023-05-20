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
#ifndef _PACKET_H_
#define	_PACKET_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include "Arduino.h"
//#include "PacketCRC.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "LOG.h"

#define START_BYTE 				 0x7E
#define STOP_BYTE  				 0x81

#define PREAMBLE_SIZE   		 4
#define POSTAMBLE_SIZE			 2
#define MAX_PACKET_SIZE			 0xFE // Maximum allowed payload bytes per packet

#define DEFAULT_TIMEOUT 		 50

#define Default_crcLen		8
//#define TableLen(_crcLen)			pow(2, _crcLen)

typedef void (*functionPtr)();

typedef enum 
{
	STALE_PACKET_ERROR 		= -3,
	STOP_BYTE_ERROR    		= -2,
	PAYLOAD_ERROR      		= -1,
	CRC_ERROR          		= 0,
	NO_DATA            		= 1,
	NEW_DATA           		= 2,
	CONTINUE           		= 3,
}PacketStatus_TypeDef;

typedef enum 
{
	find_start_byte,
	find_id_byte,
	find_overhead_byte,
	find_payload_len,
	find_payload,
	find_crc,
	find_end_byte
}fsm_TypeDef;
#if 0
fsm_TypeDef state = find_start_byte;

const functionPtr* callbacks    = NULL;
uint8_t            callbacksLen = 0;

uint8_t bytesToRec      = 0;
uint8_t payIndex        = 0;
uint8_t idByte          = 0;
uint8_t overheadByte    = 0;
uint8_t recOverheadByte = 0;

uint32_t packetStart    = 0;
uint32_t timeout;
#endif


//extern uint8_t CRC_csTable[TableLen]; 	//用于crc校验算法

/**
 * @struct D_I_BoardHandler_t
 * @brief
 * DI board handler structure
 * Contains all the variables required for Modbus daemon operation
 */
typedef struct 
{
	uint8_t PtxBuff[MAX_PACKET_SIZE];
	uint8_t PrxBuff[MAX_PACKET_SIZE];
	uint8_t preamble[PREAMBLE_SIZE];		//   = {START_BYTE, 0, 0, 0};
	uint8_t postamble[POSTAMBLE_SIZE];		// = {0, STOP_BYTE};

//	uint16_t 	messageLen;
//	uint8_t 	packetID;
	uint8_t 	bytesRead;						// = 0;
	int8_t  	status;							//    = 0;

}PacketHandler_t;

extern PacketHandler_t pktHandle;

void PacketInit(void);		//(PacketHandler_t *pktHandle);
uint8_t PacketConstructPacket(const uint16_t messageLen, const uint8_t packetID);
uint8_t PacketParse(const uint8_t *recChar, const uint8_t valid);
uint8_t PacketCurrentPacketID();
void PacketCalcOverhead(uint8_t arr[], const uint8_t len);
int16_t PacketFindLast(uint8_t arr[], const uint8_t len);
void PacketStuffPacket(uint8_t arr[], const uint8_t len);
void PacketUnpackPacket(uint8_t arr[]);
uint16_t PacketTxObj(uint8_t* val, const uint16_t index, const uint16_t len);
uint16_t PacketRxObj(uint8_t* val, const uint16_t index, const uint16_t len);
void PacketReset();

void CRC_Init(const uint8_t poly);

uint8_t CRC_Calculate(uint8_t arr[], uint16_t len);

#ifdef __cplusplus
}
#endif

#endif	/* _PACKET_H_ */