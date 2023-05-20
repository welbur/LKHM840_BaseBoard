#include "Packet.h"


PacketHandler_t pktHandle;
fsm_TypeDef state = find_start_byte;

const functionPtr* callbacks    = NULL;
uint8_t            callbacksLen = 0;

uint8_t bytesToRec      = 0;
uint8_t payIndex        = 0;
uint8_t idByte          = 0;
uint8_t overheadByte    = 0;
uint8_t recOverheadByte = 0;

uint32_t packetStart    = 0;
uint32_t timeout;	// = DEFAULT_TIMEOUT;

uint8_t CRC_csTable[256]; 
//uint8_t txBuff[MAX_PACKET_SIZE];
//uint8_t rxBuff[MAX_PACKET_SIZE];
//uint8_t preamble[PREAMBLE_SIZE]   = {START_BYTE, 0, 0, 0};
//uint8_t postamble[POSTAMBLE_SIZE] = {0, STOP_BYTE};

void PacketInit(void)	//(PacketHandler_t *pktHandle)
{
	pktHandle.preamble[0] = START_BYTE;	pktHandle.preamble[1] = 0; pktHandle.preamble[2] = 0; pktHandle.preamble[3] = 0; 
	pktHandle.postamble[0] = 0; pktHandle.postamble[1] = STOP_BYTE;
	pktHandle.bytesRead			= 0;
	pktHandle.status			= 0;
	//pktHandle = *PktH;
}
/*
 uint8_t Packet::constructPacket(const uint16_t& messageLen, const uint8_t& packetID)
 Description:
 ------------
  * Calculate, format, and insert the packet protocol metadata into the packet transmit
  buffer
 Inputs:
 -------
  * const uint16_t& messageLen - Number of values in txBuff
  to send as the payload in the next packet
  * const uint8_t& packetID - The packet 8-bit identifier
 Return:
 -------
  * uint8_t - Number of payload bytes included in packet
*/
uint8_t PacketConstructPacket(const uint16_t messageLen, const uint8_t packetID)
{
	if (messageLen > MAX_PACKET_SIZE)
	{
		PacketCalcOverhead(pktHandle.PtxBuff, MAX_PACKET_SIZE);
		PacketStuffPacket(pktHandle.PtxBuff, MAX_PACKET_SIZE);
		uint8_t crcVal = CRC_Calculate(pktHandle.PtxBuff, MAX_PACKET_SIZE);

		pktHandle.preamble[1] = packetID;
		pktHandle.preamble[2] = overheadByte;
		pktHandle.preamble[3] = MAX_PACKET_SIZE;

		pktHandle.postamble[0] = crcVal;

		return MAX_PACKET_SIZE;
	}
	else
	{
		PacketCalcOverhead(pktHandle.PtxBuff, (uint8_t)messageLen);
		PacketStuffPacket(pktHandle.PtxBuff, (uint8_t)messageLen);
		uint8_t crcVal = CRC_Calculate(pktHandle.PtxBuff, (uint8_t)messageLen);

		pktHandle.preamble[1] = packetID;
		pktHandle.preamble[2] = overheadByte;
		pktHandle.preamble[3] = messageLen;

		pktHandle.postamble[0] = crcVal;

		return (uint8_t)messageLen;
	}
}

/*
 uint8_t Packet::parse(const uint8_t& recChar, const bool& valid)
 Description:
 ------------
  * Parses incoming serial data, analyzes packet contents,
  and reports errors/successful packet reception. Executes
  callback functions for parsed packets whos ID has a
  corresponding callback function set via
  "void Packet::begin(const configST configs)"
 Inputs:
 -------
  * const uint8_t& recChar - Next char to parse in the stream
  * const bool& valid - Set if stream is "available()" and clear if not
 Return:
 -------
  * uint8_t - Num bytes in RX buffer
*/
uint8_t PacketParse(const uint8_t *recChar, const uint8_t valid)
{
	uint8_t packet_fresh;
	packet_fresh = (packetStart == 0) || ((HAL_GetTick() - packetStart) < DEFAULT_TIMEOUT);
	if (!packet_fresh)
	{ // packet is stale, start over.
		Addto_osPrintf("packet_fresh = 0\n");
		pktHandle.bytesRead = 0;
		state = find_start_byte;
		pktHandle.status = STALE_PACKET_ERROR;
		packetStart = 0;
		return pktHandle.bytesRead;
	}

	if (valid)
	{
		switch (state)
		{
		case find_start_byte:
		{
			if (*recChar == START_BYTE)
			{
				state = find_id_byte;
				packetStart = HAL_GetTick(); // start the timer
			}
			break;
		}

		case find_id_byte:
		{
			idByte = *recChar;
			state = find_overhead_byte;
			break;
		}

		case find_overhead_byte:
		{
			recOverheadByte = *recChar;
			state = find_payload_len;
			break;
		}

		case find_payload_len:
		{
			if ((*recChar > 0) && (*recChar <= MAX_PACKET_SIZE))
			{
				bytesToRec = *recChar;
				payIndex = 0;
				state = find_payload;
			}
			else
			{
				pktHandle.bytesRead = 0;
				state = find_start_byte;
				pktHandle.status = PAYLOAD_ERROR;
				PacketReset();
				return pktHandle.bytesRead;
			}
			break;
		}

		case find_payload:
		{
			if (payIndex < bytesToRec)
			{
				pktHandle.PrxBuff[payIndex] = *recChar;
				payIndex++;
				if (payIndex == bytesToRec)
					state = find_crc;
			}
			//Addto_osPrintf("payIndex = %d, bytesToRec = %d\r\n", payIndex, bytesToRec);
			break;
		}

		case find_crc:
		{
			uint8_t calcCrc = CRC_Calculate(pktHandle.PrxBuff, bytesToRec);
			//Addto_osPrintf("recchar = %d, calccrc = %d\r\n", *recChar, calcCrc);
			if (calcCrc == *recChar)
				state = find_end_byte;
			else
			{
				pktHandle.bytesRead = 0;
				state = find_start_byte;
				pktHandle.status = CRC_ERROR;

				PacketReset();
				return pktHandle.bytesRead;
			}
			break;
		}

		case find_end_byte:
		{
			state = find_start_byte;
			//Addto_osPrintf("rec char is stop byte : %d\r\n", *recChar);
			if (*recChar == STOP_BYTE)
			{
				PacketUnpackPacket(pktHandle.PrxBuff);
				pktHandle.bytesRead = bytesToRec;
				pktHandle.status = NEW_DATA;

				if (callbacks)
				{
					if (idByte < callbacksLen)
						callbacks[idByte]();
				}
				packetStart = 0; // reset the timer
				return bytesToRec;
			}

			pktHandle.bytesRead = 0;
			pktHandle.status = STOP_BYTE_ERROR;
			Addto_osPrintf("goto packet reset!\r\n");
			PacketReset();
			return pktHandle.bytesRead;
			break;
		}

		default:
		{
			PacketReset();
			pktHandle.bytesRead = 0;
			state = find_start_byte;
			break;
		}
		}
	}
	else
	{
		pktHandle.bytesRead = 0;
		pktHandle.status = NO_DATA;
		return pktHandle.bytesRead;
	}

	pktHandle.bytesRead = 0;
	pktHandle.status = CONTINUE;
	return pktHandle.bytesRead;
}

/*
 uint8_t Packet::currentPacketID()
 Description:
 ------------
  * Returns the ID of the last parsed packet
 Inputs:
 -------
  * void
 Return:
 -------
  * uint8_t - ID of the last parsed packet
*/
uint8_t PacketCurrentPacketID()
{
	return idByte;
}

/*
 void Packet::calcOverhead(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Calculates the COBS (Consistent Overhead Stuffing) Overhead
  byte and stores it in the class's overheadByte variable. This
  variable holds the byte position (within the payload) of the
  first payload byte equal to that of START_BYTE
 Inputs:
 -------
  * uint8_t arr[] - Array of values the overhead is to be calculated
  over
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * void
*/
void PacketCalcOverhead(uint8_t arr[], const uint8_t len)
{
	overheadByte = 0xFF;

	for (uint8_t i = 0; i < len; i++)
	{
		if (arr[i] == START_BYTE)
		{
			overheadByte = i;
			break;
		}
	}
}

/*
 int16_t Packet::findLast(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Finds last instance of the value START_BYTE within the given
  packet array
 Inputs:
 -------
  * uint8_t arr[] - Packet array
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * int16_t - Index of last instance of the value START_BYTE within the given
  packet array
*/
int16_t PacketFindLast(uint8_t arr[], const uint8_t len)
{
	for (uint8_t i = len - 1; i != 0xFF; i--)
	{
		if (arr[i] == START_BYTE)
		{
			return i;
		}
	}

	return -1;
}

/*
 void Packet::stuffPacket(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Enforces the COBS (Consistent Overhead Stuffing) ruleset across
  all bytes in the packet against the value of START_BYTE
 Inputs:
 -------
  * uint8_t arr[] - Array of values to stuff
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * void
*/
void PacketStuffPacket(uint8_t arr[], const uint8_t len)
{
	int16_t refByte = PacketFindLast(arr, len);

	if (refByte != -1)
	{
		for (uint8_t i = len - 1; i != 0xFF; i--)
		{
			if (arr[i] == START_BYTE)
			{
				arr[i] = refByte - i;
				refByte = i;
			}
		}
	}
}

/*
 void Packet::unpackPacket(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Unpacks all COBS-stuffed bytes within the array
 Inputs:
 -------
  * uint8_t arr[] - Array of values to unpack
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * void
*/
void PacketUnpackPacket(uint8_t arr[])
{
	uint8_t testIndex = recOverheadByte;
	uint8_t delta = 0;
	if (testIndex <= MAX_PACKET_SIZE)
	{
		while (arr[testIndex])
		{
			delta = arr[testIndex];
			arr[testIndex] = START_BYTE;
			testIndex += delta;
		}
		arr[testIndex] = START_BYTE;
	}
}

uint16_t PacketTxObj(uint8_t *val, const uint16_t index, const uint16_t len)
{
	uint8_t *ptr = (uint8_t *)val;
	uint16_t maxIndex;

	if ((len + index) > MAX_PACKET_SIZE)
	{
		maxIndex = MAX_PACKET_SIZE;
	}
	else
	{
		maxIndex = len + index;
	}
	//LOG("ptxbuff : ");
	for (uint16_t i = index; i < maxIndex; i++)
	{
		pktHandle.PtxBuff[i] = *ptr;
		//LOG(" %02X ", pktHandle.PtxBuff[i]);
		ptr++;
	}
	//LOG("\r\n");
	return maxIndex;
}

uint16_t PacketRxObj(uint8_t *val, const uint16_t index, const uint16_t len)
{
	uint8_t *ptr = (uint8_t *)val;
	uint16_t maxIndex;
	if ((len + index) > MAX_PACKET_SIZE)
		maxIndex = MAX_PACKET_SIZE;
	else
		maxIndex = len + index;
	for (uint16_t i = index; i < maxIndex; i++)
	{
		*ptr = pktHandle.PrxBuff[i];
		ptr++;
	}
	return maxIndex;
}

/*
 void Packet::reset()
 Description:
 ------------
  * Clears out the tx, and rx buffers, plus resets
  the "bytes read" variable, finite state machine, etc
 Inputs:
 -------
  * void
 Return:
 -------
  * void
*/
void PacketReset()
{
	memset(pktHandle.PtxBuff, 0, sizeof(pktHandle.PtxBuff));
	memset(pktHandle.PrxBuff, 0, sizeof(pktHandle.PrxBuff));

	pktHandle.bytesRead = 0;
	packetStart = 0;
}


void CRC_Init(const uint8_t poly)
{
	//uint8_t *csTable;
	uint16_t tableLen_ = 1<<Default_crcLen;
//	Addto_osPrintf("tableLen_ = %d\n", tableLen_);
	for (uint16_t i = 0; i < tableLen_; ++i)
	{
		int curr = i;
		for (int j = 0; j < 8; ++j)
		{
			if ((curr & 0x80) != 0)
			{
				curr = (curr << 1) ^ poly;
			}
			else
			{
				curr <<= 1;
			}
		}
		CRC_csTable[i] = curr;
	}
}

#if 0
uint8_t CRC_calculate(const uint8_t val)
{
	if (val < tableLen_)
	{
		return csTable[val];
	}
	return 0;
}
#endif

uint8_t CRC_Calculate(uint8_t arr[], uint16_t len)
{
	uint8_t crc = 0;
	for (uint16_t i = 0; i < len; i++)
	{

		crc = CRC_csTable[crc ^ arr[i]];
//		Addto_osPrintf("arr[%d] = %d,  crc = %d\n", i, arr[i], crc);
	}

	return crc;
}

#if 0
extern void *Packet_C_New(void)
{
	return new Packet(false);
}
extern uint8_t Packet_C_constructPacket(void *packet, uint16_t messageLen, uint8_t packetID)
{
	Packet *pack = (Packet *)packet;
	return pack->constructPacket(messageLen, packetID);
}
extern uint8_t Packet_C_parse(void *packet, uint8_t recChar, int valid)
{
	Packet *pack = (Packet *)packet;
	return pack->parse(recChar, valid);
}
extern int8_t Packet_C_getPacketStatus(void *packet)
{
	Packet *pack = (Packet *)packet;
	return pack->status;
}
extern uint8_t * Packet_C_getPackettxBuff(void *packet)
{
	Packet *pack = (Packet *)packet;
	return pack->txBuff;
}
extern uint8_t * Packet_C_getPacketrxBuff(void *packet)
{
	Packet *pack = (Packet *)packet;
	return pack->rxBuff;
}
extern uint16_t Packet_C_txobj(void *packet, uint8_t *val, uint16_t index)
{
	Packet *pack = (Packet *)packet;
	uint8_t txbuf[50];
	for (int i =0; i < 4; i++) {
		txbuf[i] = *val;
		val++;
	}
	index = pack->txObj(txbuf, index);
	return index;			
}
extern uint16_t Packet_C_rxobj(void *packet, uint8_t *val, uint16_t index)
{
	Packet *pack = (Packet *)packet;
	uint8_t rbuf[50];
	index = pack->rxObj(rbuf, index, pack->bytesRead);
	for (int i =0; i < index; i++) {
		*val = rbuf[i];
		val++;
	}
	return index;			
}
extern void Packet_C_reset(void *packet)
{
	Packet *pack = (Packet *)packet;
	pack->reset();
}
#endif