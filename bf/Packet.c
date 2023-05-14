#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define START_BYTE 0xAA
#define STOP_BYTE 0xBB
#define MAX_PACKET_SIZE 1024

typedef struct {
  uint8_t id;
  uint8_t overhead;
  uint8_t payload_len;
  uint8_t payload[MAX_PACKET_SIZE];
  uint8_t crc;
} packet_t;

void Packet_begin(const bool& debug, const uint32_t& timeout);

uint8_t Packet_constructPacket(const uint16_t& messageLen, const uint8_t& packetID);

uint8_t Packet_parse(const uint8_t& recChar);

uint8_t Packet_currentPacketID();

void Packet_calcOverhead(uint8_t arr[], const uint8_t& len);

int16_t Packet_findLast(uint8_t arr[], const uint8_t& len);

void Packet_stuffPacket(uint8_t arr[], const uint8_t& len);

void Packet_unpackPacket(uint8_t arr[]);

void Packet_reset();

#endif // PACKET_H


uint8_t Packet_constructPacket(const uint16_t& messageLen, const uint8_t& packetID) {
  if (messageLen > MAX_PACKET_SIZE) {
    // Calculate the overhead for the packet.
    uint8_t overhead = Packet_calcOverhead(txBuff, MAX_PACKET_SIZE);
    // Stuff the packet with the message and the CRC.
    Packet_stuffPacket(txBuff, MAX_PACKET_SIZE);
    // Calculate the CRC for the packet.
    uint8_t crcVal = crc.calculate(txBuff, MAX_PACKET_SIZE);
    // Write the packet ID, overhead, and payload length to the preamble.
    preamble[1] = packetID;
    preamble[2] = overhead;
    preamble[3] = MAX_PACKET_SIZE;
    // Write the CRC to the postamble.
    postamble[0] = crcVal;
    // Return the size of the packet.
    return MAX_PACKET_SIZE;
  } else {
    // Calculate the overhead for the packet.
    uint8_t overhead = Packet_calcOverhead(txBuff, (uint8_t)messageLen);
    // Stuff the packet with the message and the CRC.
    Packet_stuffPacket(txBuff, (uint8_t)messageLen);
    // Calculate the CRC for the packet.
    uint8_t crcVal = crc.calculate(txBuff, (uint8_t)messageLen);
    // Write the packet ID, overhead, and payload length to the preamble.
    preamble[1] = packetID;
    preamble[2] = overhead;
    preamble[3] = messageLen;
    // Write the CRC to the postamble.
    postamble[0] = crcVal;
    // Return the size of the packet.
    return (uint8_t)messageLen;
  }
}

uint8_t Packet::parse(const uint8_t recChar, const bool valid)
{
  bool packet_fresh = (packetStart == 0) || ((HAL_GetTick() - packetStart) < timeout);

  if (!packet_fresh) // packet is stale, start over.
  {
    bytesRead = 0;
    state = find_start_byte;
    status = STALE_PACKET_ERROR;
    packetStart = 0;

    return bytesRead;
  }

  if (valid)
  {
    switch (state)
    {
    case find_start_byte:
    {
      if (recChar == START_BYTE)
      {
        state = find_id_byte;
        packetStart = HAL_GetTick(); // start the timer
      }

      break;
    }

    case find_id_byte:
    {
      idByte = recChar;
      state = find_overhead_byte;
      break;
    }

    case find_overhead_byte:
    {
      recOverheadByte = recChar;
      state = find_payload_len;
      break;
    }

    case find_payload_len:
    {
      if ((recChar > 0) && (recChar <= MAX_PACKET_SIZE))
      {
        bytesToRec = recChar;
        payIndex = 0;
        state = find_payload;
      }
      else
      {
        bytesRead = 0;
        state = find_start_byte;
        status = PAYLOAD_ERROR;

        reset();
        return bytesRead;
      }
      break;
    }

    case find_payload:
    {
      if (payIndex < bytesToRec)
      {
        rxBuff[payIndex] = recChar;
        payIndex++;
        if (payIndex == bytesToRec)
          state = find_crc;
      }
      break;
    }

    case find_crc:
    {
      uint8_t calcCrc = crc.calculate(rxBuff, bytesToRec);
      if (calcCrc == recChar)
        state = find_end_byte;
      else
      {
        bytesRead = 0;
        state = find_start_byte;
        status = CRC_ERROR;

        reset();
        return bytesRead;
      }

      break;
    }

    case find_end_byte:
    {
      state = find_start_byte;

      if (recChar == STOP_BYTE)
      {
        unpackPacket(rxBuff);
        bytesRead = bytesToRec;
        status = NEW_DATA;

        if (callbacks)
        {
          if (idByte < callbacksLen)
            callbacks[idByte]();
        }
        packetStart = 0; // reset the timer
        return bytesToRec;
      }

      bytesRead = 0;
      status = STOP_BYTE_ERROR;

      reset();
      return bytesRead;
      break;
    }

    default:
    {

      reset();
      bytesRead = 0;
      state = find_start_byte;
      break;
    }
    }
  }
  else
  {
    bytesRead = 0;
    status = NO_DATA;
    return bytesRead;
  }

  bytesRead = 0;
  status = CONTINUE;
  return bytesRead;
}



void Packet_calcOverhead(uint8_t arr[], uint8_t len)
{
	uint8_t overheadByte = 0xFF;

	for (uint8_t i = 0; i < len; i++)
	{
		if (arr[i] == START_BYTE)
		{
			overheadByte = i;
			break;
		}
	}
}



#include "Packet.h"

PacketCRC crc;

void Packet::begin(const configST& configs)
{
	//debugPort    = configs.debugPort;
	debug        = configs.debug;
	callbacks    = configs.callbacks;
	callbacksLen = configs.callbacksLen;
	timeout 	 = configs.timeout;
}

void Packet::begin(const bool& _debug, const uint32_t& _timeout)
{
	//debugPort = &_debugPort;
	debug     = _debug;
	timeout   = _timeout;
}

uint8_t Packet::constructPacket(const uint16_t& messageLen, const uint8_t& packetID)
{
	if (messageLen > MAX_PACKET_SIZE)
	{
		calcOverhead(txBuff, MAX_PACKET_SIZE);
		stuffPacket(txBuff, MAX_PACKET_SIZE);
		uint8_t crcVal = crc.calculate(txBuff, MAX_PACKET_SIZE);

		preamble[1] = packetID;
		preamble[2] = overheadByte;
		preamble[3] = MAX_PACKET_SIZE;

		postamble[0] = crcVal;

		return MAX_PACKET_SIZE;
	}
	else
	{
		calcOverhead(txBuff, (uint8_t)messageLen);
		stuffPacket(txBuff, (uint8_t)messageLen);
		uint8_t crcVal = crc.calculate(txBuff, (uint8_t)messageLen);

		preamble[1] = packetID;
		preamble[2] = overheadByte;
		preamble[3] = messageLen;

		postamble[0] = crcVal;

		return (uint8_t)messageLen;
	}
}

uint8_t Packet::parse(const uint8_t& recChar, const bool& valid)
{
	bool packet_fresh = (packetStart == 0) || ((HAL_GetTick() - packetStart) < timeout);

	if(!packet_fresh) //packet is stale, start over.
	{

		bytesRead   = 0;
		state       = find_start_byte;
		status      = STALE_PACKET_ERROR;
		packetStart = 0;

		return bytesRead;
	}

	if (valid)
	{
		switch (state)
		{
		case find_start_byte: /////////////////////////////////////////
		{
			if (recChar == START_BYTE)
			{
				state       = find_id_byte;
				packetStart = HAL_GetTick();	//start the timer
			}

			break;
		}

		case find_id_byte: ////////////////////////////////////////////
		{
			idByte = recChar;
			state  = find_overhead_byte;
			break;
		}

		case find_overhead_byte: //////////////////////////////////////
		{
			recOverheadByte = recChar;
			state           = find_payload_len;
			break;
		}

		case find_payload_len: ////////////////////////////////////////
		{
			if ((recChar > 0) && (recChar <= MAX_PACKET_SIZE))
			{
				bytesToRec = recChar;
				payIndex   = 0;
				state      = find_payload;
			}
			else
			{
				bytesRead = 0;
				state     = find_start_byte;
				status    = PAYLOAD_ERROR;

				reset();
				return bytesRead;
			}
			break;
		}

		case find_payload: ////////////////////////////////////////////
		{
			if (payIndex < bytesToRec)
			{
				rxBuff[payIndex] = recChar;
				payIndex++;
				if (payIndex == bytesToRec)
					state    = find_crc;
			}
			break;
		}

		case find_crc: ///////////////////////////////////////////
		{
			uint8_t calcCrc = crc.calculate(rxBuff, bytesToRec);
			if (calcCrc == recChar)
				state = find_end_byte;
			else
			{
				bytesRead = 0;
				state     = find_start_byte;
				status    = CRC_ERROR;

				reset();
				return bytesRead;
			}

			break;
		}

		case find_end_byte: ///////////////////////////////////////////
		{
			state = find_start_byte;

			if (recChar == STOP_BYTE)
			{
				unpackPacket(rxBuff);
				bytesRead = bytesToRec;
				status    = NEW_DATA;

				if (callbacks)
				{
					if (idByte < callbacksLen)
						callbacks[idByte]();
				}
				packetStart = 0;	// reset the timer
				return bytesToRec;
			}

			bytesRead = 0;
			status    = STOP_BYTE_ERROR;

			reset();
			return bytesRead;
			break;
		}

		default:
		{

			reset();
			bytesRead = 0;
			state     = find_start_byte;
			break;
		}
		}
	}
	else
	{
		bytesRead = 0;
		status    = NO_DATA;
		return bytesRead;
	}

	bytesRead = 0;
	status    = CONTINUE;
	return bytesRead;
}

uint8_t Packet::currentPacketID()
{
	return idByte;
}

void Packet::calcOverhead(uint8_t arr[], const uint8_t& len)
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

int16_t Packet::findLast(uint8_t arr[], const uint8_t& len)
{
	for (uint8_t i = (len - 1); i != 0xFF; i--)
		if (arr[i] == START_BYTE)
			return i;

	return -1;
}

void Packet::stuffPacket(uint8_t arr[], const uint8_t& len)
{
	int16_t refByte = findLast(arr, len);

	if (refByte != -1)
	{
		for (uint8_t i = (len - 1); i != 0xFF; i--)
		{
			if (arr[i] == START_BYTE)
			{
				arr[i]  = refByte - i;
				refByte = i;
			}
		}
	}
}

void Packet::unpackPacket(uint8_t arr[])
{
	uint8_t testIndex = recOverheadByte;
	uint8_t delta     = 0;
	if (testIndex <= MAX_PACKET_SIZE)
	{
		while (arr[testIndex])
		{
			delta          = arr[testIndex];
			arr[testIndex] = START_BYTE;
			testIndex += delta;
		}
		arr[testIndex] = START_BYTE;
	}
}

void Packet::reset()
{
	memset(txBuff, 0, sizeof(txBuff));
	memset(rxBuff, 0, sizeof(rxBuff));

	bytesRead   = 0;
	packetStart = 0;
}
convert this code into c language