#include "ModbPacket.h"

/**
 * @brief
 * 解析接收到的modbus数据包
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
uint8_t Modbus_FC15_ParsePacket(uint16_t *u16regs, uint16_t u16regsaddr, uint16_t u16regsLen)
{
	/* 判断是否为dq板的数据 */
	if (u16regsaddr == FC15_DQBoard_AddrOffset)
	{
		for(uint16_t i = 0; i < u16regsLen; i++)
		{
			LOG("dq board data u16regs[%d] : %04X\r\n", i, u16regs[i]);
		}
	}

	return 1;
}

uint8_t Modbus_FC16_ParsePacket(uint16_t *u16regs, uint16_t u16regsaddr, uint16_t u16regsLen)
{
	for(uint16_t i = 0; i < u16regsLen; i++)
	{
		LOG("power board data u16regs[%d] : %04X\r\n", (i+u16regsaddr), u16regs[i+u16regsaddr]);
	}

	return 1;
}

uint16_t Modbus_ConvertTo_DataAddress(uint16_t u16regsaddr, uint16_t u16regsnum, uint8_t mb_fc)
{
	switch (mb_fc)
	{
		case MB_FC_WRITE_MULTIPLE_COILS:
			u16regsaddr = u16regsaddr - FC15_DQBoard_AddrOffset;
			//if (u16regsaddr < 0)
			//	return -1;
			//modH->FCAddrHandle.FC15_u16StartCoil = u16StartCoil;
			//modH->FCAddrHandle.FC15_u16regsno = u16Coilno / 16;
			//LOG("u16StartCoil : %04X\r\n", u16StartCoil);
			break;

		default:
			break;
	}
	return u16regsaddr;
}


