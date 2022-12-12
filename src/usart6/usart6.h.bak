#pragma once
#include "project_config.h"

namespace usart6
{
	void usart6Init();
	void write(uint8_t);
	uint16_t read();
	uint16_t available();
	extern volatile uint8_t tx[8];
	extern volatile uint8_t rx[8];
	extern volatile uint16_t _rxCnt;
	extern volatile uint16_t _txCnt;
	extern volatile bool flag;
	extern volatile uint16_t _readCnt;
	extern volatile uint16_t _sendCnt;
}
