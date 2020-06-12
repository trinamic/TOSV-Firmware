/*
 * RS485.h
 *
 *  Created on: 12.06.2020
 *      Author: ED
 */

#ifndef RS485_H
#define RS485_H

	#include "../Hal_Definitions.h"

#if defined(USE_RS485_INTERFACE)

	void rs485_init(uint32_t baudRate);
	void rs485_write(char ch);
	char rs485_read(char *ch);
	uint32_t rs485_checkTimeout();

#endif

#endif
