/*
 * TMCL.h
 *
 *  Created on: 01.04.2020
 *      Author: OK / ED
 */

#ifndef TMCL_H
#define TMCL_H

	#include "Definitions.h"
	#include "BLDC.h"
	#include "hal/modules/SelectModule.h"

	void tmcl_init();
	void tmcl_processCommand();
	void tmcl_resetCPU(uint8_t resetPeripherals);

#endif
