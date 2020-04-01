/*
 * Definitions.h
 *
 *  Created on: 31.03.2020
 *      Author: ED / OK
 */

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

	#include "TMC-API/tmc/helpers/API_Header.h"

//	#define UNUSED(x) (void)(x)
//
//	#include <stddef.h>
//	#include <stdbool.h>
//	#include <stdint.h>

	#include "modules/SelectModule.h"

	typedef struct
	{
		uint8_t baudrate;
		uint8_t serialModuleAddress;
		uint8_t serialHostAddress;
	} TModuleConfig;

	typedef struct
	{
		int32_t maxPositioningSpeed;
		uint32_t maximumCurrent;
	} TMotorConfig;

	TModuleConfig moduleConfig;
	TMotorConfig motorConfig;

	// init functions
	extern void tmcm_initModuleConfig();
	extern void tmcm_initMotorConfig();
	extern void tmcm_initModuleSpecificIO();

	// LEDs
	extern void tmcm_led_run_toggle();

	// driver control
	extern void tmcm_enableDriver();
	extern void tmcm_disableDriver();
	extern uint8_t tmcm_getDriverState();

#endif /* DEFINITIONS_H */
