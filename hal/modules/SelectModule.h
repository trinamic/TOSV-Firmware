/*
 * SelectModule.h
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#ifndef SELECT_MODULE_H
#define SELECT_MODULE_H

	// define cpu types
	#define STM32F103		1
	#define STM32F205		2

	// define module type
	#define STARTRAMPE_TOSV_V10				1
	#define TMC4671_TMC6100_TOSV_REF_V10	2

	// select the actual module
//	#define DEVICE STARTRAMPE_TOSV_V10
	#define DEVICE TMC4671_TMC6100_TOSV_REF_V10

#if DEVICE == STARTRAMPE_TOSV_V10

	/* device configuration for Startrampe based eval setup (STM32F205 256k) */
	#include "Startrampe-TOSV_v1.0.h"

#elif DEVICE == TMC4671_TMC6100_TOSV_REF_V10

	/* device configuration for TOSV reference board (STM32F103 128k) */
	#include "TMC4671-TMC6100-TOSV-REF_v1.0.h"

#else

	/* device not found */
	#error "No module selected!"
#endif

#endif /* SELECT_MODULE_H */
