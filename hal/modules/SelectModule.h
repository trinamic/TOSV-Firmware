/*
 * SelectModule.h
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#ifndef SELECT_MODULE_H
#define SELECT_MODULE_H

	// define cpu types
	#define STM32F205				1

	// define module type
	#define STARTRAMPE_TOSV_V10		1

	// select the actual module
	#define DEVICE STARTRAMPE_TOSV_V10

#if DEVICE == STARTRAMPE_TOSV_V10

	/* device configuration for TMCM-0020 (STM32F205 256k) */
	#include "Startrampe-TOSV_v1.0.h"

#else

	/* device not found */
	#error "No module selected!"
#endif

#endif /* SELECT_MODULE_H */
