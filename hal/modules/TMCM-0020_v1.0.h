/*
 * TMCM-0020_v1.0.h
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#ifndef TMCM_0020_V10_H
#define TMCM_0020_V10_H

	#include "SelectModule.h"

#if DEVICE==TMCM_0020_V10

	#define BOARD_CPU STM32F205

	#include "cpu/STM32F205/stm32f2xx.h"
	#include "cpu/STM32F205/stm32f2xx_dac.h"
	#include "cpu/STM32F205/core_cmInstr.h"
	#include "cpu/STM32F205/usbd_req.h"
	#include "cpu/STM32F205/usb_dcd_int.h"
	#include "cpu/STM32F205/usbd_usr.h"
	#include "cpu/STM32F205/usbd_desc.h"
	#include "cpu/STM32F205/usbd_cdc_core.h"

	#define MAX_VELOCITY 				(int32_t)200000
	#define MAX_ACCELERATION			(int32_t)100000
	#define TMCM_MAX_TORQUE 			(int32_t)5000

	#define USE_USB_INTERFACE
	#define USE_ALIVE_LED

	#define USBD_VID                        0x2A3C
	#define USBD_PID                        0x0700
	#define USBD_PRODUCT_FS_STRING          "Evaluation Device"
	#define USBD_SERIALNUMBER_FS_STRING     "TMCEVAL"

	#include "../../Definitions.h"
	#include "../HAL_Definitions.h"
	#include "../Flags.h"

	// module number in HEX (0020)
	#define SW_TYPE_HIGH 		0x00
	#define SW_TYPE_LOW  		0x14

	#define SW_VERSION_HIGH 	1
	#define SW_VERSION_LOW  	0

#endif /* DEVICE==TMCM_0020_V10 */

#endif /* TMCM_0020_V10_H */
