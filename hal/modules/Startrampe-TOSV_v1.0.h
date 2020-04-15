/*
 * Startrampe-TOSV_v1.0.h
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#ifndef STARTRAMPE_TOSV_V10_H
#define STARTRAMPE_TOSV_V10_H

	#include "SelectModule.h"

#if DEVICE==STARTRAMPE_TOSV_V10

	#define BOARD_CPU STM32F205
	#define NUMBER_OF_MOTORS 		1

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
	#define TMCM_MAX_CURRENT 			(int32_t)5000

	#define EEPROM_SPI1_ON_PB3_PB4_PB5

	// ===== UART configuration =====
	#define USE_UART_INTERFACE
	#define USE_USART3_ON_PD8_PD9

	// ===== USB configuration =====
	#define USE_USB_INTERFACE
	#define USBD_VID                        0x2A3C
	#define USBD_PID                        0x0700
	#define USBD_PRODUCT_FS_STRING          "Evaluation Device"
	#define USBD_SERIALNUMBER_FS_STRING     "TMCEVAL"

	#include "../../Definitions.h"
	#include "../HAL_Definitions.h"
	#include "../Flags.h"

	#include "TMC-API/tmc/ic/TMC4671/TMC4671.h"
	#include "TMC-API/tmc/ic/TMC4671/TMC4671_Variants.h"
	#include "TMC-API/tmc/ic/TMC6200/TMC6200.h"
	#include "TMC-API/tmc/ramp/LinearRamp.h"

	// module number in HEX (0020)
	#define SW_TYPE_HIGH 		0x00
	#define SW_TYPE_LOW  		0x14

	#define SW_VERSION_HIGH 	1
	#define SW_VERSION_LOW  	0

	#define TMCM_EEPROM_MAGIC	(uint8_t)0x64	// 100

	#define WEASEL_SPI3_ON_PC10_PC11_PC12
	#define DRAGON_SPI3_ON_PC10_PC11_PC12

	#define PRESSURE_SENSOR_PIN 2

	TMC_LinearRamp rampGenerator[NUMBER_OF_MOTORS];

#endif /* DEVICE==STARTRAMPE_TOSV_V10 */

#endif /* STARTRAMPE_TOSV_V10_H */
