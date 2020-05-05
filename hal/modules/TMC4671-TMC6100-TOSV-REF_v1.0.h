/*
 * TMC4671-TMC6100-TOSV-REF_v1.0.h
 *
 *  Created on: 08.04.2020
 *      Author: ED
 */

#ifndef TMC4671_TMC6100_TOSV_REF_V10_H
#define TMC4671_TMC6100_TOSV_REF_V10_H

	#include "SelectModule.h"

#if DEVICE==TMC4671_TMC6100_TOSV_REF_V10

	#define BOARD_CPU STM32F103
	#define NUMBER_OF_MOTORS 		1

	#include "cpu/STM32F103/stm32f10x_lib.h"
	#include "cpu/STM32F103/stm32f10x_gpio.h"
	#include "cpu/STM32F103/stm32f10x_rcc.h"

	#define MAX_VELOCITY 				(int32_t)200000
	#define MAX_ACCELERATION			(int32_t)100000

	#define MAX_CURRENT 				(int32_t)6000		// RMS current
	#define MAX_PRESSURE				(int32_t)70000
	#define MAX_VOLUME				    (int32_t)70000

	#define TMCM_USE_IIC_INTERFACE
	#define I2C_PRESSURE_SENSOR_SM9333
	#define DIFF_PRESSURE_SENSOR_SM9333

	// ===== UART configuration =====
	#define USE_UART_INTERFACE
	#define USE_USART1_ON_PB6_PB7

	#include "../../Definitions.h"
	#include "../HAL_Definitions.h"
	#include "../Flags.h"
	#include "../../TOSV.h"

	#include "TMC-API/tmc/ic/TMC4671/TMC4671.h"
	#include "TMC-API/tmc/ic/TMC4671/TMC4671_Variants.h"
	#include "TMC-API/tmc/ic/TMC6200/TMC6200.h"
	#include "TMC-API/tmc/ramp/LinearRamp.h"

	// module number in HEX (0020)
	#define SW_TYPE_HIGH 		0x00
	#define SW_TYPE_LOW  		0x14

	#define SW_VERSION_HIGH 	1
	#define SW_VERSION_LOW  	7

	#define TMCM_EEPROM_MAGIC	(uint8_t)0x6B	// 107

	#define WEASEL_SPI2_ON_PB13_PB14_PB15
	#define DRAGON_SPI2_ON_PB13_PB14_PB15
	#define EEPROM_SPI2_ON_PB13_PB14_PB15

	TMC_LinearRamp rampGenerator[NUMBER_OF_MOTORS];
	TOSV_Config tosvConfig[NUMBER_OF_MOTORS];

	#define MIN_CRITICAL_TEMP     	100 //100°C
	#define MAX_CRITICAL_TEMP     	120	//120°C

	#define MAX_SUPPLY_VOLTAGE		3600 	// 36.0V
	#define ON__SUPPLY_VOLTAGE		60 		// 6.0V
	#define MIN_SUPPLY_VOLTAGE		60		// 6.0V
	#define VOLTAGE_FAKTOR		   	79.2	// *10 because of [0,1V] TMC4671-LA
	#define VOLTAGE_OFFSET		  	37100 	//TMC4671-LA

	#define PRESSURE_SENSOR_PIN		0

#endif /* DEVICE==TMC4671_TMC6100_TOSV_REF_V10 */

#endif /* TMC4671_TMC6100_TOSV_REF_V10_H */
