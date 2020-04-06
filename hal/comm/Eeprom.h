/*
 * Eeprom.h
 *
 *  Created on: 06.04.2020
 *      Author: OK / ED
 */

#ifndef EEPROM_H
#define EEPROM_H

	#include "../Hal_Definitions.h"

	// constants for the EEPROM
	#define TMCM_ADDR_MODULE_CONFIG (u32)0
	#define TMCM_ADDR_MOTOR_CONFIG 	(u32)64
	#define TMCM_ADDR_EEPROM_MAGIC 	(u32)2047

	void eeprom_initConfig();

	void eeprom_writeConfigByte(uint32_t address, uint8_t Value);
	uint8_t eeprom_readConfigByte(uint32_t address);

	void eeprom_writeConfigBlock(uint32_t address, uint8_t *block, uint32_t size);
	void eeprom_readConfigBlock(uint32_t address, uint8_t *block, uint32_t size);

#endif
