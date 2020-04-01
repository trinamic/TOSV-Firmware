/*
 * Cpu.h
 *
 *  Created on: 01.04.2020
 *      Author: ED
 */
#ifndef CPU_TMC_H
#define CPU_TMC_H

	#include "../Hal_Definitions.h"

#if BOARD_CPU==STM32F205
	#define CKTIM	((u32)120000000uL) 	// silicon running at 120MHz resolution: 1Hz
#else
	#error "CKTIM() for BOARD_CPU not supported!"
#endif

	void cpu_init();

#endif /* CPU_TMC_H */


