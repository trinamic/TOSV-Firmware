/*
 * TOSV.c
 *
 *  Created on: 8.4.2020
 *      Author: OK / ED
 */

#ifndef TOSV_H
#define TOSV_H

	#include "TMC-API/tmc/helpers/API_Header.h"

	typedef struct
	{
		uint8_t actualState;
		uint32_t timer;

		uint32_t timeStartup;
		uint32_t timeState1;
		uint32_t timeState2;
		uint32_t timeState3;
		uint32_t timeState4;

		uint32_t maxPressure;
		uint32_t peepPressure;


//		uint32_t PEEP;
//		uint32_t Limit;
//		uint32_t RiseTime;
//		uint32_t FallTime;
//		uint32_t Frequency;
//		uint32_t ItoE;
//		uint32_t Volume;
	} TOSV_Config;

	#define TOSV_STATE_STOPPED	0
	#define TOSV_STATE_STARTUP	1

	#define TOSV_STATE_1	    2
	#define TOSV_STATE_2	    3
	#define TOSV_STATE_3	    4
	#define TOSV_STATE_4	    5

	void tosv_init(TOSV_Config *config);
	void tosv_process(TOSV_Config *config);
	void tosv_enableVentilator(TOSV_Config *config, bool enable);
	bool tosv_isVentilatorEnabled(TOSV_Config *config);

//uint8_t TOSV_setPEEP(uint32_t value);
//uint8_t TOSV_setLimit(uint32_t value);
//uint8_t TOSV_setRiseTime(uint32_t value);
//uint8_t TOSV_setFallTime(uint32_t value);
//uint8_t TOSV_setFrequency(uint32_t value);
//uint8_t TOSV_setItoE(uint32_t value);
//uint8_t TOSV_setVolume(uint32_t value);
//uint32_t TOSV_getPEEP(void);
//uint32_t TOSV_getLimit(void);
//uint32_t TOSV_getRiseTime(void);
//uint32_t TOSV_getFallTime(void);
//uint32_t TOSV_getFrequency(void);
//uint32_t TOSV_getItoE(void);
//uint32_t TOSV_getVolume(void);


#endif
