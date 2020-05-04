/*
 * TOSV.c
 *
 *  Created on: 8.4.2020
 *      Author: OK / ED
 */

#ifndef TOSV_H
#define TOSV_H

	#include "TMC-API/tmc/helpers/API_Header.h"

	typedef enum
	{
	  TOSV_MODE_PRESSURE_CONTROL,
	  TOSV_MODE_VOLUME_CONTROL,
	} TOSV_Mode;

	typedef struct
	{
		uint8_t  actualState;
		uint16_t timer;
		uint16_t tStartup;
		uint16_t tInhalationRise;
		uint16_t tInhalationPause;
		uint16_t tExhalationFall;
		uint16_t tExhalationPause;
		uint32_t pLIMIT;
		uint32_t pPEEP;
		uint32_t volumeMax;
		TOSV_Mode mode;
	} TOSV_Config;

	#define TOSV_STATE_STOPPED				0
	#define TOSV_STATE_STARTUP				1
	#define TOSV_STATE_INHALATION_RISE	    2
	#define TOSV_STATE_INHALATION_PAUSE	    3
	#define TOSV_STATE_EXHALATION_FALL	    4
	#define TOSV_STATE_EXHALATION_PAUSE	    5

	void tosv_init(TOSV_Config *config);
	void tosv_initFlowSensor();
	void tosv_process(TOSV_Config *config);
	void tosv_enableVentilator(TOSV_Config *config, bool enable);
	bool tosv_isVentilatorEnabled(TOSV_Config *config);

	void tosv_zeroFlow();
	void tosv_resetVolumeIntegration();
	int32_t tosv_getFlowValue();
	void tosv_updateFlowSensor();
	int32_t tosv_updateVolume(uint8_t motor);

#endif
