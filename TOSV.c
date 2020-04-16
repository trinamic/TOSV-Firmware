/*
 * TOSV.c
 *
 *  Created on: 8.4.2020
 *      Author: OK / ED
 */

#include "TOSV.h"

extern bool bldc_setTargetPressure(uint8_t motor, int32_t targetPressure);

void tosv_init(TOSV_Config *config)
{
	config->actualState = TOSV_STATE_STOPPED;
	config->timer = 0;
	config->timeStartup = 2000;
	config->timeState1 = 500;
	config->timeState2 = 1000;
	config->timeState3 = 500;
	config->timeState4 = 1000;
	config->maxPressure = 2000;
	config->peepPressure = 1200;
}

void tosv_enableVentilator(TOSV_Config *config, bool enable)
{
	if (enable)
	{
		config->actualState = TOSV_STATE_STARTUP;
	} else {
		config->actualState = TOSV_STATE_STOPPED;
		bldc_setTargetPressure(0, 0);
	}
}

bool tosv_isVentilatorEnabled(TOSV_Config *config)
{
	return (config->actualState != TOSV_STATE_STOPPED) ? true : false;
}

void tosv_process(TOSV_Config *config)
{
	config->timer++;

	switch(config->actualState)
	{
		case TOSV_STATE_STOPPED:
			// reset timer
			config->timer = 0;
			break;
		case TOSV_STATE_STARTUP:
			bldc_setTargetPressure(0, 0 + (config->peepPressure*config->timer)/config->timeStartup);
			if (config->timer >= config->timeStartup)
			{
				config->actualState = TOSV_STATE_1;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_1:
			bldc_setTargetPressure(0, config->peepPressure + ((config->maxPressure-config->peepPressure)*config->timer)/config->timeState1);
			if (config->timer >= config->timeState1)
			{
				config->actualState = TOSV_STATE_2;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_2:
			bldc_setTargetPressure(0, config->maxPressure);
			if (config->timer >= config->timeState2)
			{
				config->actualState = TOSV_STATE_3;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_3:
			bldc_setTargetPressure(0, config->peepPressure);
			if (config->timer >= config->timeState3)
			{
				config->actualState = TOSV_STATE_4;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_4:
			bldc_setTargetPressure(0, config->peepPressure);
			if (config->timer >= config->timeState4)
			{
				config->actualState = TOSV_STATE_1;
				config->timer = 0;
			}
			break;
	}
}


//uint8_t TOSV_setPEEP(uint32_t value)
//{
//  if(value<=20)
//    TOSVParameters.PEEP=value;
//  else
//    return false;
//
//  return true;
//}
//
//uint8_t TOSV_setLimit(uint32_t value)
//{
//  if(value<=50)
//    TOSVParameters.Limit=value;
//  else
//    return false;
//
//  return true;
//}
//
//uint8_t TOSV_setRiseTime(uint32_t value)
//{
//  if(value<=500)
//    TOSVParameters.RiseTime=value;
//  else
//    return false;
//
//  return true;
//}
//
//uint8_t TOSV_setFallTime(uint32_t value)
//{
//  if(value<=500)
//    TOSVParameters.FallTime=value;
//  else
//    return false;
//
//  return true;
//}
//
//uint8_t TOSV_setFrequency(uint32_t value)
//{
//  if(value>=5 && value<=40)
//    TOSVParameters.Frequency=value;
//  else
//    return false;
//
//  return true;
//}
//
//uint8_t TOSV_setItoE(uint32_t value)
//{
//  if(value<=100)
//    TOSVParameters.ItoE=value;
//  else
//    return false;
//
//  return true;
//}
//
//uint8_t TOSV_setVolume(uint32_t value)
//{
//  if(value<=50)
//    TOSVParameters.Volume=value;
//  else
//    return false;
//
//  return true;
//}
//
//uint32_t TOSV_getPEEP(void)
//{
//  return TOSVParameters.PEEP;
//}
//
//uint32_t TOSV_getLimit(void)
//{
//  return TOSVParameters.Limit;
//}
//
//uint32_t TOSV_getRiseTime(void)
//{
//  return TOSVParameters.RiseTime;
//}
//
//uint32_t TOSV_getFallTime(void)
//{
//  return TOSVParameters.FallTime;
//}
//
//uint32_t TOSV_getFrequency(void)
//{
//  return TOSVParameters.Frequency;
//}
//
//uint32_t TOSV_getItoE(void)
//{
//  return TOSVParameters.ItoE;
//}
//
//uint32_t TOSV_getVolume(void)
//{
//  return TOSVParameters.Volume;
//}
//
