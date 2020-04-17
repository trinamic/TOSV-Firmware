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
	config->tStartup = 2000;
	config->tInhalationRise = 300;
	config->tInhalationPause = 1000;
	config->tExhalationFall = 200;
	config->tExhalationPause = 1500;
	config->pLIMIT = 5000;
	config->pPEEP = 1300;
}

void tosv_enableVentilator(TOSV_Config *config, bool enable)
{
	if (enable)
	{
		// only start if not running
		if (config->actualState == TOSV_STATE_STOPPED)
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
			bldc_setTargetPressure(0, 0 + (config->pPEEP*config->timer)/config->tStartup);
			if (config->timer >= config->tStartup)
			{
				config->actualState = TOSV_STATE_INHALATION_RISE;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_INHALATION_RISE:
			bldc_setTargetPressure(0, config->pPEEP + ((config->pLIMIT-config->pPEEP)*config->timer)/config->tInhalationRise);
			if (config->timer >= config->tInhalationRise)
			{
				config->actualState = TOSV_STATE_INHALATION_PAUSE;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_INHALATION_PAUSE:
			bldc_setTargetPressure(0, config->pLIMIT);
			if (config->timer >= config->tInhalationPause)
			{
				config->actualState = TOSV_STATE_EXHALATION_FALL;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_EXHALATION_FALL:
			bldc_setTargetPressure(0, config->pPEEP + ((config->pLIMIT-config->pPEEP)*(config->tExhalationFall-config->timer))/config->tExhalationFall);
			if (config->timer >= config->tExhalationFall)
			{
				config->actualState = TOSV_STATE_EXHALATION_PAUSE;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_EXHALATION_PAUSE:
			bldc_setTargetPressure(0, config->pPEEP);
			if (config->timer >= config->tExhalationPause)
			{
				config->actualState = TOSV_STATE_INHALATION_RISE;
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
