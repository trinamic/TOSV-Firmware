/*
 * TOSV.c
 *
 *  Created on: 8.4.2020
 *      Author: OK
 */

#include "TOSV.h"

typedef struct
{
  uint32_t PEEP;
  uint32_t Limit;
  uint32_t RiseTime;
  uint32_t FallTime;
  uint32_t Frequency;
  uint32_t ItoE;
  uint32_t Volume;
} TOSVParameters_T;

static TOSVParameters_T TOSVParameters;

uint8_t TOSV_setPEEP(uint32_t value)
{
  if(value<=20)
    TOSVParameters.PEEP=value;
  else
    return false;

  return true;
}

uint8_t TOSV_setLimit(uint32_t value)
{
  if(value<=50)
    TOSVParameters.Limit=value;
  else
    return false;

  return true;
}

uint8_t TOSV_setRiseTime(uint32_t value)
{
  if(value<=500)
    TOSVParameters.RiseTime=value;
  else
    return false;

  return true;
}

uint8_t TOSV_setFallTime(uint32_t value)
{
  if(value<=500)
    TOSVParameters.FallTime=value;
  else
    return false;

  return true;
}

uint8_t TOSV_setFrequency(uint32_t value)
{
  if(value>=5 && value<=40)
    TOSVParameters.Frequency=value;
  else
    return false;

  return true;
}

uint8_t TOSV_setItoE(uint32_t value)
{
  if(value<=100)
    TOSVParameters.ItoE=value;
  else
    return false;

  return true;
}

uint8_t TOSV_setVolume(uint32_t value)
{
  if(value<=50)
    TOSVParameters.Volume=value;
  else
    return false;

  return true;
}

uint32_t TOSV_getPEEP(void)
{
  return TOSVParameters.PEEP;
}

uint32_t TOSV_getLimit(void)
{
  return TOSVParameters.Limit;
}

uint32_t TOSV_getRiseTime(void)
{
  return TOSVParameters.RiseTime;
}

uint32_t TOSV_getFallTime(void)
{
  return TOSVParameters.FallTime;
}

uint32_t TOSV_getFrequency(void)
{
  return TOSVParameters.Frequency;
}

uint32_t TOSV_getItoE(void)
{
  return TOSVParameters.ItoE;
}

uint32_t TOSV_getVolume(void)
{
  return TOSVParameters.Volume;
}

uint8_t TOSV_startVentilator(void)
{
  return false;  //Ventilator konnte nicht gestartet werden
}

void TOSV_stopVentilator(void)
{
}

void TOSV_init(void)
{
	//TEST_OK: Maybe read parameters from EEPROM in future versions?
	TOSVParameters.Frequency=5;
}

void TOSV_process(void)
{
}
