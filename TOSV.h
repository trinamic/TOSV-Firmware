/*
 * TOSV.c
 *
 *  Created on: 8.4.2020
 *      Author: OK
 */

#ifndef __TOSV_H
#define __TOSV_H

#include "Definitions.h"

uint8_t TOSV_setPEEP(uint32_t value);
uint8_t TOSV_setLimit(uint32_t value);
uint8_t TOSV_setRiseTime(uint32_t value);
uint8_t TOSV_setFallTime(uint32_t value);
uint8_t TOSV_setFrequency(uint32_t value);
uint8_t TOSV_setItoE(uint32_t value);
uint8_t TOSV_setVolume(uint32_t value);
uint32_t TOSV_getPEEP(void);
uint32_t TOSV_getLimit(void);
uint32_t TOSV_getRiseTime(void);
uint32_t TOSV_getFallTime(void);
uint32_t TOSV_getFrequency(void);
uint32_t TOSV_getItoE(void);
uint32_t TOSV_getVolume(void);
uint8_t TOSV_startVentilator(void);
void TOSV_stopVentilator(void);
void TOSV_init(void);
void TOSV_process(void);

#endif
