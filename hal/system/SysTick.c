/*
 * SysTick.c
 *
 *  Created on: 31.03.2020
 *      Author: OK / ED
 */

#include "SysTick.h"

static volatile uint32_t sysTickTimer = 0;
static volatile uint8_t sysTickDivFlag = 0;

void __attribute__ ((interrupt))SysTick_Handler(void);

/* handler for SysTick interrupt */
void SysTick_Handler(void)
{
	if(sysTickDivFlag)
	{
		// 1ms timer for systick_getTimer()
		sysTickTimer++;

		sysTickDivFlag = 0;
	}
	else
	{
		sysTickDivFlag = 1;
	}
}

/* initialize SysTick timer */
void systick_init()
{
#if BOARD_CPU == STM32F205
	SysTick_Config(7500);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
#else
	#error "systick_init not defined for selected BOARD_CPU!"
#endif
}

/* get system timer in [ms] */
uint32_t systick_getTimer()
{
	return sysTickTimer;
}
