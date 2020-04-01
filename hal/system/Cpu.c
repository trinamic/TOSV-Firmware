/*
 * Cpu.c
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */
#include "Cpu.h"

void cpu_init()
{
#if	(BOARD_CPU==STM32F205)
	  if(FLASH_OB_GetBOR() != OB_BOR_LEVEL3)
	  {
		  FLASH_OB_Unlock();
		  FLASH_OB_BORConfig(OB_BOR_LEVEL3);
		  FLASH_OB_Launch();
		  FLASH_OB_Lock();
	  }

	  // do RCC and NVIC configuration
	  SystemInit();
#else
	#error "cpu_init() for BOARD_CPU not supported!"
#endif
}
