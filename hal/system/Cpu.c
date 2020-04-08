/*
 * Cpu.c
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */
#include "Cpu.h"

void cpu_init()
{
#if (BOARD_CPU==STM32F103)

	// configure system clock
	ErrorStatus HSEStartUpStatus;

	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

	    /* PLLCLK = 16MHz / 2 * 9 = 72 MHz */
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08) {}
	}
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// configure NVIC
#ifdef  VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
	#if defined(BOOTLOADER)
	//Set the vector table base location at 0x08004000 (right after the TMCM boot loader)
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);
  	#else
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
  	#endif
#endif
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	asm volatile("CPSID I\n");//io_disableInterrupts();
	asm volatile("CPSIE I\n");//io_enableInterrupts();

#elif (BOARD_CPU==STM32F205)
	  if(FLASH_OB_GetBOR() != OB_BOR_LEVEL3)
	  {
		  FLASH_OB_Unlock();
		  FLASH_OB_BORConfig(OB_BOR_LEVEL3);
		  FLASH_OB_Launch();
		  FLASH_OB_Lock();
	  }

	  // do RCC and NVIC configuration
	  SystemInit();

	  // enable interrupts globally
	  __enable_irq();

#else
	#error "cpu_init() for BOARD_CPU not supported!"
#endif
}
