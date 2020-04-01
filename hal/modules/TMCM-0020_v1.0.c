/*
 * TMCM-0020.c
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#include "TMCM-0020_v1.0.h"

#if DEVICE==TMCM_0020_V10

// general module settings
const char *VersionString="0020V100";
uint32_t PLL_M = 8;
uint32_t PLL_N = 120;

void tmcm_initModuleConfig()
{
	moduleConfig.baudrate 				= 7; // UART 115200bps
	moduleConfig.serialModuleAddress 	= 1;
	moduleConfig.serialHostAddress		= 2;
}

void tmcm_initMotorConfig()
{
	motorConfig.maxPositioningSpeed = 4000;
	motorConfig.maximumCurrent 		= 1000;
}

/* expected by the libTMC library for io_init()*/
void tmcm_initModuleSpecificIO()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// === enable port A ===
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// === enable port B ===
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// === enable port C ===
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // === enable port D ===
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// === enable port E ===
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// outputs port E
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIOE->BSRRH = GPIO_Pin_0;
	GPIOE->BSRRL = GPIO_Pin_1;

	// === enable dac clock ===
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
}

void tmcm_led_run_toggle()
{
	GPIOE->ODR ^= GPIO_Pin_0;
}

void tmcm_enableDriver()
{
//	GPIOB->BSRRL = BIT12;
}

void tmcm_disableDriver()
{
//	GPIOB->BSRRH = BIT12;
}

uint8_t tmcm_getDriverState()
{
	return 0;//(GPIOB->IDR & BIT12);
}

#endif
