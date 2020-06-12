/*
 * main.c
 *
 *  Created on: 31.03.2020
 *      Author: ED / OK
 */

#include "hal/system/Cpu.h"
#include "hal/comm/Eeprom.h"
#include "hal/system/SysTick.h"
#include "hal/system/SystemInfo.h"
#include "hal/comm/SPI.h"
#include "hal/comm/I2C.h"
#include "BLDC.h"
#include "TMCL.h"
#include "TOSV.h"

#if defined(USE_UART_INTERFACE)
	#include "hal/comm/UART.h"
#endif

#if defined(USE_USB_INTERFACE)
	#include "hal/comm/USB.h"
#endif

int main(void)
{
	cpu_init();

	// load default values of the module
	tmcm_initModuleConfig();
	tmcm_initMotorConfig();

	// initialize periphery
	tmcm_initModuleSpecificIO();
	tmcm_initModuleSpecificADC();

	// initialize hal functionality
	systick_init();

	spi_init();
	eeprom_initConfig();

	InitIIC();

	// initialize communication interfaces
#ifdef USE_UART_INTERFACE
	uart_init(moduleConfig.baudrate);
#endif

#ifdef USE_RS485_INTERFACE
	rs485_init(moduleConfig.baudrate);
#endif

#ifdef USE_USB_INTERFACE
	usb_init();
#endif

	// initialize software
	tmcl_init();
	bldc_init();

	// initialize ICs
	tmcm_updateConfig();
	tosv_initFlowSensor();

	for(;;)
	{
		systemInfo_incMainLoopCounter();

		// process incoming tmcl commands
		tmcl_processCommand();

		// do motion control
		bldc_processBLDC();

		// I am alive LED
		static uint32_t ledCounterCheckTime = 0;
		if (abs(systick_getTimer()-ledCounterCheckTime) > 1000)
		{
			tmcm_led_run_toggle();
			ledCounterCheckTime = systick_getTimer();
		}

		systemInfo_update(systick_getTimer());
	}
	return 0;
}
