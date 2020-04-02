/*
 * Definitions.h
 *
 *  Created on: 31.03.2020
 *      Author: ED / OK
 */

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

	#include "TMC-API/tmc/helpers/API_Header.h"

	#define DEFAULT_MC  0
	#define DEFAULT_DRV 1

	#include "modules/SelectModule.h"

	typedef struct
	{
		uint8_t baudrate;
		uint8_t serialModuleAddress;
		uint8_t serialHostAddress;
	} TModuleConfig;

	typedef struct
	{
		uint32_t maximumCurrent;
		uint32_t openLoopCurrent;
		uint32_t pwm_freq;
		int32_t maxPositioningSpeed;
		int32_t acceleration;

		uint16_t adc_I0_offset;
		uint16_t adc_I1_offset;
		uint16_t dualShuntFactor;		// u8.u8

		uint16_t pidTorque_P_param;
		uint16_t pidTorque_I_param;
		uint16_t pidVelocity_P_param;
		uint16_t pidVelocity_I_param;

		uint8_t motorType;
		uint8_t motorPolePairs;
		uint8_t commutationMode;
		uint8_t useVelocityRamp;

		uint8_t hallPolarity;
		uint8_t hallDirection;
		int16_t hallPhiEOffset;
		uint8_t hallInterpolation;
	} TMotorConfig;

	TModuleConfig moduleConfig;
	TMotorConfig motorConfig;

	// commutation modes
	#define COMM_MODE_FOC_DISABLED			0
	#define COMM_MODE_FOC_OPEN_LOOP			1
	#define COMM_MODE_FOC_DIGITAL_HALL		2

	// init functions
	extern void tmcm_initModuleConfig();
	extern void tmcm_initMotorConfig();
	extern void tmcm_initModuleSpecificIO();
	extern void tmcm_updateConfig();

	// LEDs
	extern void tmcm_led_run_toggle();

	// driver control
	extern void tmcm_enableDriver(uint8_t motor);
	extern void tmcm_disableDriver(uint8_t motor);
	extern uint8_t tmcm_getDriverState(uint8_t motor);

	// chipset control
	extern void tmcm_enableCsWeasel(uint8_t motor);
	extern void tmcm_disableCsWeasel(uint8_t motor);
	extern void tmcm_enableCsDragon(uint8_t motor);
	extern void tmcm_disableCsDragon(uint8_t motor);

	// IOs
	extern void tmcm_clearModuleSpecificIOPin(uint8_t pin);
	extern void tmcm_setModuleSpecificIOPin(uint8_t pin);
	extern uint8_t tmcm_getModuleSpecificIOPinStatus(uint8_t pin);
	extern uint16_t tmcm_getModuleSpecificADCValue(uint8_t pin);

#endif /* DEFINITIONS_H */
