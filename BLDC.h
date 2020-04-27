/*
 * BLDC.h
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#ifndef BLDC_H
#define BLDC_H

	#include "Definitions.h"
	#include "hal/modules/SelectModule.h"

	void bldc_init();
	void bldc_processBLDC();
	void bldc_updateHallSettings(uint8_t motor);

	// ===== general info =====
	int16_t bldc_getSupplyVoltage();
	int16_t bldc_getMotorTemperature();
	int32_t bldc_getFlowValue();
	int32_t bldc_getVolumeValue();
	void bldc_zeroFlow();
	void bldc_resetVolumeIntegration();

	// ===== ADC offset configuration =====
	uint16_t bldc_getAdcI0Offset(uint8_t motor);
	void bldc_setAdcI0Offset(uint8_t motor, uint16_t offset);

	uint16_t bldc_getAdcI1Offset(uint8_t motor);
	void bldc_setAdcI1Offset(uint8_t motor, uint16_t offset);

	// ===== motor settings =====
	uint8_t bldc_getMotorPolePairs(uint8_t motor);
	void bldc_updateMotorPolePairs(uint8_t motor, uint8_t motorPolePairs);

	uint16_t bldc_getMaxMotorCurrent(uint8_t motor);
	void bldc_updateMaxMotorCurrent(uint8_t motor, uint16_t maxCurrent);

	uint8_t bldc_getMotorDirection(uint8_t motor);
	bool bldc_setMotorDirection(uint8_t motor, uint8_t direction);

	uint8_t bldc_getCommutationMode(uint8_t motor);
	bool bldc_setCommutationMode(uint8_t motor, uint8_t mode);

	// ===== torque mode settings =====
	int bldc_getTargetMotorCurrent(uint8_t motor);
	bool bldc_setTargetMotorCurrent(uint8_t motor, int32_t targetCurrent);
	int32_t bldc_getActualMotorCurrent(uint8_t motor);

	// ===== velocity mode settings =====
	int32_t bldc_getTargetVelocity(uint8_t motor);
	bool bldc_setTargetVelocity(uint8_t motor, int32_t velocity);
	int32 bldc_getRampGeneratorVelocity(uint8_t motor);
	int32_t bldc_getActualVelocity(uint8_t motor);
	bool bldc_setMaxVelocity(uint8_t motor, int32_t maxVelocity);
	bool bldc_setAcceleration(uint8_t motor, int32_t acceleration);
	bool bldc_setRampEnabled(uint8_t motor, int32_t enableRamp);

	// ===== pressure control mode settings =====
	int32 bldc_getTargetPressure(uint8_t motor);
	int32 bldc_getRampPressure(uint8_t motor);
	bool bldc_setTargetPressure(uint8_t motor, int32_t pressure);
	int32 bldc_getActualPressure(uint8_t motor);
	int32_t bldc_getPressureErrorSum(uint8_t motor);

	// ===== pi controller mode settings =====
	void bldc_switchToRegulationMode(uint8_t motor, uint32_t mode);

	// ===== general motor control mode handling =====
	void bldc_checkCommutationMode(uint8_t motor);

#endif
