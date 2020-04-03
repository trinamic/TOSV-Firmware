/*
 * BLDC.h
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#ifndef BLDC_H
#define BLDC_H

	#include "Definitions.h"

	void bldc_init();
	void bldc_processBLDC();
	void bldc_updateHallSettings(uint8_t motor);


	bool bldc_setMotorType(u8 motor, u8 mode);
	u8 bldc_getMotorType(u8 motor);

	bool bldc_setMotorDirection(u8 motor, u8 direction);
	u8 bldc_getMotorDirection(u8 motor);

	bool bldc_setCommutationMode(u8 mode);
	u8 bldc_getCommutationMode();


	int bldc_getAdcI0Offset(u8 motor);
	bool bldc_setAdcI0Offset(u8 motor, int offset);

	int bldc_getAdcI1Offset(u8 motor);
	bool bldc_setAdcI1Offset(u8 motor, int offset);

	void bldc_updateMotorPolePairs(u8 motor, int motorPolePairs);
	u8 bldc_getMotorPolePairs(u8 motor);

	void bldc_updateMaxMotorCurrent(u8 motor, int maxCurrent);
	s32 bldc_getMaxMotorCurrent();



#endif
