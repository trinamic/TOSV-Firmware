/*
 * BLDC.c
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#include "BLDC.h"

	// === private variables ===

	// === implementation ===

void bldc_init()
{
}

/* main regulation function */
void bldc_processBLDC()
{
}

void bldc_updateHallSettings(uint8_t motor)
{
	if (motor != DEFAULT_MC)
		return;

	uint16_t hallModeSettings = 0;

	if (motorConfig.hallPolarity)
		hallModeSettings |= TMC4671_HALL_POLARITY_MASK;

	if (motorConfig.hallDirection)
		hallModeSettings |= TMC4671_HALL_DIRECTION_MASK;

	if (motorConfig.hallInterpolation)
		hallModeSettings |= TMC4671_HALL_INTERPOLATION_MASK;

	// update hall_mode
	TMC4671_FIELD_UPDATE(motor, TMC4671_HALL_MODE, TMC4671_HALL_MODE_MASK, TMC4671_HALL_MODE_SHIFT, hallModeSettings);

	// update phi_e offset
	TMC4671_FIELD_UPDATE(motor, TMC4671_HALL_PHI_E_PHI_M_OFFSET, TMC4671_HALL_PHI_E_OFFSET_MASK, TMC4671_HALL_PHI_E_OFFSET_SHIFT, motorConfig.hallPhiEOffset);
}

int bldc_getAdcI0Offset(u8 motor)
{
	motorConfig.adc_I0_offset = tmc4671_getAdcI0Offset(motor);
	return motorConfig.adc_I0_offset;
}

bool bldc_setAdcI0Offset(u8 motor, int offset)
{
	if(offset >= 0 && offset < 65536)
	{
		motorConfig.adc_I0_offset = offset;
		tmc4671_setAdcI0Offset(motor, motorConfig.adc_I0_offset);
		return true;
	}
	return false;
}

int bldc_getAdcI1Offset(u8 motor)
{
	motorConfig.adc_I1_offset = tmc4671_getAdcI1Offset(motor);
	return motorConfig.adc_I1_offset;
}

bool bldc_setAdcI1Offset(u8 motor, int offset)
{
	if(offset >= 0 && offset < 65536)
	{
		motorConfig.adc_I1_offset = offset;
		tmc4671_setAdcI1Offset(motor, motorConfig.adc_I1_offset);
		return true;
	}
	return false;
}

u8 bldc_getMotorPolePairs(u8 motor)
{
	motorConfig.motorPolePairs = tmc4671_getPolePairs(motor);
	return motorConfig.motorPolePairs;
}

void bldc_updateMotorPolePairs(u8 motor, int motorPolePairs)
{
	motorConfig.motorPolePairs = motorPolePairs;
	tmc4671_setPolePairs(motor, motorConfig.motorPolePairs);
}

void bldc_updateMaxMotorCurrent(u8 motor, int maxCurrent)
{
	motorConfig.maximumCurrent = maxCurrent;
	tmc4671_setTorqueFluxLimit_mA(motor, motorConfig.dualShuntFactor, maxCurrent);
}

s32 bldc_getMaxMotorCurrent()
{
	return motorConfig.maximumCurrent;
}

bool bldc_setMotorType(u8 motor, u8 type)
{
	if ((type == TMC4671_NO_MOTOR)
	  ||(type == TMC4671_SINGLE_PHASE_DC)
	  ||(type == TMC4671_TWO_PHASE_STEPPER)
	  ||(type == TMC4671_THREE_PHASE_BLDC))
	{
		motorConfig.motorType = type;
		tmc4671_setMotorType(motor, type);
		return true;
	}
	return false;
}

u8 bldc_getMotorType(u8 motor)
{
	motorConfig.motorType = tmc4671_getMotorType(motor);
	return motorConfig.motorType;
}

//bool bldc_setMotorDirection(u8 motor, u8 direction)
//{
//	if ((direction == 0) || (direction == 1))
//	{
//		motorConfig.shaftBit = direction;
//		return true;
//	}
//	return false;
//}

//u8 bldc_getMotorDirection(u8 motor)
//{
//	return motorConfig.shaftBit;
//}

bool bldc_setCommutationMode(u8 mode)
{
	if ((mode == COMM_MODE_FOC_DISABLED)
	 || (mode == COMM_MODE_FOC_OPEN_LOOP)
	 || (mode == COMM_MODE_FOC_DIGITAL_HALL))
	{
		motorConfig.commutationMode = mode;
		return true;
	}
	return false;
}

u8 bldc_getCommutationMode()
{
	return motorConfig.commutationMode;
}
