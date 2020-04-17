/*
 * BLDC.c
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#include "BLDC.h"
#include "hal/system/SysTick.h"
#include "hal/system/SystemInfo.h"
#include "hal/system/Debug.h"
#include <math.h>

	// === private variables ===

	// general information
	int16_t gActualMotorTemperature = 0;				// actual motor temperature
	int16_t	gActualSupplyVoltage = 0;					// actual supply voltage

	// torque regulation
	int64_t akkuActualTorqueFlux[NUMBER_OF_MOTORS];
	int32_t actualTorquePT1[NUMBER_OF_MOTORS];
	int32_t gTargetTorque[NUMBER_OF_MOTORS];
	int32_t gTargetFlux[NUMBER_OF_MOTORS];

	// velocity regulation
	int32_t	gDesiredVelocity[NUMBER_OF_MOTORS];			// requested target velocity
	int32_t gActualVelocity[NUMBER_OF_MOTORS];
	int32_t gTargetSpeed[NUMBER_OF_MOTORS];

	// pressure regulation
	int32_t	gDesiredPressure[NUMBER_OF_MOTORS];			// requested target pressure
	int32_t gActualPressure[NUMBER_OF_MOTORS];
	int32_t gTargetPressure[NUMBER_OF_MOTORS];
	PIControl pressurePID;

	// commutation mode
	uint8_t	gLastSetCommutationMode[NUMBER_OF_MOTORS];	// actual regulation mode

	// motion mode
	uint32_t gMotionMode[NUMBER_OF_MOTORS];

	// general information
	void bldc_checkSupplyVoltage(uint8_t motor);
	void bldc_checkMotorTemperature();


	// === implementation ===

void bldc_init()
{
	for (int i = 0; i < NUMBER_OF_MOTORS; i++)
	{
		// pid controller mode
		gMotionMode[i] = STOP_MODE;
		gLastSetCommutationMode[i] = 0xFF;

		// torque mode
		akkuActualTorqueFlux[i] = 0;
		actualTorquePT1[i] = 0;
		gTargetTorque[i] = 0;
		gTargetFlux[i] = 0;

		// velocity mode
		gDesiredVelocity[i] = 0;
		gActualVelocity[i] = 0;
		gTargetSpeed[i] = 0;

		// pressure mode
		gDesiredPressure[i] = 0;
		gActualPressure[i] = 0;
		gTargetPressure[i] = 0;
		pressurePID.errorSum = 0;

		// flags
		flags_init(i);
		flags_setStatusFlag(i, STOP_MODE);
	}
}

int32_t bldc_getTargetTorqueFromPressurePIRegulator(int32_t targetPressure, int32_t actualPressure, PIControl *pid, int32_t maxPressure, int32_t maxTorque)
{
	// limit the target pressure
	targetPressure = tmc_limitInt(targetPressure, 0, maxPressure);

	pid->pParam = motorConfig->pidPressure_P_param;
	pid->iParam = motorConfig->pidPressure_I_param;

	int32_t pDivisor = 256;
	int64_t iDivisor = 65536;

	pid->error = targetPressure-actualPressure;
	pid->errorSum += pid->error;

	// compute max possible errorSum to reach the max target velocity
	int64_t maxErrorSum = (pid->iParam == 0) ? 0: ((int64_t)maxTorque * iDivisor) / (int64_t)pid->iParam;

	// limit error sum to prevent chasing and stay in positiv area
	pid->errorSum = tmc_limitS64(pid->errorSum, 0, maxErrorSum);

	int pPart = (pid->pParam * pid->error) / pDivisor;
	int iPart = ((int64_t)pid->iParam * pid->errorSum) / iDivisor;
	pid->result = pPart + iPart;

	// limit the result to max alowed torque only in positive direction
	return tmc_limitInt(pid->result, 0, maxTorque);
}

int32_t bldc_getPressureErrorSum(uint8_t motor)
{
	UNUSED(motor);
	return pressurePID.errorSum; // [motor]
}

/* main regulation function */
void bldc_processBLDC()
{
	static uint32_t lastMsCheckTime = 0;

	uint32_t actualTime = systick_getTimer();

	if (actualTime != lastMsCheckTime)
	{
		systemInfo_incVelocityLoopCounter();

		for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++)
		{
			// do ventilator control
			tosv_process(&tosvConfig[motor]);

			bldc_checkSupplyVoltage(motor);
			bldc_checkMotorTemperature();
			bldc_checkCommutationMode(motor);

			// always read actual velocity with shaft bit correction
			int32_t shaftVelocityActual = tmc4671_readInt(motor, TMC4671_PID_VELOCITY_ACTUAL) / motorConfig[motor].motorPolePairs;
			if (motorConfig[motor].shaftBit == 0)
				shaftVelocityActual = -shaftVelocityActual;

			gActualVelocity[motor] = (motorConfig[motor].commutationMode == COMM_MODE_FOC_OPEN_LOOP) ? (rampGenerator[motor].rampVelocity) : shaftVelocityActual;

			// always read actual torque+flux and filter
			int32_t torqueFluxValue  = tmc4671_readInt(motor, TMC4671_PID_TORQUE_FLUX_ACTUAL);
			int16_t actualFluxRaw    = (torqueFluxValue & 0xFFFF);
			int16_t actualTorqueRaw  = ((torqueFluxValue >> 16) & 0xFFFF);

			if ((actualTorqueRaw > -32000) && (actualTorqueRaw < 32000) && (actualFluxRaw > -32000) && (actualFluxRaw < 32000))
			{
				int32_t actualCurrent = (((int32_t)actualTorqueRaw+(int32_t)actualFluxRaw) * (int32_t)motorConfig[motor].dualShuntFactor) / 256;

				if (motorConfig[motor].commutationMode != COMM_MODE_FOC_OPEN_LOOP)
				{
					// make shaft bit correction
					if (motorConfig[motor].shaftBit == 0)
						actualCurrent = -actualCurrent;
				}

				actualTorquePT1[motor] = tmc_filterPT1(&akkuActualTorqueFlux[motor], actualCurrent, actualTorquePT1[motor], 1/*4*/, 8);
			}

			// always read actual pressure
#ifdef USE_PRESSURE_SENSOR_1
			int32_t pressure = ((int32_t)tmcm_getModuleSpecificADCValue(PRESSURE_SENSOR_PIN)*100000)/2068-20000; // todo: adjustable offset parameter needed (ED)
#endif
			gActualPressure[motor] = (pressure > 0) ? pressure : 0;

			// ramp handling
			if (flags_isStatusFlagSet(motor, PRESSURE_MODE))
			{
				// no ramp for pressure up to now
				gTargetPressure[motor] = gDesiredPressure[motor];

				// pressure pi regulation
				gTargetTorque[motor] = bldc_getTargetTorqueFromPressurePIRegulator(gTargetPressure[motor], gActualPressure[motor], &pressurePID, motorConfig[motor].maxPressure, motorConfig[motor].maximumCurrent); // todo: adjustable MaxPressure parameter needed (ED)

				// update ramp generator for velocity control to keep actual velocity as ramp velocity
				rampGenerator[motor].targetVelocity = gActualVelocity[motor];
				rampGenerator[motor].rampVelocity = gActualVelocity[motor];
				gTargetSpeed[motor] = gActualVelocity[motor];
			}

			if (flags_isStatusFlagSet(motor, VELOCITY_MODE))
			{
				// ramp generator for velocity control
				rampGenerator[motor].targetVelocity = gDesiredVelocity[motor];
				tmc_linearRamp_computeRampVelocity(&rampGenerator[motor]);
				gTargetSpeed[motor] = rampGenerator[motor].rampVelocity;
			}

			if (flags_isStatusFlagSet(motor, TORQUE_MODE))
			{
				// update ramp generator for velocity control to keep actual velocity as ramp velocity
				rampGenerator[motor].targetVelocity = gActualVelocity[motor];
				rampGenerator[motor].rampVelocity = gActualVelocity[motor];
				gTargetSpeed[motor] = gActualVelocity[motor];
			}

			if (flags_isStatusFlagSet(motor, STOP_MODE))
			{
				// nothing to do here
			}
			else
			{
				if (motorConfig[motor].commutationMode == COMM_MODE_FOC_OPEN_LOOP)
				{
					tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_TORQUE);

					int32_t targetFlux = (gTargetSpeed[motor] == 0) ? 0 : motorConfig[motor].openLoopCurrent;

					// do not use shaft bit corrected here!
					TMC4671_FIELD_UPDATE(motor, TMC4671_PID_TORQUE_FLUX_TARGET, TMC4671_PID_FLUX_TARGET_MASK, TMC4671_PID_FLUX_TARGET_SHIFT, (targetFlux * 256) / (int32_t)motorConfig[motor].dualShuntFactor);

					// and no target torque
					TMC4671_FIELD_UPDATE(motor, TMC4671_PID_TORQUE_FLUX_TARGET, TMC4671_PID_TORQUE_TARGET_MASK, TMC4671_PID_TORQUE_TARGET_SHIFT, 0);

					// update target velocity (shaft bit corrected)
					int32_t shaftTargetVelocity = (motorConfig[motor].shaftBit == 0) ? -gTargetSpeed[motor] : gTargetSpeed[motor];
					tmc4671_writeInt(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, shaftTargetVelocity);
				}
				else if (motorConfig[motor].commutationMode == COMM_MODE_FOC_DIGITAL_HALL)
				{
					if (flags_isStatusFlagSet(motor, MODULE_INITIALIZED))
					{
						if (gMotionMode[motor] == VELOCITY_MODE)
						{
							// set new target velocity (shaft bit corrected)
							int32_t shaftTargetVelocity = (motorConfig[motor].shaftBit == 0) ? -gTargetSpeed[motor] : gTargetSpeed[motor];
							tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_TARGET, shaftTargetVelocity * motorConfig[motor].motorPolePairs);

							// update target flux (shaft bit corrected)
							int32_t shaftTargetFlux   = (motorConfig[motor].shaftBit == 0) ? -gTargetFlux[motor] : gTargetFlux[motor];
							TMC4671_FIELD_UPDATE(motor, TMC4671_PID_TORQUE_FLUX_TARGET, TMC4671_PID_FLUX_TARGET_MASK, TMC4671_PID_FLUX_TARGET_SHIFT, (shaftTargetFlux * 256) / (int32_t)motorConfig[motor].dualShuntFactor);
						}
						else if ((gMotionMode[motor] == TORQUE_MODE) || (gMotionMode[motor] == PRESSURE_MODE))
						{
							// set new target torque (shaft bit corrected)
							int32_t shaftTargetTorque = (motorConfig[motor].shaftBit == 0) ? -gTargetTorque[motor] : gTargetTorque[motor];
							TMC4671_FIELD_UPDATE(motor, TMC4671_PID_TORQUE_FLUX_TARGET, TMC4671_PID_TORQUE_TARGET_MASK, TMC4671_PID_TORQUE_TARGET_SHIFT, (shaftTargetTorque* 256) / (int32_t)motorConfig[motor].dualShuntFactor);

							// update target flux (shaft bit corrected)
							int32_t shaftTargetFlux   = (motorConfig[motor].shaftBit == 0) ? -gTargetFlux[motor] : gTargetFlux[motor];
							TMC4671_FIELD_UPDATE(motor, TMC4671_PID_TORQUE_FLUX_TARGET, TMC4671_PID_FLUX_TARGET_MASK, TMC4671_PID_FLUX_TARGET_SHIFT, (shaftTargetFlux * 256) / (int32_t)motorConfig[motor].dualShuntFactor);
						}
					}
				}
			}
		}
		lastMsCheckTime = actualTime;
	}
}

// ===== general info =====

int16_t bldc_getSupplyVoltage()
{
	return gActualSupplyVoltage;
}

/* observe over-/under-voltage and disable driver if necessary */
void bldc_checkSupplyVoltage(uint8_t motor)
{
	gActualSupplyVoltage = (VOLTAGE_FAKTOR*tmcm_getModuleSpecificADCValue(ADC_VOLTAGE))/4095;

	if (gActualSupplyVoltage >= MAX_SUPPLY_VOLTAGE)
	{
		flags_setStatusFlag(motor, OVERVOLTAGE);
	}
	else if (gActualSupplyVoltage <= MIN_SUPPLY_VOLTAGE)
	{
		flags_setStatusFlag(motor, UNDERVOLTAGE);
		flags_clearStatusFlag(motor, OVERVOLTAGE);
	}
	else if (gActualSupplyVoltage > ON__SUPPLY_VOLTAGE)
	{
		flags_clearStatusFlag(motor, OVERVOLTAGE);
		flags_clearStatusFlag(motor, UNDERVOLTAGE);
	}
}

int16_t bldc_getMotorTemperature()
{
	return gActualMotorTemperature;
}

void bldc_checkMotorTemperature()
{
	float vTherm = tmcm_getModuleSpecificADCValue(ADC_MOT_TEMP)*3.3 / 4095.0;
	float rNTC = (3.3-vTherm)/(vTherm/4700.0);
	float b = 3455.0;
	float temp = (1.0/((log(rNTC/10000.0)/b) + (1.0/298.16)))-273.16;
	gActualMotorTemperature = temp;

	for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++)
	{
		if(gActualMotorTemperature >= MAX_CRITICAL_TEMP)
		{
			flags_setStatusFlag(motor, OVERTEMPERATURE);

			// hold module in stop mode to suppress regulation
			bldc_switchToRegulationMode(motor, STOP_MODE);
			tmcm_disableDriver(motor);
		}
		else if(gActualMotorTemperature <= MIN_CRITICAL_TEMP)
		{
			flags_clearStatusFlag(motor, OVERTEMPERATURE);
		}
	}
}

// ===== ADC offset configuration =====

uint16_t bldc_getAdcI0Offset(uint8_t motor)
{
	motorConfig[motor].adc_I0_offset = tmc4671_getAdcI0Offset(motor);
	return motorConfig[motor].adc_I0_offset;
}

void bldc_setAdcI0Offset(uint8_t motor, uint16_t offset)
{
	motorConfig[motor].adc_I0_offset = offset;
	tmc4671_setAdcI0Offset(motor, motorConfig[motor].adc_I0_offset);
}

uint16_t bldc_getAdcI1Offset(uint8_t motor)
{
	motorConfig[motor].adc_I1_offset = tmc4671_getAdcI1Offset(motor);
	return motorConfig[motor].adc_I1_offset;
}

void bldc_setAdcI1Offset(uint8_t motor, uint16_t offset)
{
	motorConfig[motor].adc_I1_offset = offset;
	tmc4671_setAdcI1Offset(motor, motorConfig[motor].adc_I1_offset);
}

// ===== motor settings =====

uint8_t bldc_getMotorPolePairs(uint8_t motor)
{
	return motorConfig[motor].motorPolePairs;
}

void bldc_updateMotorPolePairs(uint8_t motor, uint8_t motorPolePairs)
{
	motorConfig[motor].motorPolePairs = motorPolePairs;
	tmc4671_setPolePairs(motor, motorConfig[motor].motorPolePairs);
}

uint16_t bldc_getMaxMotorCurrent(uint8_t motor)
{
	return motorConfig[motor].maximumCurrent;
}

void bldc_updateMaxMotorCurrent(uint8_t motor, uint16_t maxCurrent)
{
	motorConfig[motor].maximumCurrent = maxCurrent;
	tmc4671_setTorqueFluxLimit_mA(motor, motorConfig[motor].dualShuntFactor, maxCurrent);
}

uint8_t bldc_getMotorDirection(uint8_t motor)
{
	return motorConfig[motor].shaftBit;
}

bool bldc_setMotorDirection(uint8_t motor, uint8_t direction)
{
	if ((direction == 0) || (direction == 1))
	{
		motorConfig[motor].shaftBit = direction;
		return true;
	}
	return false;
}

uint8_t bldc_getCommutationMode(uint8_t motor)
{
	return motorConfig[motor].commutationMode;
}

bool bldc_setCommutationMode(uint8_t motor, uint8_t mode)
{
	if ((mode == COMM_MODE_FOC_DISABLED)
	 || (mode == COMM_MODE_FOC_OPEN_LOOP)
	 || (mode == COMM_MODE_FOC_DIGITAL_HALL))
	{
		motorConfig[motor].commutationMode = mode;
		return true;
	}
	return false;
}

// ===== torque mode settings =====

int bldc_getTargetMotorCurrent(uint8_t motor)
{
	if (motorConfig[motor].commutationMode == COMM_MODE_FOC_OPEN_LOOP)
	{
		// do not make shaft bit correction
		return tmc4671_getTargetTorqueFluxSum_mA(motor, motorConfig[motor].dualShuntFactor);
	} else {
		// make shaft bit correction
		if (motorConfig[motor].shaftBit == 0)
			return -tmc4671_getTargetTorqueFluxSum_mA(motor, motorConfig[motor].dualShuntFactor);
		else
			return tmc4671_getTargetTorqueFluxSum_mA(motor, motorConfig[motor].dualShuntFactor);
	}
}

bool bldc_setTargetMotorCurrent(uint8_t motor, int32_t targetCurrent)
{
	if ((motorConfig[motor].commutationMode == COMM_MODE_FOC_DISABLED)
	  ||(motorConfig[motor].commutationMode == COMM_MODE_FOC_OPEN_LOOP))
		return false;

	if((targetCurrent >= -MAX_CURRENT) && (targetCurrent <= MAX_CURRENT))
	{
		gTargetTorque[motor] = targetCurrent;

		// switch to torque mode
		bldc_switchToRegulationMode(motor, TORQUE_MODE);
		return true;
	}
	return false;
}

int32_t bldc_getActualMotorCurrent(uint8_t motor)
{
	return actualTorquePT1[motor];
}

// ===== velocity mode settings =====

int32_t bldc_getTargetVelocity(uint8_t motor)
{
	return gDesiredVelocity[motor];
}

/* set target velocity [rpm] (x{>0:CW | 0:Stop | <0: CCW} */
bool bldc_setTargetVelocity(uint8_t motor, int32_t velocity)
{
	if (motorConfig[motor].commutationMode == COMM_MODE_FOC_DISABLED)
		return false;

	if ((velocity >= -MAX_VELOCITY) && (velocity <= MAX_VELOCITY))
	{
		gDesiredVelocity[motor] = velocity;

		// switch to velocity motion mode
		bldc_switchToRegulationMode(motor, VELOCITY_MODE);
		return true;
	}
	return false;
}

/* actual ramp generator velocity */
int32 bldc_getRampGeneratorVelocity(uint8_t motor)
{
	return gTargetSpeed[motor];
}

/* actual velocity in rpm */
int32_t bldc_getActualVelocity(uint8_t motor)
{
	return gActualVelocity[motor];
}

bool bldc_setMaxVelocity(uint8_t motor, int32_t maxVelocity)
{
	if((maxVelocity >= 0) && (maxVelocity <= MAX_VELOCITY))
	{
		motorConfig[motor].maxVelocity = maxVelocity;
		rampGenerator[motor].maxVelocity = maxVelocity;
		tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_LIMIT, maxVelocity * motorConfig[motor].motorPolePairs);
		return true;
	}
	return false;
}

bool bldc_setAcceleration(uint8_t motor, int32_t acceleration)
{
	if((acceleration >= 0) && (acceleration <= MAX_ACCELERATION))
	{
		motorConfig[motor].acceleration = acceleration;
		rampGenerator[motor].acceleration = acceleration;
		return true;
	}
	return false;
}

bool bldc_setRampEnabled(uint8_t motor, int32_t enableRamp)
{
	if((enableRamp == 0) || (enableRamp == 1))
	{
		motorConfig[motor].useVelocityRamp = enableRamp;
		rampGenerator[motor].rampEnabled = enableRamp;
		return true;
	}
	return false;
}

// ===== pi controller mode settings =====

void bldc_switchToRegulationMode(uint8_t motor, uint32_t mode)
{
	flags_clearStatusFlag(motor, STOP_MODE | PRESSURE_MODE | TORQUE_MODE | VELOCITY_MODE | POSITION_END);

	switch (mode)
	{
		case PRESSURE_MODE:
			flags_setStatusFlag(motor, PRESSURE_MODE);
			gMotionMode[motor] = PRESSURE_MODE;
			tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_TORQUE);
			break;
		case VELOCITY_MODE:
			flags_setStatusFlag(motor, VELOCITY_MODE);
			gMotionMode[motor] = VELOCITY_MODE;
			tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_VELOCITY);
			break;
		case TORQUE_MODE:
			flags_setStatusFlag(motor, TORQUE_MODE);
			gMotionMode[motor] = TORQUE_MODE;
			tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_TORQUE);
			break;
		case STOP_MODE:
			flags_setStatusFlag(motor, STOP_MODE);
			gMotionMode[motor] = STOP_MODE;
			break;
	}
}

// ===== hall sensor settings =====

void bldc_updateHallSettings(uint8_t motor)
{
	uint16_t hallModeSettings = 0;

	if (motorConfig[motor].hallPolarity)
		hallModeSettings |= TMC4671_HALL_POLARITY_MASK;

	if (motorConfig[motor].hallDirection)
		hallModeSettings |= TMC4671_HALL_DIRECTION_MASK;

	if (motorConfig[motor].hallInterpolation)
		hallModeSettings |= TMC4671_HALL_INTERPOLATION_MASK;

	// update hall_mode
	TMC4671_FIELD_UPDATE(motor, TMC4671_HALL_MODE, TMC4671_HALL_MODE_MASK, TMC4671_HALL_MODE_SHIFT, hallModeSettings);

	// update phi_e offset
	TMC4671_FIELD_UPDATE(motor, TMC4671_HALL_PHI_E_PHI_M_OFFSET, TMC4671_HALL_PHI_E_OFFSET_MASK, TMC4671_HALL_PHI_E_OFFSET_SHIFT, motorConfig[motor].hallPhiEOffset);
}

// ===== general motor control mode handling =====

void bldc_checkCommutationMode(uint8_t motor)
{
	if(gLastSetCommutationMode[motor] != motorConfig[motor].commutationMode)
	{
		switch(motorConfig[motor].commutationMode)
		{
			case COMM_MODE_FOC_DISABLED:
				tmc4671_writeInt(motor, TMC4671_UQ_UD_EXT, 0);
				tmc4671_writeInt(motor, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_EXTERNAL);
				tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_UQ_UD_EXT);
				tmc4671_writeInt(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, 0);
				flags_clearStatusFlag(motor, MODULE_INITIALIZED);
				break;
			case COMM_MODE_FOC_OPEN_LOOP:
				tmc4671_writeInt(motor, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_OPEN_LOOP);
				tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_TORQUE);
				flags_setStatusFlag(motor, MODULE_INITIALIZED);
				break;
			case COMM_MODE_FOC_DIGITAL_HALL:
				tmc4671_writeInt(motor, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_HALL);

				if (flags_isStatusFlagSet(motor, STOP_MODE))
					tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_STOPPED);
				else if (flags_isStatusFlagSet(motor, TORQUE_MODE))
					tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_TORQUE);
				else if (flags_isStatusFlagSet(motor, PRESSURE_MODE))
					tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_TORQUE);
				else	// velocity mode
					tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_VELOCITY);

				flags_setStatusFlag(motor, MODULE_INITIALIZED);
				break;
		}
		gLastSetCommutationMode[motor] = motorConfig[motor].commutationMode;
	}
}

int32 bldc_getTargetPressure(uint8_t motor)
{
	return gDesiredPressure[motor];
}

int32 bldc_getRampPressure(uint8_t motor)
{
	return gTargetPressure[motor];
}

bool bldc_setTargetPressure(uint8_t motor, int32_t targetPressure)
{
	if ((motorConfig[motor].commutationMode == COMM_MODE_FOC_DISABLED)
	  ||(motorConfig[motor].commutationMode == COMM_MODE_FOC_OPEN_LOOP))
		return false;

	if((targetPressure >= 0) && (targetPressure <= motorConfig[motor].maxPressure))
	{
		gDesiredPressure[motor] = targetPressure;

		// switch to velocity mode
		bldc_switchToRegulationMode(motor, PRESSURE_MODE);
		return true;
	}
	return false;
}

int32_t bldc_getActualPressure(uint8_t motor) // unit: Pa
{
	return gActualPressure[motor];
}
