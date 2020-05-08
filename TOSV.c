/*
 * TOSV.c
 *
 *  Created on: 8.4.2020
 *      Author: OK / ED
 */

#include "TOSV.h"
#include "BLDC.h"

// private variables

int32_t gActualFlowValue = 0;
int32_t gActualFlowValuePT1 = 0;
int64_t gActualFlowValueAccu = 0;
int32_t gAcutalVolume = 0;
int32_t gFlowOffset = 0;
int64_t gFlowSum = 0;
int32_t gVolumeMax = 0;

bool gIsFlowSensorPresent = false; // don't crash the system if pressure sensor for flow measurement is not present

// private function declarations

void tosv_process_pressure_control(TOSV_Config *config);
void tosv_process_volume_control(TOSV_Config *config);

bool tosv_hasAsbTrigger(TOSV_Config *config);

// public function implementations

void tosv_init(TOSV_Config *config)
{
	config->actualState 		= TOSV_STATE_STOPPED;
	config->timer 				= 0;
	config->tStartup 			= 1000;
	config->tInhalationRise 	= 1000;
	config->tInhalationPause 	= 1000;
	config->tExhalationFall 	= 1000;
	config->tExhalationPause 	= 1000;
	config->pLIMIT				= 20000;
	config->pPEEP 				= 2000;
	config->mode				= TOSV_MODE_PRESSURE_CONTROL;
	config->asbEnable 			= false;
	config->asbThreshold        = 500;
	config->asbVolumeCondition  = 70;
}

void tosv_initFlowSensor()
{
	// try writing to flow sensor to check its presence
	uint8_t writeData[] = {0x30};
	uint8_t isI2cWriteSuccessful = I2C_Master_BufferWrite(I2C1, writeData, sizeof(writeData), 0xD8);

	// if not, don't read out the flow sensor cyclically
	gIsFlowSensorPresent = (bool)isI2cWriteSuccessful;
}

void tosv_enableVentilator(TOSV_Config *config, bool enable)
{
	if (enable)
	{
		// only start if not running
		if (config->actualState == TOSV_STATE_STOPPED)
		{
			config->actualState = TOSV_STATE_STARTUP;
			tosv_zeroFlow();
		}
	} else {
		config->actualState = TOSV_STATE_STOPPED;
		bldc_setTargetPressure(0, 0);
	}
}

bool tosv_isVentilatorEnabled(TOSV_Config *config)
{
	return (config->actualState != TOSV_STATE_STOPPED);
}

void tosv_process(TOSV_Config *config)
{
	switch (config->mode)
	{
		case TOSV_MODE_PRESSURE_CONTROL:
			tosv_process_pressure_control(config);
			break;
		case TOSV_MODE_VOLUME_CONTROL:
			tosv_process_volume_control(config);
			break;
		default:
			break;
	}
}

int32_t tosv_getFlowValue()
{
	return gActualFlowValuePT1;
}


void tosv_zeroFlow()
{
	gFlowOffset = gActualFlowValue;
}

void tosv_resetVolumeIntegration()
{
	gFlowSum = 0;
	gVolumeMax = 0;
}

/* Read out SM9333 I2C pressure sensor value and calculate flow value from it.
 *
 * In the first step the address (0x30) to be read from is send to the sensor via write.
 * Then the pressure sensor value (0x30) and sync'ed status word (0x32) is retrieved via read.
 * For now the status word is not processed in any way.
 *
 * The SM9333 comprises also a temperature sensor for temperature compensation if necessary.
 *
 * https://www.si-micro.com/fileadmin/00_smi_relaunch/products/digital/datasheet/SM933X_datasheet.pdf
 *
 * We constantly filled a 120 liter garbage bag for rough calibration. It took 2 minutes until
 * filled with air and we read a sensor count of 32000 during filling, thus we
 * approximate 1 count to 2 ml/min.
 *
 * Please beware that this is not very accurate!
 */
void tosv_updateFlowSensor()
{
	if (gIsFlowSensorPresent)
	{
		uint8_t writeData[] = {0x30};
		uint16_t readData[1];
		uint8_t isI2cWriteSuccessful;

		isI2cWriteSuccessful = I2C_Master_BufferWrite(I2C1, writeData, sizeof(writeData), 0xD8);

		if (isI2cWriteSuccessful)
		{
			I2C_Master_BufferRead(I2C1, (uint8_t*)readData, sizeof(readData), 0xD8);

			int16_t pressureSensorCount = readData[0];
			gActualFlowValue = (int32_t)pressureSensorCount * 2;
			gActualFlowValuePT1 = tmc_filterPT1(&gActualFlowValueAccu, (gActualFlowValue-gFlowOffset), gActualFlowValuePT1, 5, 8);
		}
	}
}

/* Volume is given in ml.
 *
 * As flow is ml/min and cycle time is 1 ms we need to divide the sum by 60000 (min -> s -> ms)
 */
int32_t tosv_updateVolume(uint8_t motor)
{
	UNUSED(motor);

	if (gIsFlowSensorPresent)
	{
		gFlowSum += (gActualFlowValue-gFlowOffset);

		gAcutalVolume = gFlowSum / 60000;

		if (gAcutalVolume > gVolumeMax)
			gVolumeMax = gAcutalVolume;

		return gAcutalVolume;
	}
	else
	{
		return 0;
	}
}

// private function implementations

/*
 * state machine for pressure based control
 */
void tosv_process_pressure_control(TOSV_Config *config)
{
	config->timer++;

	switch(config->actualState)
	{
		case TOSV_STATE_STOPPED:
			// reset timer
			config->timer = 0;
			tosv_resetVolumeIntegration();
			break;
		case TOSV_STATE_STARTUP:
			bldc_setTargetPressure(0, 0 + (config->pPEEP*config->timer)/config->tStartup);
			tosv_resetVolumeIntegration();
			if (config->timer >= config->tStartup)
			{
				config->actualState = TOSV_STATE_INHALATION_RISE;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_INHALATION_RISE:
			bldc_setTargetPressure(0, config->pPEEP + ((config->pLIMIT-config->pPEEP)*config->timer)/config->tInhalationRise);
			if (config->timer >= config->tInhalationRise)
			{
				config->actualState = TOSV_STATE_INHALATION_PAUSE;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_INHALATION_PAUSE:
			bldc_setTargetPressure(0, config->pLIMIT);
			if (config->timer >= config->tInhalationPause)
			{
				config->actualState = TOSV_STATE_EXHALATION_FALL;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_EXHALATION_FALL:
			bldc_setTargetPressure(0, config->pPEEP + ((config->pLIMIT-config->pPEEP)*(config->tExhalationFall-config->timer))/config->tExhalationFall);
			if (config->timer >= config->tExhalationFall)
			{
				config->actualState = TOSV_STATE_EXHALATION_PAUSE;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_EXHALATION_PAUSE:
			bldc_setTargetPressure(0, config->pPEEP);
			if ((config->timer >= config->tExhalationPause) || (tosv_hasAsbTrigger(config)))
			{
				config->actualState = TOSV_STATE_INHALATION_RISE;
				config->timer = 0;
				tosv_resetVolumeIntegration();
			}
			break;
	}
}


/*
 * state machine for volume based control
 */
void tosv_process_volume_control(TOSV_Config *config)
{
	config->timer++;

	switch(config->actualState)
	{
		case TOSV_STATE_STOPPED:
			// reset timer
			config->timer = 0;
			tosv_resetVolumeIntegration();
			break;
		case TOSV_STATE_STARTUP:
			bldc_setTargetVolume(0, 0);
			tosv_resetVolumeIntegration();
			if (config->timer >= config->tStartup)
			{
				config->actualState = TOSV_STATE_INHALATION_RISE;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_INHALATION_RISE:
			bldc_setTargetVolume(0, (config->volumeMax*config->timer)/config->tInhalationRise);
			if (config->timer >= config->tInhalationRise)
			{
				config->actualState = TOSV_STATE_INHALATION_PAUSE;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_INHALATION_PAUSE:
			bldc_setTargetVolume(0, config->volumeMax);
			if (config->timer >= config->tInhalationPause)
			{
				config->actualState = TOSV_STATE_EXHALATION_FALL;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_EXHALATION_FALL:
			bldc_setTargetVolume(0, (config->volumeMax)*(config->tExhalationFall-config->timer)/config->tExhalationFall);
			if (config->timer >= config->tExhalationFall)
			{
				config->actualState = TOSV_STATE_EXHALATION_PAUSE;
				config->timer = 0;
			}
			break;
		case TOSV_STATE_EXHALATION_PAUSE:
			bldc_setTargetVolume(0, 0);
			if ((config->timer >= config->tExhalationPause) || (tosv_hasAsbTrigger(config)))
			{
				config->actualState = TOSV_STATE_INHALATION_RISE;
				config->timer = 0;
				tosv_resetVolumeIntegration();
			}
			break;
	}
}


bool tosv_hasAsbTrigger(TOSV_Config *config)
{
	if (config->asbEnable)
	{
		uint32_t acutalVolumePercent = gAcutalVolume*100/gVolumeMax;

		return ((gActualFlowValue > config->asbThreshold) && (acutalVolumePercent <= config->asbVolumeCondition));
	}
	else
	{
		return false;
	}
}
