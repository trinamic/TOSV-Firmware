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
