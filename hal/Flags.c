/*
 * Flags.c
 *
 *  Created on: 31.03.2020
 *      Author: ED / OK
 */

#include "Flags.h"

volatile uint32_t statusFlags[NUMBER_OF_MOTORS]; // error and status flags (Overvoltage, Overcurrent,...)

void flags_init(uint8_t motor)
{
	statusFlags[motor] = 0;
}

/* set status flags */
void flags_setStatusFlag(uint8_t motor, uint32_t flag)
{
	statusFlags[motor] |= flag;
}

/* clear status flag */
void flags_clearStatusFlag(uint8_t motor, uint32_t flag)
{
	statusFlags[motor] &= ~flag;
}

/* clear/set status flag */
void flags_setStatusFlagEnabled(uint8_t motor, uint32_t flag, bool enabled)
{
	if (enabled)
		statusFlags[motor] |= flag;
	else
		statusFlags[motor] &= ~flag;
}

/* check if status flag is set */
uint8_t flags_isStatusFlagSet(uint8_t motor, uint32_t flag)
{
	if(statusFlags[motor] & flag)
		return true;
	else
		return false;
}

/* reset error flags */
void flags_resetErrorFlags(uint8_t motor)
{
	flags_clearStatusFlag(motor, OVERCURRENT|OVERTEMPERATURE|HALLERROR|DRIVER_ERROR);
}

/* get status flags and afterwards reset error flags */
uint32_t flags_getAllStatusFlags(uint8_t motor)
{
	uint32_t flags = statusFlags[motor];
	// reset error flags
	flags_resetErrorFlags(motor);
	return flags;
}

/* get and afterwards reset error flags */
uint32_t flags_getErrorFlags(uint8_t motor)
{
	uint32_t flags = statusFlags[motor];
	uint32_t mask = OVERCURRENT|UNDERVOLTAGE|OVERTEMPERATURE|HALLERROR|DRIVER_ERROR;

	// return only error flags
	flags &= mask;
	// reset error flags
	flags_resetErrorFlags(motor);
    return flags;
}
