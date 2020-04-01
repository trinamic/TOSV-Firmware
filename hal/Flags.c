/*
 * Flags.c
 *
 *  Created on: 31.03.2020
 *      Author: ED / OK
 */

#include "Flags.h"

volatile uint32_t statusFlags = 0;	// error and status flags (Overvoltage, Overcurrent,...)

/* set status flags */
void flags_setStatusFlag(uint32_t flag)
{
	statusFlags |= flag;
}

/* clear status flag */
void flags_clearStatusFlag(uint32_t flag)
{
	statusFlags &= ~flag;
}

/* clear/set status flag */
void flags_setStatusFlagEnabled(uint32_t flag, bool enabled)
{
	if (enabled)
		statusFlags |= flag;
	else
		statusFlags &= ~flag;
}

/* check if status flag is set */
uint8_t flags_isStatusFlagSet(uint32_t flag)
{
	if(statusFlags & flag)
		return true;
	else
		return false;
}

/* reset error flags */
void flags_resetErrorFlags()
{
	flags_clearStatusFlag(OVERCURRENT|OVERTEMPERATURE|HALLERROR|DRIVER_ERROR);
}

/* get status flags and afterwards reset error flags */
uint32_t flags_getAllStatusFlags()
{
	uint32_t flags = statusFlags;
	// reset error flags
	flags_resetErrorFlags();
	return flags;
}

/* get and afterwards reset error flags */
uint32_t flags_getErrorFlags()
{
	uint32_t flags = statusFlags;
	uint32_t mask = OVERCURRENT|UNDERVOLTAGE|OVERTEMPERATURE|HALLERROR|DRIVER_ERROR;

	// return only error flags
	flags &= mask;
	// reset error flags
	flags_resetErrorFlags();
    return flags;
}
