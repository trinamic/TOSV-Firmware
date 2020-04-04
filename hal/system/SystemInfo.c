/*
 * SystemInfo.c
 *
 *  Created on: 04.04.2020
 *      Author: ED
 */
#include "SystemInfo.h"

uint32_t loopCounterCheckTime 	= 0;

uint32_t mainLoopCounter 		= 0;
uint32_t mainLoopsPerSecond	 	= 0;

uint32_t velocityLoopCounter 	= 0;
uint32_t velocityLoopsPerSecond	= 0;

uint32_t commLoopCounter 		= 0;
uint32_t commLoopsPerSecond		= 0;

void systemInfo_update(uint32_t actualSystick)
{
	if (abs(actualSystick-loopCounterCheckTime) >= 1000)
	{
		// reset main loop counter
		mainLoopsPerSecond = mainLoopCounter;
		mainLoopCounter = 0;

		// reset velocity loop counter
		velocityLoopsPerSecond = velocityLoopCounter;
		velocityLoopCounter = 0;

		// reset communication loop counter
		commLoopsPerSecond = commLoopCounter;
		commLoopCounter = 0;

		loopCounterCheckTime = actualSystick;
	}
}

void systemInfo_incMainLoopCounter()
{
	mainLoopCounter++;
}

uint32_t systemInfo_getMainLoopCounter()
{
	return mainLoopCounter;
}

uint32_t systemInfo_getMainLoopsPerSecond()
{
	return mainLoopsPerSecond;
}

void systemInfo_incVelocityLoopCounter()
{
	velocityLoopCounter++;
}

uint32_t systemInfo_getVelocityLoopsPerSecond()
{
	return velocityLoopsPerSecond;
}

uint32_t systemInfo_getVelocityLoopCounter()
{
	return velocityLoopCounter;
}

void systemInfo_incCommunicationLoopCounter()
{
	commLoopCounter++;
}

uint32_t systemInfo_getCommunicationsPerSecond()
{
	return commLoopsPerSecond;
}
