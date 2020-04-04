/*
 * Debug.h
 *
 *  Created on: 04.04.2020
 *      Author: ED
 */

#ifndef DEBUG_H
#define DEBUG_H

	#include "../Hal_Definitions.h"

	// getter/setter for debug variables

	int32_t debug_getTestVar0();
	int32_t debug_getTestVar1();
	int32_t debug_getTestVar2();
	int32_t debug_getTestVar3();
	int32_t debug_getTestVar4();
	int32_t debug_getTestVar5();
	int32_t debug_getTestVar6();
	int32_t debug_getTestVar7();
	int32_t debug_getTestVar8();
	int32_t debug_getTestVar9();

	void debug_setTestVar0(int32_t value);
	void debug_setTestVar1(int32_t value);
	void debug_setTestVar2(int32_t value);
	void debug_setTestVar3(int32_t value);
	void debug_setTestVar4(int32_t value);
	void debug_setTestVar5(int32_t value);
	void debug_setTestVar6(int32_t value);
	void debug_setTestVar7(int32_t value);
	void debug_setTestVar8(int32_t value);
	void debug_setTestVar9(int32_t value);

#endif /* DEBUG_H */
