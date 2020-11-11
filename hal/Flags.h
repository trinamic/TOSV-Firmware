/*
 * Flags.h
 *
 *  Created on: 31.03.2020
 *      Author: ED / OK
 */

#ifndef FLAGS_H
#define FLAGS_H

	#include "Hal_Definitions.h"

	// error- and status flags
	#define OVERCURRENT             0x00000001	// 0
	#define UNDERVOLTAGE            0x00000002	// 1
	#define OVERVOLTAGE             0x00000004	// 2
	#define OVERTEMPERATURE         0x00000008  // 3

	#define MOTORHALTED             0x00000010	// 4
	#define HALLERROR               0x00000020	// 5
	#define DRIVER_ERROR			0x00000040	// 6
	#define INIT_ERROR				0x00000080	// 7

	#define STOP_MODE			 	0x00000100	// 8
	#define VELOCITY_MODE			0x00000200	// 9
	#define PRESSURE_MODE           0x00000400	// 10
	#define TORQUE_MODE				0x00000800	// 11

	#define VOLUME_MODE				0x00001000	// 12
	#define FREERUNNING             0x00002000	// 13
	#define POSITION_END            0x00004000	// 14
	#define MODULE_INITIALIZED		0x00008000	// 15

	void flags_init(uint8_t motor);
	void flags_setStatusFlag(uint8_t motor, uint32_t flag);
	void flags_clearStatusFlag(uint8_t motor, uint32_t flag);
	void flags_setStatusFlagEnabled(uint8_t motor, uint32_t flag, bool enabled);
	uint8_t flags_isStatusFlagSet(uint8_t motor, uint32_t flag);
	void flags_resetErrorFlags(uint8_t motor);
	uint32_t flags_getAllStatusFlags(uint8_t motor);
	uint32_t flags_getErrorFlags(uint8_t motor);

#endif // FLAGS_H
