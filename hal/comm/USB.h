/*
 * USB.h
 *
 *  Created on: 31.03.2020
 *      Author: ED
 */

#ifndef USB_H
#define USB_H

	#include "../Hal_Definitions.h"
	#include "hal/modules/SelectModule.h"

#if defined(USE_USB_INTERFACE)

	#include <string.h>

	void usb_init();
	uint8_t usb_getUSBCmd(uint8_t *cmd);
	void usb_sendData(uint8_t *buffer, uint32_t size);
	void usb_detach();

#endif

#endif
