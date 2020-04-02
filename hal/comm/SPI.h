/*
 * SPI.h
 *
 *  Created on: 02.04.2020
 *      Author: ED
 */

#ifndef SPI_H
#define SPI_H

	#include "../Hal_Definitions.h"

	void spi_init();
	uint8_t weasel_spi_readWriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer);
	uint8_t dragon_spi_readWriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer);

#endif /* WEASEL_H */
