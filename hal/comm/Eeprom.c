/*
 * Eeprom.c
 *
 *  Acces to external EEPROM (AT25128)
 *
 *  Created on: 06.04.2020
 *      Author: OK / ED
 */

#include "Eeprom.h"
#include "SPI.h"

extern uint32_t TMCM_MOTOR_CONFIG_SIZE;

/* initialize the eeprom and read the default parameters from eeprom */
void eeprom_initConfig()
{
	// initialize the eeprom
	if (eeprom_readConfigByte(TMCM_ADDR_EEPROM_MAGIC) != TMCM_EEPROM_MAGIC)
	{
		eeprom_writeConfigByte(TMCM_ADDR_EEPROM_MAGIC, 0);

		// overwrite module configuration
		eeprom_writeConfigBlock(TMCM_ADDR_MODULE_CONFIG, (uint8_t *)&moduleConfig, sizeof(TModuleConfig));

		// overwrite motor configuration
		for (int i = 0; i < NUMBER_OF_MOTORS; i++)
		{
			eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG + i*TMCM_MOTOR_CONFIG_SIZE, (uint8_t *)&motorConfig[i], sizeof(TMotorConfig));
		}

		// update magic byte
		eeprom_writeConfigByte(TMCM_ADDR_EEPROM_MAGIC, TMCM_EEPROM_MAGIC);
	}

	// read the default module configuration from EEPROM
	eeprom_readConfigBlock(TMCM_ADDR_MODULE_CONFIG, (uint8_t *)&moduleConfig, sizeof(TModuleConfig));

	// read the default motor configurations from EEPROM
	for (int i = 0; i < NUMBER_OF_MOTORS; i++)
	{
		eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG + i*TMCM_MOTOR_CONFIG_SIZE, (uint8_t *)&motorConfig[i], sizeof(TMotorConfig));
	}
}

/* read a byte from the configuration eeprom (address: 0..2047) */
uint8_t eeprom_readConfigByte(uint32_t address)
{
	eeprom_spi_readWriteByte(0x03, false);  // command "Read"
	eeprom_spi_readWriteByte(address >> 8, false);
	eeprom_spi_readWriteByte(address & 0xff, false);
	return eeprom_spi_readWriteByte(0, true);
}

/* write a byte into the configuration eeprom (address: 0..2047) */
void eeprom_writeConfigByte(uint32_t address, uint8_t value)
{
	// enable writing
	eeprom_spi_readWriteByte(0x06, true);  // command "Write Enable"
	do
	{
		eeprom_spi_readWriteByte(0x05, false);  // command "Get Status"
	} while((eeprom_spi_readWriteByte(0x00, true) & 0x02) == 0x00);  // wait until "Write Enable"-Bit is set

	// write
	eeprom_spi_readWriteByte(0x02, false); // command "Write"
	eeprom_spi_readWriteByte(address >> 8, false);
	eeprom_spi_readWriteByte(address & 0xff, false);
	eeprom_spi_readWriteByte(value, true);

	// wait until write is finished
	do
	{
		eeprom_spi_readWriteByte(0x05, false);  // command "Get Status"
	} while(eeprom_spi_readWriteByte(0x00, true) & 0x01);
}

/* write a block into the configuration eeprom (using eeprom 25128),
 * (address: (0..2047) (block: start address of the block) */
void eeprom_writeConfigBlock(uint32_t address, uint8_t *block, uint32_t size)
{
	// enable reading
	eeprom_spi_readWriteByte(0x06, true);  // command "Write Enable"
	do
	{
		eeprom_spi_readWriteByte(0x05, false);  // command "Get Status"
	} while((eeprom_spi_readWriteByte(0x00, true) & 0x02) == 0x00);  // wait until "Write Enable"-Bit is set

	// write (Startadresse)
	eeprom_spi_readWriteByte(0x02, false); //Befehl "Write"
	eeprom_spi_readWriteByte(address >> 8, false);
	eeprom_spi_readWriteByte(address & 0xff, false);

	// write data
	uint32_t i;
	for(i=0; i<size; i++)
	{
		//Adresse mitzählen und bei Überlauf der untersten sechs Bits das EEPROM deselektieren
		//und neuen Write-Befehl senden (bzw. beim letzten Datenbyte einfach nur EEPROM
		//deselektieren).
		//Dies ist erforderlich, da beim Beschreiben im 25128 nur die untersten sechs Bits der
		//Adresse hochgezählt werden (anders als beim Lesen).
		address++;
		eeprom_spi_readWriteByte(*(block+i), (address & 0x0000003f)==0 || i==size-1);
		if((address & 0x0000003f)==0 && i<size-1)  //Adressbits übergelaufen, aber noch Bytes zu schreiben?
		{
			// wait until write finished
			do
			{
				eeprom_spi_readWriteByte(0x05, false);  // command "Get Status"
			} while(eeprom_spi_readWriteByte(0x00, true) & 0x01);

			// new "Write Enable" command
			eeprom_spi_readWriteByte(0x06, true);  // command "Write Enable"
			do
			{
				eeprom_spi_readWriteByte(0x05, false);  // command "Get Status"
			} while((eeprom_spi_readWriteByte(0x00, true) & 0x02)==0x00);  // wait until "Write Enable" bit is set

			// new "Write" command (with next address)
			eeprom_spi_readWriteByte(0x02, false); // command "Write"
			eeprom_spi_readWriteByte(address >> 8, false);
			eeprom_spi_readWriteByte(address & 0xff, false);
		}
	}

	// wait until write finished
	do
	{
		eeprom_spi_readWriteByte(0x05, false);  // command "Get Status"
	} while(eeprom_spi_readWriteByte(0x00, true) & 0x01);
}

/* read a block from configuration eeprom
 * (address: (0..2047) (block: start address) (size: max. 64)) */
void eeprom_readConfigBlock(uint32_t address, uint8_t *block, uint32_t size)
{
	eeprom_spi_readWriteByte(0x03, false);  // command "Read"
	eeprom_spi_readWriteByte(address >> 8, false);
	eeprom_spi_readWriteByte(address & 0xff, false);

	uint32_t i;
	for(i=0; i<size; i++)
	{
		*(block+i) = eeprom_spi_readWriteByte(0, i==size-1);  // unselect EEPROM with last written byte
	}
}
