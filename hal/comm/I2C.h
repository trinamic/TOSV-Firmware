/****************************************************
  Projekt:

  Modul:   I2C.h
           Funktionen für I2C-Kommunikation

  Datum:   31.8.2011 OK
*****************************************************/

#ifndef __I2C_H
#define __I2C_H

	#include "../Hal_Definitions.h"

void InitIIC(void);
uint8_t I2C_Master_BufferRead(I2C_TypeDef* I2Cx, u8* pBuffer,  u32 NumByteToRead, u8 SlaveAddress);
uint8_t I2C_Master_BufferWrite(I2C_TypeDef* I2Cx, u8* pBuffer,  u32 NumByteToWrite, u8 SlaveAddress );

#endif
