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
//UCHAR I2C_Master_BufferRead(I2C_TypeDef* I2Cx, UCHAR* pBuffer,  UINT NumByteToRead, UCHAR SlaveAddress);
//UCHAR I2C_Master_BufferWrite(I2C_TypeDef* I2Cx, UCHAR* pBuffer,  UINT NumByteToWrite, UCHAR SlaveAddress);

#endif
