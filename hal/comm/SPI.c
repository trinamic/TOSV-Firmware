/*
 * SPI.c
 *
 *  Created on: 02.04.2020
 *      Author: ed
 */
#include "SPI.h"

void spi_init()
{
#if defined(WEASEL_SPI3_ON_PC10_PC11_PC12)

	// enable clock for SPI3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// use pins PC10, PC11, PC12 for SPI3
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// PC10, PC11, PC12 mit SPI3 verbinden
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

	// initialize SPI3
	SPI_InitTypeDef SPIInit;
	SPIInit.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
	SPIInit.SPI_Mode              = SPI_Mode_Master;
	SPIInit.SPI_DataSize          = SPI_DataSize_8b;
	SPIInit.SPI_CPOL              = SPI_CPOL_High;
	SPIInit.SPI_CPHA              = SPI_CPHA_2Edge;
	SPIInit.SPI_NSS               = SPI_NSS_Soft;
	SPIInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPIInit.SPI_FirstBit          = SPI_FirstBit_MSB;
	SPIInit.SPI_CRCPolynomial     = 0;
	SPI_Init(SPI3, &SPIInit);
	SPI_Cmd(SPI3, ENABLE);

#endif
}

uint8_t weasel_spi_readWriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
#if defined(WEASEL_SPI3_ON_PC10_PC11_PC12)
	SPI_TypeDef *spiChannel = SPI3;
#endif

	// set CS signal low
	tmcm_enableCsWeasel(motor);

	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(spiChannel, data);
	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_RXNE)==RESET);
	uint8_t data2 = SPI_I2S_ReceiveData(spiChannel);

	// set CS signal high after last transfer
	if(lastTransfer)
		tmcm_disableCsWeasel(motor);

	return data2;
}

uint8_t dragon_spi_readWriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
#if defined(DRAGON_SPI3_ON_PC10_PC11_PC12)
	SPI_TypeDef *spiChannel = SPI3;
#endif

	// set CS signal low
	tmcm_enableCsDragon(motor);

	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(spiChannel, data);
	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_RXNE)==RESET);
	u8 data2 = SPI_I2S_ReceiveData(spiChannel);

	// set CS signal high after last transfer
	if(lastTransfer)
		tmcm_disableCsDragon(motor);

	return data2;
}

