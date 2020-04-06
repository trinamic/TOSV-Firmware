/*
 * SPI.c
 *
 *  Created on: 02.04.2020
 *      Author: ed
 */
#include "SPI.h"

void spi_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPIInit;

#if defined(EEPROM_SPI1_ON_PB3_PB4_PB5)

	// enable clock for SPI1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	// enable clock for GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// use pins PB3, PB4, PB5 for SPI1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PA3, PA4, PA5 mit SPI1 verbinden
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);

	// initialize SPI1
	SPIInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPIInit.SPI_Mode = SPI_Mode_Master;
	SPIInit.SPI_DataSize = SPI_DataSize_8b;
	SPIInit.SPI_CPOL = SPI_CPOL_High;
	SPIInit.SPI_CPHA = SPI_CPHA_2Edge;
	SPIInit.SPI_NSS = SPI_NSS_Soft;
	SPIInit.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_32;
	SPIInit.SPI_FirstBit = SPI_FirstBit_MSB;
	SPIInit.SPI_CRCPolynomial = 0;
	SPI_Init(SPI1, &SPIInit);
	SPI_Cmd(SPI1, ENABLE);

#endif

#if defined(WEASEL_SPI3_ON_PC10_PC11_PC12)

	// enable clock for SPI3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// use pins PC10, PC11, PC12 for SPI3
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
	uint8_t data2 = SPI_I2S_ReceiveData(spiChannel);

	// set CS signal high after last transfer
	if(lastTransfer)
		tmcm_disableCsDragon(motor);

	return data2;
}

/* read and write EEPROM spi data */
uint8_t eeprom_spi_readWriteByte(uint8_t data, uint8_t lastTransfer)
{
#if defined(EEPROM_SPI1_ON_PB3_PB4_PB5)
	SPI_TypeDef *spiChannel = SPI1;
#endif

	// set CS signal low
	tmcm_enableCsMem();

	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(spiChannel, data);
	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_RXNE)==RESET);
	uint8_t data2 = SPI_I2S_ReceiveData(spiChannel);

	// set CS signal high after last transfer
	if(lastTransfer)
		tmcm_disableCsMem();

	return data2;
}
