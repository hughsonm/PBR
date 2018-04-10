#include "ads1220.h"
#include "pinmap.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"


uint8_t	SPI_SendReceiveData(SPI_TypeDef* SPIx, uint8_t data)
{
	while((SPIx->SR & SPI_I2S_FLAG_BSY) || (!(SPIx->SR & SPI_I2S_FLAG_TXE)));
	SPIx->DR = data;
	while((SPIx->SR & SPI_I2S_FLAG_BSY) || (!(SPIx->SR & SPI_I2S_FLAG_RXNE)));
	return(SPIx->DR);
}

void SPI_SendBuffer(SPI_TypeDef* SPIx, uint8_t *ptr, uint8_t length)
{

	for(uint8_t ii = 0;ii<length;ii++)
	{
		while((SPIx->SR & SPI_I2S_FLAG_BSY) || (!(SPIx->SR & SPI_I2S_FLAG_TXE)));
		SPIx->DR = ptr[ii];
	}
}

void ADS_InitHW(void)
{
	SPI_InitTypeDef 	SPI_InitStruct;
	GPIO_InitTypeDef	GPIO_InitStruct;

	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* configure pins used by SPI2
	 * PB15 = MOSI
	 * PB14 = MISO
	 * PB13 = SCK
	 * PB12 = CS1
	 * PC6  = CS2
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// connect SPI2 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);


	/* Configure the chip select pin
	   in this case we will use PB12 for ADS1, PC6 for ADS2 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	// Set up ADS2's CS
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	// Set up the DRDY pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	// Set both CS signals high
	GPIO_SetBits(PM_ADS_CS1);
	GPIO_SetBits(PM_ADS_CS2);

	// enable peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* configure SPI2 in Mode 0
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 1 --> data is sampled at the second edge
	 * In this case, the second edge is the falling edge.
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;      // data sampled at second (falling) edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // set the NSS management to software-controlled
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStruct);

	SPI_Cmd(SPI2, ENABLE); // enable SPI2
}

uint8_t SPI_ADS_Init(void)
{
	volatile uint8_t cmd;
	uint8_t	it_worked = 1;

	ADS_InitHW();
	// CS low
	GPIO_ResetBits(PM_ADS_CS1);
	// Tell the ADS to reset.
	cmd = SPI_SendReceiveData(SPI2,ADS_RST);
	for(uint32_t ii = 0;ii<20;ii++); // Delay so reset can sink in.
	// Tell it we're going to write 4 contiguous registers, starting at CFG0
	// This sequence taken from the datasheet for the ADS1220
	SPI_SendReceiveData(SPI2, 0x43);
	SPI_SendReceiveData(SPI2, 0x08);
	SPI_SendReceiveData(SPI2, 0x04);
	SPI_SendReceiveData(SPI2, 0x10);
	SPI_SendReceiveData(SPI2, 0x00);
	SPI_SendReceiveData(SPI2, 0x23);
	cmd = SPI_SendReceiveData(SPI2, 0x00);
	if(cmd != 0x08) it_worked = 0;
	cmd = SPI_SendReceiveData(SPI2, 0x00);
	if(cmd != 0x08) it_worked = 0;
	cmd = SPI_SendReceiveData(SPI2, 0x00);
	if(cmd != 0x08) it_worked = 0;
	cmd = SPI_SendReceiveData(SPI2, 0x00);
	if(cmd != 0x08) it_worked = 0;
	// Tell the ADS to start converting
	SPI_SendReceiveData(SPI2,ADS_STSYNC);
	// CS high
	GPIO_SetBits(PM_ADS_CS1);

	if(it_worked)
	{
		it_worked *= 3;
		it_worked += 2;
	}
	else
	{
		it_worked = 11;
		it_worked *= 7;
	}

	return(it_worked);
/*
	cmd = ADS_RREG(4,ADS_REG_CFG0);
	cmd = SPI_SendReceiveData(SPI2, cmd);
	cmd = SPI_SendReceiveData(SPI2, cmd);
	cmd = SPI_SendReceiveData(SPI2, cmd);
	cmd = SPI_SendReceiveData(SPI2, cmd);
	cmd = SPI_SendReceiveData(SPI2, cmd);
	cmd = 0x08;
	SPI_SendReceiveData(SPI2, cmd);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);*/
}

