#include "ads1220.h"
#include "pinmap.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"

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


	// From the ADS1120 datasheet:
	// " Only SPI mode 1 (CPOL = 0, CPHA = 1) is supported"

	/* configure SPI2 in Mode 1
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

	// IMPORTANT NOTICE ABOUT DEBUGGING THIS SPI INTERFACE!
	// The ADS chip has an SPI timeout feature. If there's no communication for
	// roughly 14000 clock cycles, the chip will stop communicating.
	// fmod = 256 kHz from the internal oscillator.
	// tmod = 3.9 microseconds
	// timeout kicks in after 13955*tmod or 54.5 milliseconds.
	// SPI clock is 16MHz, with a prescaler of 32, so 500 MHz.
	// 54.5 milliseconds gives us room for 27255859 serial clock
	// ticks before the ADS resets.
	// That's plenty of time for the processor, but not much time if we stop
	// it and try to step through this section with the debugger.

	GPIO_ResetBits(PM_ADS_CS1);
	// Tell the ADS to reset.
	cmd = SPI_SendReceiveData(SPI2,ADS_RST);
	GPIO_SetBits(PM_ADS_CS1);
	// From the datasheet:
	// Wait at least (50 µs + 32 · t (CLK) )
	// tclk = 16/fmod = 16/256k = 62.5 us.
	// Wait at least (50 us + 32*62.5 us) = 2.05 ms. MILLISECONDS!!!
	// Suppose a for-loop delay operates at 16MHz, to be safe.
	// 2.05ms/(1/16000000) = 32800 clock ticks of delay.
	for(uint32_t ii = 0;ii<32800;ii++); // Delay so reset can sink in.
	// Tell it we're going to write 4 contiguous registers, starting at CFG0
	// This sequence taken from the datasheet for the ADS1220
	GPIO_ResetBits(PM_ADS_CS1);
	SPI_SendReceiveData(SPI2, ADS_WREG(ADS_REG_CFG0,4));//SPI_SendReceiveData(SPI2, 0x43);
	SPI_SendReceiveData(SPI2, 4<<ADS_CFG0_GAIN);
	SPI_SendReceiveData(SPI2, 1<<ADS_CFG1_CM);
	SPI_SendReceiveData(SPI2, 0b10<<ADS_CFG2_5060);
	SPI_SendReceiveData(SPI2, 0x00);

	SPI_SendReceiveData(SPI2, ADS_RREG(ADS_REG_CFG0,4));
	cmd = SPI_SendReceiveData(SPI2, 0x00);
	if(cmd != (4<<ADS_CFG0_GAIN)) it_worked = 0;
	cmd = SPI_SendReceiveData(SPI2, 0x00);
	if(cmd != (1<<ADS_CFG1_CM)) it_worked = 0;
	cmd = SPI_SendReceiveData(SPI2, 0x00);
	if(cmd != (0b10<<ADS_CFG2_5060)) it_worked = 0;
	cmd = SPI_SendReceiveData(SPI2, 0x00);
	if(cmd != 0x00) it_worked = 0;
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

