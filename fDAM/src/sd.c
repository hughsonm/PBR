/*
 * sd.c
 *
 *  Created on: May 24, 2018
 *      Author: maxak
 */
#include "main.h"
#include "misc.h"
#include "pinmap.h"
#include "stm32f4xx_spi.h"
#include "sd.h"

void SD_Wait(void)
{
	/*
	volatile uint32_t	cnt;
	for(cnt = 0;cnt<3;cnt++)
	{

	}*/
}

int16_t SD_InitHW(void)
{
	SPI_InitTypeDef	SPI_InitStruct;
	GPIO_InitTypeDef	GPIO_InitStruct;
	// Set SPI1 up
	// Chip detect is on PA3
	// nslaveselect is on PA4.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);


	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// connect SPI2 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);


	/* Configure the chip select pin
	   in this case we will use PA4 for cs*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_SetBits(PM_ADS_CS1);

	// PA3 is the chip detect pin.
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// enable peripheral clock
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at second (falling) edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // set the NSS management to software-controlled
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE); // enable SPI2
	return(0x0000);
}

int16_t	SD_InitCard(void)
{
	uint8_t	command_frame[6] = {0,0,0,0,0,0};
	uint8_t	ii,xx;

	int16_t	status = 1;

	if(GPIO_ReadInputDataBit(PM_SD_CD))return(1);

	xx=0xFF;
	GPIO_SetBits(PM_SD_CS);
	// Send >= 74 clock pulses.
	for(ii=0;ii<10;ii++)
	{
		SPI_SendBuffer(SPI1,&xx,1);
	}

	// GO_IDLE_STATE
	command_frame[0] = (0b01000000) | (0);
	command_frame[5] = (0x95);	// Valid CRC7 with stop bit
	GPIO_ResetBits(PM_SD_CS);
	SD_Wait();SPI_SendBuffer(SPI1,command_frame,6);

	// Wait for R1 response with idle bit set
	for(ii=0;ii<8;ii++)
	{
		// expect an R1 response.
		SD_Wait();
		xx = SPI_SendReceiveData(SPI1,0xFF);
		if(xx == 0x01)break;
	}

	if(xx != 0x01){GPIO_SetBits(PM_SD_CS);return(2);}

	GPIO_SetBits(PM_SD_CS);
	GPIO_ResetBits(PM_SD_CS);

	//Send CMD8 with a correct CRC (0x87)
	command_frame[0] = 0x48;
	command_frame[3] = 0x01;
	command_frame[4] = 0xAA;
	command_frame[5] = 0x87;
	SD_Wait();SPI_SendBuffer(SPI1,command_frame,6);
	for(ii=0;ii<8;ii++)
	{
		SD_Wait();
		xx = SPI_SendReceiveData(SPI1,0xFF);
		if(xx!=0xFF)break;
	}
	if(xx==0xFF)return(3);

	uint8_t	rcv_buf[5] = {0,0,0,0,0};
	for(ii=0;ii<4;ii++)
	{
		SD_Wait();
		rcv_buf[ii] = SPI_SendReceiveData(SPI1,0xFF);
	}
	uint8_t	match = (rcv_buf[2] == 0x01) && (rcv_buf[3] == 0xAA);
	match = match || ((rcv_buf[1] == 0x01) && (rcv_buf[2] == 0xAA));
	if(!match){GPIO_SetBits(PM_SD_CS);return(4);}
	// Now send ACMD41
	xx = 0x01;
	while(xx==0x01)
	{
		// Send cmd55 (first half of ACMD41
		command_frame[0] = 0x40 | (55);
		command_frame[1] = 0;
		command_frame[2] = 0;
		command_frame[3] = 0;
		command_frame[4] = 0;
		command_frame[5] = 0;
		SD_Wait();SPI_SendBuffer(SPI1,command_frame,6);
		for(ii=0;ii<8;ii++)
		{
			SD_Wait();
			xx = SPI_SendReceiveData(SPI1,0xFF);
			if(xx!=0xFF)break;
		}
		if(xx!=0x01){GPIO_SetBits(PM_SD_CS);return(5);}
		// Send 41 (second half of ACMD41
		command_frame[0] = 0x40 | 41;
		command_frame[1] = 0x40;
		SD_Wait();SPI_SendBuffer(SPI1,command_frame,6);
		for(ii=0;ii<8;ii++)
		{
			SD_Wait();
			xx = SPI_SendReceiveData(SPI1,0xFF);
			if(xx!=0xFF)break;
		}	// If we got 0x01, continue trying ACMD41
	}
	if(xx!=0x00){GPIO_SetBits(PM_SD_CS);return(6);}
	// CMD 58 - read the OCR
	command_frame[0] = 0x40 | 58;
	command_frame[1] = 0x00;
	command_frame[2] = 0x00;
	command_frame[3] = 0x00;
	command_frame[4] = 0x00;
	SD_Wait();SPI_SendBuffer(SPI1,command_frame,6);
	for(ii=0;ii<8;ii++)
	{
		SD_Wait();
		xx = SPI_SendReceiveData(SPI1,0xFF);
		if(xx!=0xFF)break;
	}
	if(xx==0xFF){GPIO_SetBits(PM_SD_CS);return(7);}

	for(ii=0;ii<4;ii++)
	{
		SD_Wait();
		rcv_buf[ii] = SPI_SendReceiveData(SPI1,0xFF);
	}
	uint8_t	ccs = (rcv_buf[0] & 0b01000000);
	if(ccs){GPIO_SetBits(PM_SD_CS);return(0);}
	command_frame[0] = 0x40 | 16;
	command_frame[1] = 0x00;
	command_frame[2] = 0x00;
	command_frame[3] = 0x02;
	command_frame[4] = 0x00;
	SD_Wait();SPI_SendBuffer(SPI1,command_frame,6);
	for(ii=0;ii<8;ii++)
	{
		SD_Wait();
		xx = SPI_SendReceiveData(SPI1,0xFF);
		if(xx!=0xFF)break;
	}
	// At this point, we should crank up the SPI clock.
	GPIO_SetBits(PM_SD_CS);
	return(0);
}

static	uint8_t	SD_GetResponse(uint8_t desired, uint8_t mask, uint16_t n_tries)
{
	uint16_t ii;
	uint8_t rcv=0;
	for(ii = 0;ii<n_tries;ii++)
	{
		rcv = SPI_SendReceiveData(SPI1,0xFF);
		if((rcv&mask)==(desired&mask))break;
	}
	return(rcv);
}

int16_t SD_DiskWrite(char * buff,uint16_t sector, uint16_t count)
{
	int16_t status = -1;
	uint8_t command_frame[6] = {0,0,0,0,0,0};
	uint8_t	rcv;
	uint16_t	buff_ptr, tx_cnt;
	GPIO_SetBits(PM_SD_CS);
	GPIO_ResetBits(PM_SD_CS);

	// send CMD25
	command_frame[0] = 0x40 | 25;
	command_frame[3] = (uint8_t)(sector>>8);
	command_frame[4] = (uint8_t)(sector);
	SPI_SendBuffer(SPI1,command_frame,6);

	rcv = SD_GetResponse(0x00,0xFF,8);
	if(rcv != 0x00){GPIO_SetBits(PM_SD_CS);return(status);}

	/*
	 * It appears that a period of >= 1 byte must follow
	 * the receipt of the response. See if we can live
	 * without it for now.
	 */
	buff_ptr = 0;
	while(count)
	{
		// Wait for card to not be busy
		while(!GPIO_ReadInputDataBit(PM_SD_MISO));
		// Send the data token
		SPI_SendReceiveData(SPI1,0b11111100);
		for(tx_cnt = 0;tx_cnt<512;tx_cnt++)
		{
			SPI_SendReceiveData(SPI1,buff[buff_ptr++]);
		}
		// Two CRC bytes
		SPI_SendReceiveData(SPI1,0xFF);
		SPI_SendReceiveData(SPI1,0xFF);
		// Get the data response
		rcv = SD_GetResponse(0b00000001,0b00010001,8);
		if((rcv&0b00011111) !=0b00000101){GPIO_SetBits(PM_SD_CS);return(status);}
		// Send the 8 post-clocks
		SPI_SendReceiveData(SPI1,0xFF);
		count--;
	}
	//Wait for card to not be busy then send stop tran token
	while(!GPIO_ReadInputDataBit(PM_SD_MISO));
	SPI_SendReceiveData(SPI1,0b11111101);
	GPIO_SetBits(PM_SD_CS);
	return(0);
}

int16_t	SD_DiskRead(char * buff, uint16_t sector, uint16_t count)
{
	/*
	 * CMD18 is for reading multiple consecutive sectors
	 * CMD12 terminates the transaction.
	 * After CMD12, the card sends a stuff byte that
	 * should be discarded before receiving the response
	 * to CMD12.
	 */
	uint8_t	command_frame[6] = {0,0,0,0,0,0};
	uint8_t	rcv,ii;
	uint16_t	byte_cnt;
	uint32_t	buff_ptr;
	// If the SD card is busy, wait for it.
	while(!GPIO_ReadInputDataBit(PM_SD_MISO));
	// continue.
	command_frame[0] = 0x40 | 18;
	command_frame[3] = (uint8_t)(sector>>8);
	command_frame[4] = (uint8_t)sector;
	GPIO_ResetBits(PM_SD_CS);
	SPI_SendBuffer(SPI1,command_frame,6);
	for(ii=0;ii<8;ii++)
	{
		// expect an R1 response.
		SD_Wait();
		rcv = SPI_SendReceiveData(SPI1,0xFF);
		if(rcv != 0xFF)break;
	}
	if(rcv != 0x00){GPIO_SetBits(PM_SD_CS); return(-1);}
	for(ii=0;ii<100;ii++)
	{
		SD_Wait();
		rcv = SPI_SendReceiveData(SPI1,0xFF);
		if(rcv != 0xFF)break;
	}
	if(rcv != 0b11111110){GPIO_SetBits(PM_SD_CS); return(-1);}
	buff_ptr = 0;
	while(count)
	{
		// Begin reading data
		// Poll the spi bus until a data token appears.
		for(byte_cnt = 0;byte_cnt < 512;byte_cnt++)
		{
			buff[buff_ptr++] = SPI_SendReceiveData(SPI1,0xFF);
		}
		// Then 2 bytes of CRC.
		SPI_SendReceiveData(SPI1,0xFF);
		SPI_SendReceiveData(SPI1,0xFF);
		count--;

	}
	// Now we have read all count sectors. Send a CMD12 to terminate.
	command_frame[0] = 0x40 | 12;
	command_frame[3] = 0;
	command_frame[4] = 0;
	SPI_SendBuffer(SPI1,command_frame,6);
	// Bypass that stuff bit.
	SPI_SendReceiveData(SPI1,0xFF);

	for(ii = 0;ii<8;ii++)
	{
		rcv = SPI_SendReceiveData(SPI1,0xFF);
		if(rcv==0x00)break;
	}
	if(rcv == 0xFF){GPIO_SetBits(PM_SD_CS);return(-1);}
	GPIO_SetBits(PM_SD_CS);
	return(0);
}
