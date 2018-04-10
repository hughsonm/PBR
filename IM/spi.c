/*
 * spi.c
 *
 * Created: 2014-03-08 23:40:46
 *  Author: Brandon Hill
 */ 

#include "spi.h"
#include <avr/io.h>

void spi_init(void)
{
	//SPI setup
	DDRB |= 0b00100111;
	// SCK, MOSI and SS as outputs MISO is not connected	
	PORTB |= 0b00100001; // Set slave selects (DATA1, DATA2) high
	//SPI control reg setup
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);//0b01010011;   
	SPSR |= 0;
	// Set as Master, divided clock by 128, enable spi 
	//no interrupts yet
}

void wait_spi(void)
{
	while (!(SPSR & (1<<SPIF)))//Block until SPI Int flag is set in status reg
	{
		
	}
}

u8 spi_byte(u8 data)
{
	
	SPDR = data;
	wait_spi();
	data = SPDR;
	return(data);
}
 
