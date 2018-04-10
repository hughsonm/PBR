/*
 * as1107.c
 *
 * Created: 2014-03-08 23:41:12
 *  Author: Brandon Hill
 */ 

#include "as1107.h"
#include "spi.h"
#include "output.h"

void blockingDataSPIWrite(char reg, char data)
{
	cli();
	dataSSOn(); //set data SS low
	SPDR = reg; //Send register byte
	wait_spi(); //Wait for finish
	SPDR = data; //Send register data
	wait_spi(); //Wait for finish
	dataSSOff(); //set data SS high to latch
	sei();
}

void blockingrgbSPIWrite(char reg, char data)
{
	cli();
	rgbSSOn(); //set data SS low
	SPDR = reg; //Send register byte
	wait_spi(); //Wait for finish
	SPDR = data; //Send register data
	wait_spi(); //Wait for finish
	rgbSSOff(); //set data SS high to latch
	sei();
}

u8	led_cmd(u8 ch_sel, u8 address, u8 data)
{
	// LED_CS is PB0, MAT_CS is PB4
	cli();
	ss_low(ch_sel);
	spi_byte(address);
	data = spi_byte(data);
	ss_hi(ch_sel);	
	sei();
	return(data);
}

void led_init(u8 ch_sel)
{
	u8 ii;
	if(ch_sel == LED_CS)
	{
		led_cmd(ch_sel, DECMODE, NO_DEC);
		led_cmd(ch_sel, INTENCON, DUTY_MAX);
		led_cmd(ch_sel, SCANLIM, ONLY_0_6); // how many digits to display
		led_cmd(ch_sel, FEATURE, 0x00);
		led_cmd(ch_sel, DISPLAYTEST, NORM_OP); // don't do display test, normal operation
		led_cmd(ch_sel, SHUTDOWN, NORM_UN); // blanks all displays when written to
		for (ii = DIG0; ii <= DIG7; ii++)
		{
			// Light all the displays
			led_cmd(ch_sel, ii, 0xFF);
		}
	} else if (ch_sel == MAT_CS)
	{
		led_cmd(ch_sel, DECMODE, NO_DEC);
		led_cmd(ch_sel, INTENCON, DUTY_MAX);
		led_cmd(ch_sel, SCANLIM, ONLY_0_7); // how many digits to display
		led_cmd(ch_sel, FEATURE, 0x00);
		led_cmd(ch_sel, DISPLAYTEST, NORM_OP); // don't do display test, normal operation
		led_cmd(ch_sel, SHUTDOWN, NORM_UN); // blanks all displays when written to
		for (ii = DIG0; ii <= DIG7; ii++)
		{
			// Light all the displays
			led_cmd(ch_sel, ii, 0xFF);
		}
	}
	
}

void ss_low(u8 pin)
{
	PORTB &= ~(1<<pin);
}

void ss_hi(u8 pin)
{
	PORTB |= (1<<pin);
}

void inline dataSSOn()//set as1107 slave select low to transmit to gear
{
	PORTB &= 0b11011111; 
}

void inline dataSSOff()//set as1107 slave select high to latch
{
	PORTB |= 0b00100000; 
}

void inline rgbSSOn()
{
	PORTB &= 0b10111111; 
}

void inline rgbSSOff()
{
	PORTB |= 0b01000000; 
}

