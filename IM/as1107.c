/*
 * as1107.c
 *
 * Created: 2014-03-08 23:41:12
 *  Author: Brandon Hill
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include "as1107.h"
#include "spi.h"

u8	led_cmd(u8 ch_sel, u8 address, u8 data)
{
	ss_low(ch_sel);
	spi_byte(address);
	data = spi_byte(data);
	ss_hi(ch_sel);	
	return(data);
}

void led_init(u8 ch_sel)
{
	u8 ii;
	if(ch_sel == DATA1_CS)
	{
		led_cmd(DATA1_CS, DECMODE, DEC_0|DEC_1|DEC_2|DEC_3);
		led_cmd(DATA1_CS, INTENCON, DUTY_8);
		led_cmd(DATA1_CS, SCANLIM, ONLY_0_4);
		led_cmd(DATA1_CS, FEATURE, 0x00);
		led_cmd(DATA1_CS, DISPLAYTEST, NORM_OP);
		led_cmd(DATA1_CS, SHUTDOWN, NORM_UN);
		for (ii = DIG0; ii <= DIG7; ii++)
		{
			led_cmd(DATA1_CS, ii, 0x00);			
		}
	} else if(ch_sel == DATA2_CS)
	{
		led_cmd(DATA2_CS, DECMODE,  DEC_0|DEC_1|DEC_2|DEC_3);
		led_cmd(DATA2_CS, INTENCON, DUTY_8);
		led_cmd(DATA2_CS, SCANLIM, ONLY_0_4);
		led_cmd(DATA2_CS, FEATURE, 0x00);
		led_cmd(DATA2_CS, DISPLAYTEST, NORM_OP);
		led_cmd(DATA2_CS, SHUTDOWN, NORM_UN);
		for (ii = DIG0; ii <= DIG7; ii++)
		{
			led_cmd(DATA2_CS, ii, 0x00);			
		}
	}	
}

void u16_to_digits(u16 value, u8 *dest, u16 div, u8 dec_plc)
{
	u8 ii;
	u8 digit;
	for(ii = 0; ii < 4; ii++)
	{
		if(div)
		{
			digit = (u8)(value / div);
			digit = digit % 10;
		} else
		{
			digit = 0;
		}
		// Determine bits in order from MSb to LSb
		if(ii == dec_plc)
		{
			// Turn on the decimal point for this digit
			digit |= 0b10000000;
		}
		dest[ii] = digit;
		div /= 10;
	}
}

void sev_seg_write(u8 ch_sel, u16 value)
{
	u8 ii;
	u16 div = 1000;
	u8 digit;
	for (ii = 0; ii < 4; ii++)
	{
		digit = (u8)((value / div) % 10);
		led_cmd(ch_sel, DIG0+ii, digit);
		div /= 10;
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