/*
 * gear.c
 *
 * Created: 2014-03-08 23:41:01
 *  Author: Brandon Hill
 * Gear is displayed on a psa12-11srwa
 */ 

#include "gear.h"
#include "types.h"
#include "output.h"

extern u8 font[128][8];

void gear_off(void)
{
	DDRA = 0x00;
	DDRC = 0x00;
}

void gear_on(void)
{
	DDRA = 0xFF;
	DDRC = 0xFF;
}

void disp_gear(u8 gear_num)
{
	// Display a character on the dot matrix display
	u8 *row_ptr;
	
	if (gear_num == 0) {		
		row_ptr = &(font['N'][0]);
	}
	else {		
		row_ptr = &(font[gear_num+'0'][0]);
	}
	for (u8 jj = 0; jj < 8; jj++)
	{
		led_cmd(MAT_CS, DIG0 + jj, row_ptr[jj]);
	}
}








