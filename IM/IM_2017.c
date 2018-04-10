/*
 * IM_2016.c
 *
 * Created: 2016-03-13 6:32:04 PM
 *  Author: Max
 */ 

#include "IM_2017.h"
#include "spi.h"
#include "as1107.h"
#include "types.h"
#include "output.h"
#include "buttons.h"
#include "canbus.h"
#include "timer.h"
#include <avr/eeprom.h>

extern u8 CANTXData[1];
extern carState Car;
extern u8 SendCANData;
extern u32 UpdateTick_ms;

int main(void)
{
	spi_init();
    output_setup();
	button_init();
	can_setup();
	timer_init();
	DDRA |= 0b00000111;
	PORTA |= 0b00000111;
	while(1)
	{				
		if(UpdateTick_ms > UPDATE_PERIOD_MS)
		{			
			update_display();
			UpdateTick_ms = 0;				
		}
		
		if(SendCANData)
		{
			writeTXMOB(IM_CTRL_MOB, IM_CTRL_ID, CANTXData, 1);
			SendCANData = 0;
		}
	}		
	return(0);
}