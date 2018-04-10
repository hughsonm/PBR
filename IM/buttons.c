/*
 * cycleButtons.c
 *
 * Created: 2014-03-09 00:16:11
 *  Author: Brandon Hill
 */ 
#include "IM_2017.h"
#include "buttons.h"
#include "canbus.h"
#include "output.h"
#include "timer.h"
extern indModuleState IndModule;

extern u8 CANTXData[1];
extern u8 SendCANData;
extern u32 RunTime;

void button_init(void)
{			
	/*Trac up, Trac down, upper data, lower data buttons*/
	// Set pins 4-7 as inputs
	DDRE &= ~((1<<DDE7)|(1<<DDE6)|(1<<DDE5)|(1<<DDE4));	
	// Set up negative edge interrupts for pins 4-7
	EICRB = (NEG_EDG_INT<<6)|(NEG_EDG_INT<<4)|(NEG_EDG_INT<<2)|(NEG_EDG_INT<<0);
	// Turn pullups on for pins 4-7
	PORTE |= ((1<<PE7)|(1<<PE6)|(1<<PE5)|(1<<PE4));
	
	EIMSK |= ((1<<INT7)|(1<<INT6)|(1<<INT5)|(1<<INT4));		
}

u8 debounce_pin(volatile u8 *port, u8 pin)
{
	u8 old;
	u8 new;
	u32 count = 0;
	u8 mask = 1<<pin;
	old = (*port) & mask;
	while(ENOUGH_DB > count)
	{
		new = (*port) & mask;
		if (new == old)
		{
			count++;
		} else
		{
			count = 0;
			old = new;
		}
	}
	return(old);
}

ISR(INT7_vect)
{
	// Lower data cycle button
	// Interrupt flag is automatically cleared by entering ISR
	u8 pin_state = debounce_pin(&PINE, PE7);
	if(!pin_state)
	{		
		if(3 == IndModule.data_id[0])
		{
			IndModule.data_id[0] = 0;
		} else
		{
			IndModule.data_id[0]++;
		}
		
	}
}

ISR(INT6_vect)
{
	// Upper data cycle button
	u8 pin_state = debounce_pin(&PINE, PE6);
	if(!pin_state)
	{
		if(3 == IndModule.data_id[1])
		{
			IndModule.data_id[1] = 0;
		}else
		{
			IndModule.data_id[1]++;
		}
		if(!(PINE & (1<<PE7)))
		{
			output_setup();
		}
	}
}

ISR(INT5_vect)
{
	// TRAC DOWN BUTTON
	u8 pin_state,dash_brightness;
	// Negative edge on PE0 detected, so de-bounce.
	pin_state = debounce_pin(&PINE, PE5);
	if(!pin_state)
	{
		// Pin is truly low
		if(PINE & (1<<PINE6))
		{
			// UData button isn't pressed.
			if(CANTXData[MAP_SIG] < (N_ENG_MAPS-1)) CANTXData[MAP_SIG]++;			
			IndModule.show_map = 1;
			IndModule.show_start = RunTime;
			SendCANData = 1;
		}else
		{
			// Upper data button is pressed, so
			// send an increase brightness packet
			dash_brightness = 0xFF;
			writeTXMOB(BRIGHTNESS_MOB, BRIGHTNESS_ID, &dash_brightness, BRIGHTNESS_DLC);
			increase_brightness(1);
		}
		
	}	
}

ISR(INT4_vect)
{
	// TRAC UP BUTTON
	u8 pin_state,dash_brightness;
	// Negative edge on PE1 detected, so de-bounce.
	pin_state = debounce_pin(&PINE, PE4);
	if(!pin_state)
	{
		// Pin is truly low
		if(PINE & (1<<PINE6))
		{
			// UData button isn't pressed.
			if(CANTXData[MAP_SIG]) CANTXData[MAP_SIG]--;
			IndModule.show_map = 1;
			IndModule.show_start = RunTime;			
			SendCANData = 1;
		}else
		{
			// Upper data button is pressed, so
			// send a decrease brightness packet
			dash_brightness = 0x00;
			writeTXMOB(BRIGHTNESS_MOB, BRIGHTNESS_ID, &dash_brightness, BRIGHTNESS_DLC);
			decrease_brightness(1);
		}
		
	}
}