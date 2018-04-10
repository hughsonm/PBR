/*
 * cycleButtons.c
 *
 * Created: 2014-03-09 00:16:11
 *  Author: Brandon Hill
 */ 

#include "buttons.h"
#include "canbus.h"
#include "as1107.h"
#include "spi.h"
#include "output.h"

extern u8 CANTXData[DASH2ECU_DLC];
extern u8 LaunchRXData[ECU2DASH_DLC];
extern u8 SendCANData;
extern dashState	Dashboard;

void button_init(void)
{
	u8 bit;	
	/*Fan and Launch Buttons*/
	PORTD |= ((1<<PD3)|(1<<PD2));
	DDRD &= ~((1<<DDD3)|(1<<DDD2)); // D3 and D2 as inputs
	PORTD |= ((1<<PD3)|(1<<PD2)); // Enable internal pull ups for D3 and D2
	EIMSK |= (1<<INT2)|(1<<INT3); // Enable INT2 (FAN) and INT3 (LAUNCH)
	// Setup EICRA which holds sense control bits for interrupts 2-3.
	EICRA = (NEG_EDG_INT<<6)|(NEG_EDG_INT<<4);
	
	/*LEDs in the Launch and Start Buttons*/
	
	DDRA |= (1<<PA3)|(1<<PA4);
	PORTA |= (1<<PA3)|(1<<PA4);
	
	/*SEL Inputs*/
	PORTE |= ((1<<PE7)|(1<<PE6)|(1<<PE5)|(1<<PE4)); // Enable internal pull ups for E7 - E4
	DDRE &= ~((1<<DDE7)|(1<<DDE6)|(1<<DDE5)|(1<<DDE4)); //E7-E4 as inputs
	PORTE |= ((1<<PE7)|(1<<PE6)|(1<<PE5)|(1<<PE4)); // Enable internal pull ups for E7 - E4
	EIMSK |= (1<<INT7)|(1<<INT6)|(1<<INT5)|(1<<INT4); // Enable INT4 - INT7 (SEL)
	// Setup EICRB which holds sense control bits for interrupts 4-7.
	EICRB = (NEG_EDG_INT<<6)|(NEG_EDG_INT<<4)|(NEG_EDG_INT<<2)|(NEG_EDG_INT<<0);
	
	/* Check what position the rotary switch is in*/
	bit = get_rotary_pos();
	//CANTXData[MAPS_CONTROL] = bit;
	//SendCANData = bit;
	Dashboard.disp_mode = bit;
	
}

u8 get_rotary_pos(void)
{
	u8 rotary,bit;
	rotary = ~(PINE>>4);
	for(bit = 0;bit < 4; bit++)
	{
		if(rotary & (1<<bit))
		{			
			break;
		}
	}
	bit = (4==bit)?0:bit;
	return(bit);
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

ISR(INT3_vect)
{
	// "LAUNCH" button
	// Interrupt flag is automatically cleared by entering ISR
	u8 pin_state = debounce_pin(&PIND, PIND3);
	if(!pin_state)
	{
		// Toggle "Launch" state
		CANTXData[LAUNCH_CONTROL_REQ] = !LaunchRXData[LAUNCH_CONTROL_ACT];
		SendCANData = 1;
	}
}


ISR(INT4_vect)
{
	u8 pin_state = debounce_pin(&PINE, PINE4);
	if(!pin_state)
	{
		//CANTXData[MAPS_CONTROL] = MAPS0;
		//SendCANData = 1;
		Dashboard.disp_mode = DMODE_GEAR;
	}	
}

ISR(INT5_vect)
{
	u8 pin_state = debounce_pin(&PINE, PINE5);
	if (!pin_state)
	{
		//CANTXData[MAPS_CONTROL] = MAPS1;
		//SendCANData = 1;
		Dashboard.disp_mode = DMODE_STR;
	}
}

ISR(INT6_vect)
{
	u8 pin_state = debounce_pin(&PINE, PINE6);
	if (!pin_state)
	{
		//CANTXData[MAPS_CONTROL] = MAPS2;
		//SendCANData = 1;
		Dashboard.disp_mode = DMODE_ACC;
	}
}

ISR(INT7_vect)
{
	u8 pin_state = debounce_pin(&PINE, PINE7);
	if (!pin_state)
	{
		//CANTXData[MAPS_CONTROL] = MAPS3;
		//SendCANData = 1;
		Dashboard.disp_mode = DMODE_LEVEL;		
	}
}