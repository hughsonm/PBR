/*
 * timer.c
 *
 * Created: 2016-03-18 4:04:37 PM
 *  Author: Max
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "types.h"
#include "canbus.h"
#include "output.h"

u32 Tick_ms = 0;
u32 RunTime = 0;
extern dashState Dashboard;


void timer_init(void)
{
	cli();
	TCCR1B = (1<<CS11)|(1<<WGM12);
	TCCR1C = (1<<FOC1A);
	OCR1A = 1999;	// Determined experimentally to make a period of 1 ms.
	TIMSK1 |= 1<<OCIE1A;
	TCNT1 = 0x0000;	
	sei();	
}

ISR(TIMER1_COMPA_vect)
{	
	Tick_ms++;
	RunTime++;
}