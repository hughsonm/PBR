/*
 * cycleButtons.h
 *
 * Created: 2014-03-09 00:16:30
 *  Author: Brandon Hill
 */ 

#ifndef BUTTONS_H
#define BUTTONS_H

#define ENOUGH_DB	4000

#define START_LED_PIN	(1<<PA4)
#define LAUNCH_LED_PIN	(1<<PA3)

#define NEG_EDG_INT	0x02
#define POS_EDG_INT	0x03
#define ANY_EDG_INT	0x01

#define SW_POS_0	0b11100000 
#define SW_POS_1	0b11010000
#define SW_POS_2	0b10110000
#define SW_POS_3	0b01110000
#define SW_PINS		0b11110000

#include "atmega.h"

void button_init(void);
uint8_t get_rotary_pos(void);

#endif /* CYCLEBUTTONS_H_ */