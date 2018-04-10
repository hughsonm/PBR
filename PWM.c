/*
 * PWM.c
 *
 * Created: 4/23/2016 12:40:13 PM
 *  Author: Graeme Smith
 */ 

#include "PWM.h"

void PWMSetup()
{
	//------------------
	// Setup Drive I/O
	//------------------
	
	//Enable LED and Primary Power GPIO
	DDRB = 0b01111111;
	//Enable Secondary Power GPIO, Aux Drive, Fan Drive, Lambda Drive
	DDRE = 0b11111000;

	
	//---------------------------------------
	// Setup PWM Mode, Frequency, and Channels
	//---------------------------------------
	
	//Set TOP
	ICR3 = TOP;
	
	//Enable OC3A,OC3C, select mode 14
	//TCCR3A = 0b10001010;
	TCCR3A = (TIM_MATCH_CLEAR<<COM3A0)|(TIM_MATCH_CLEAR<<COM3C0)|(1<<WGM31);
	//TCCR3B = 0b00011101; //[2:0] clk/1024
	// Set waveform generation to fast pwm, TOP in ICR3
	TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS32)|(1<<CS30);
	
	TCCR3C = 0b00000000; //Not used for PWM modes. Datasheet says we must write zeros here for PWM modes.
	
	//Initialize drives to be off
	OCR3A = 0; //Set duty cycle to 0%
	OCR3C = 0; //Set duty cycle to 0%
	
	TCNT3 = 0x0000; //set counter to zero
}

void PWMDuty(channel_t channel, float duty)
{
	uint8_t temp;
	//Set PWM counter to fraction of TOP value
	if(channel == FuelPump)
	{
		temp = TCCR3A;
		temp &= ~((1<<COM3A1)|(1<<COM3A0));
		if(0.01 <= duty)
		{
			// Turn PWM back on and set duty			
			temp |= (TIM_MATCH_CLEAR<<COM3A0);
		}// else, turn it off to stop ticking.
		TCCR3A = temp;
		OCR3A = duty*TOP;
	}
	
	if(channel == Fan)
	{
		temp = TCCR3A;
		temp &= ~((1<<COM3C1)|(1<<COM3C0));
		if(0.01 <= duty)
		{
			// Turn PWM back on and set duty			
			temp |= (TIM_MATCH_CLEAR<<COM3C0);
		}// else, turn it off to stop ticking.
		TCCR3A = temp;		
		OCR3C = duty*TOP;
	}
}

void Output(output_t name, uint8_t state)
{
	if((name == Secondary))
	{
		if(state == ENABLE)
		{
			PORTE |= (1 << SECONDARY_PIN);
		}
		else
		{
			PORTE &= ~(1 << SECONDARY_PIN);
		}
	}
	if((name == Aux))
	{
		if(state == ENABLE)
		{
			PORTE |= (1 << AUX_PIN);
		}
		else
		{
			PORTE &= ~(1 << AUX_PIN);
		}
	}
	if((name == Lambda))
	{
		if(state == ENABLE)
		{
			PORTE |= (1 << LAMBDA_PIN);
		}
		else
		{
			PORTE &= ~(1 << LAMBDA_PIN);
		}
	}
	if((name == Primary))
	{
		if(state == ENABLE)
		{
			PORTB |= (1 << PRIMARY_PIN);
		}
		else
		{
			PORTB &= ~(1 << PRIMARY_PIN);
		}
	}
	if((name == LED1))
	{
		if(state == ENABLE)
		{
			PORTB &= ~(1 << LED1_PIN);
		}
		else
		{
			PORTB |= (1 << LED1_PIN);
		}
	}
	if((name == LED2))
	{
		if(state == ENABLE)
		{
			PORTB &= ~(1 << LED2_PIN);
		}
		else
		{
			PORTB |= (1 << LED2_PIN);
		}
	}
	if((name == LED3))
	{
		if(state == ENABLE)
		{
			PORTB &= ~(1 << LED3_PIN);
		}
		else
		{
			PORTB |= (1 << LED3_PIN);
		}
	}
	if((name == LED4))
	{
		if(state == ENABLE)
		{
			PORTB &= ~(1 << LED4_PIN);
		}
		else
		{
			PORTB |= (1 << LED4_PIN);
		}
	}
	if((name == LED5))
	{
		if(state == ENABLE)
		{
			PORTB &= ~(1 << LED5_PIN);
		}
		else
		{
			PORTB |= (1 << LED5_PIN);
		}
	}
	if((name == LED6))
	{
		if(state == ENABLE)
		{
			PORTB &= ~(1 << LED6_PIN);
		}
		else
		{
			PORTB |= (1 << LED6_PIN);
		}
	}
}

void ToggleLED(uint8_t name)
{
	PORTB ^= (1 << name);
}