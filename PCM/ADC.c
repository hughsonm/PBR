/*
 * adc.c
 * ADC support for the AT90CANxx microcontroller. The current implementation
 * uses 8-bit resolution with a sampling frequency of approx 9.615 kHz.
 * A newly converted byte is available from the ISR through a callback.
 *
 *  Created on: 2010-01-17
 *  Author: John Hughes <jondo2010@gmail.com>
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "adc.h"

static void (*adc_conversion_callback)		(uint8_t byte)	= 0;

ISR (ADC_vect)
{
	uint8_t data = ADCH;

	if (adc_conversion_callback)
		adc_conversion_callback (data);
}

void
adc_init
(
	uint8_t channel
)
{
	/// Left justify data, set the input channel
	ADMUX = _BV (REFS0) | _BV (ADLAR) | (0x07 & channel);

	/// Enable intterupt, auto trigger mode, prescaler setting
	ADCSRA = _BV (ADIE) | _BV (ADATE) | (0x07 & ADC_DIV128);

	/// Free running mode
	ADCSRB = 0x00;
}

void
adc_enable ()
{
	ADCSRA |= _BV (ADEN) | _BV (ADSC);
}

void
adc_disable ()
{
	ADCSRA &= ~_BV (ADEN);
}

void
adc_set_conversion_callback
(
	void (*callback_func)(uint8_t byte)
)
{
	adc_conversion_callback = callback_func;
}
