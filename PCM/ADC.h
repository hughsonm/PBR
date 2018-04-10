/*
 * adc.h
 * ADC support for the AT90CANxx microcontroller. The current implementation
 * uses 8-bit resolution with a sampling frequency of approx 9.615 kHz.
 * A newly converted byte is available from the ISR through a callback.
 *
 *  Created on: 2010-01-17
 *  Author: John Hughes <jondo2010@gmail.com>
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_DIV2	0x1		/// ~615.385 kHz
#define ADC_DIV4	0x2
#define ADC_DIV8	0x3		/// ~153.846 kHz
#define ADC_DIV16	0x4
#define ADC_DIV32	0x5		/// ~38.462 kHz
#define ADC_DIV64	0x6
#define ADC_DIV128	0x7		/// ~9.615 kHz

/**
 * Initialize the ADC to use Channel 0 to 7.
 * The ADC requires an input clock frequency between 50 and 200 kHz for max. resolution.
 * The current implementation hard-codes the clock division to 128, which results in an
 * ADC clock frequency of 125 kHz, and sampling frequency of approx. 9.615 kHz.
 */
void
adc_init
(
	uint8_t channel
);

/**
 * Enable the ADC, starts the first conversion.
 */
void
adc_enable ();

/**
 * Disable the ADC
 */
void
adc_disable ();

void
adc_set_conversion_callback
(
	void (*callback_func)(uint8_t byte)
);

#endif /* ADC_H_ */
