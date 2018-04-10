/*
 * adc.h
 *
 *  Created on: Aug 20, 2017
 *      Author: maxak
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_PERIOD_MS	50

void Analog_Init(void);
void Analog_StartConversions(void);
uint16_t Analog_PackReadings(uint8_t *readings);
void Analog_UnpackReadings(uint16_t packed, uint8_t * unpacked);



#endif /* ADC_H_ */
