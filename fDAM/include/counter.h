/*
 * counter.h
 *
 *  Created on: Aug 21, 2017
 *      Author: maxak
 */

#ifndef COUNTER_H_
#define COUNTER_H_

#define WS_TX_PERIOD_MS	20
#define	WS_TICK_FREQ	40000

#include "main.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

void	Counter_Init(void);

uint32_t Counter_PackFreqs(uint16_t * freqs);
void Counter_UnpackFreqs(uint32_t package, uint16_t *dest);



#endif /* COUNTER_H_ */
