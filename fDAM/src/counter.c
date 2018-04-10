/*
 * counter.c
 *
 *  Created on: Aug 21, 2017
 *      Author: maxak
 */

#include "counter.h"

uint16_t	WheelCount[2] = {0,0};


void EXTI0_IRQHandler(void)
{
	WheelCount[0]++;
	EXTI_ClearFlag(EXTI_Line0);
}

void EXTI1_IRQHandler(void)
{
	WheelCount[1]++;
	EXTI_ClearFlag(EXTI_Line1);
}

uint32_t Counter_PackFreqs(uint16_t * freqs)
{
	uint32_t	packed;
	packed = ((freqs[0]<<16)|(freqs[1]));
	return(packed);
}

void Counter_UnpackFreqs(uint32_t package, uint16_t *dest)
{
	dest[0] = (uint16_t)(package>>16);
	dest[1] = (uint16_t)(package & 0x0000FFFF);
}

void Counter_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	EXTI_InitTypeDef	EXTI_InitStruct;
	NVIC_InitTypeDef	NVIC_InitStruct;

	/*
	 * Directions taken from stm32f4xx_exti.c
	 * Configure the I/O in input mode using GPIO_Init()
	 * Select the input source pin for the EXTI line using SYSCFG_EXTILineConfig()
	 * Select the mode(interrupt, event) and configure the trigger
	 * selection (Rising, falling or both) using EXTI_Init()
	 * Configure NVIC IRQ channel mapped to the EXTI line using NVIC_Init()
	 */

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);
	EXTI_InitStruct.EXTI_Line = EXTI_Line1;
	EXTI_Init(&EXTI_InitStruct);
	
	/*
	 * This interrupt receives the highest priority, since
	 * its service routine is very short - just 
	 * incrementing a counter. If wheel speed pulses
	 * arrive too frequently, however, the BNO I2C ISR
	 * will not be able to run its I2C state machine 
	 * fast enough. This will cause the I2C state 
	 * machine to break down, which will cause a 
	 * watchdog restart. 
	 * The maximum frequency of wheel speed pulses
	 * is ~200 kHz. This corresponds to a wheel speed
	 * of 22 393 km/h, which is crazy fast.
	 */

	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStruct);


}



