/*
 * adc.c
 *
 *  Created on: Aug 20, 2017
 *      Author: maxak
 */
#include "main.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "adc.h"


void Analog_Init(void)
{
	/*
	 * Set up adc interrupts so they are read then restarted
	 * after each read.
	 * AIN1 and AIN2 are the analogs that we read.
	 * The are on PA0 and PA1. PA0 is ADC123_IN0. PA1 is ADC123_IN1.
	 *
	 */
	ADC_CommonInitTypeDef	ADC_Common;
	ADC_InitTypeDef			ADC_DAQ_Init;
	GPIO_InitTypeDef		GPIO_InitStruct;

	// AINs are on port A, so make sure port A is turned on.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//RCC_APB2PeriphClockCmd(ADCx_CLK, ENABLE);
	// Set PA0 and PA1 as ADC inputs
	// Enable ADC 1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	// Enable ADC 2 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	// Configure pins 0 and 1 of port A as ADC pins
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	ADC_Common.ADC_Mode = ADC_Mode_Independent;
	ADC_Common.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_Common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_Common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles;
	ADC_CommonInit(&ADC_Common);

	ADC_DAQ_Init.ADC_Resolution = ADC_Resolution_8b;
	ADC_DAQ_Init.ADC_ScanConvMode = DISABLE;
	ADC_DAQ_Init.ADC_ContinuousConvMode = ENABLE;
	ADC_DAQ_Init.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_DAQ_Init.ADC_NbrOfConversion = 1;
	ADC_DAQ_Init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_DAQ_Init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

	ADC_Init(ADC1,&ADC_DAQ_Init);
	ADC_Init(ADC2,&ADC_DAQ_Init);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
	ADC_Cmd(ADC1,ENABLE);
	ADC_Cmd(ADC2,ENABLE);
}

void Analog_StartConversions(void)
{
	ADC_SoftwareStartConv(ADC1);
	ADC_SoftwareStartConv(ADC2);
}

uint16_t Analog_PackReadings(uint8_t *readings)
{
	uint16_t packed;
	packed = (((uint16_t)(readings[0]))<<8) | ((uint16_t)(readings[1]));
	return(packed);
}

void Analog_UnpackReadings(uint16_t packed, uint8_t * unpacked)
{
	unpacked[0] = (uint8_t)((packed&0xFF00)>>8);
	unpacked[1] = (uint8_t)(packed&0x00FF);
}


