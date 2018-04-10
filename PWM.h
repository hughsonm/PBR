/*
 * PWM.h
 *
 * Created: 4/23/2016 12:40:52 PM
 *  Author: Graeme Smith
 */ 


#ifndef PWM_H_
#define PWM_H_

//Includes
#include "main.h"

//Definitions
// Timer/Counter 3 is set up in Mode 14, fast PWM mode.
// The value of TOP is determined by the value in ICR3.
// Therefore, this value is written into ICR3.
#define TOP 0x2FE

#define PRIMARY_PIN    PB0
#define SECONDARY_PIN  PE7
#define AUX_PIN		   PE6
#define LAMBDA_PIN     PE4
#define LED1_PIN       PB1 //Module Power
#define LED2_PIN       PB2 //CAN Communication
#define LED3_PIN       PB3 //Fuel Pump
#define LED4_PIN       PB4 //Fan
#define LED5_PIN       PB5 //
#define LED6_PIN       PB6 //Undervoltage

#define DISABLE    0
#define ENABLE     1

typedef enum
{
	Primary,
	Secondary,
	Aux,
	Lambda,
	LED1,
	LED2,
	LED3,
	LED4,
	LED5,
	LED6
}output_t;

typedef enum
{
	Fan,
	FuelPump	
}channel_t;

//Prototypes
extern void PWMSetup(void); //Initialize Drives
extern void PWMDuty(channel_t channel, float duty); //Set duty cycle of specific channel
extern void Output(output_t name, uint8_t state); //Change state of one of the drives
extern void ToggleLED(uint8_t name); //Toggle one of the LEDs

// Timer/Counter Register Definitions
#define TIM_MATCH_DISC		0b00000000	// Timer disconnected
#define TIM_MATCH_TOGGLE	0b00000001	// Output toggles on match
#define TIM_MATCH_CLEAR		0b00000010	// Output clears on match#define I
#define TIM_MATCH_SET		0b00000011	// Output goes high on match

#define PWM_DISCONNECTED	0b00000000	// No PWM
#define PWM_WGM10_9BIT		0b00000010
#endif /* PWM_H_ */