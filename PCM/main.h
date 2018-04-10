#ifndef ATMEGA_H_
#define ATMEGA_H_

#define F_CPU 16000000UL //16MHz external clock frequency
#define TRUE  1
#define FALSE 0

#define CAN_TX_PERIOD		1000
#define CANLOSS_PERIOD		2000
#define SECONDARY_ON_DELAY	1000
#define LAMBDA_TO_PERIOD	10000
#define DEBUG_PERIOD		1000
#define ADC_READ_PERIOD		100
#define UNDERVOLTAGE_PERIOD	100000

//ADC Channels
#define N_ADC_CHANNELS		3

#define BATTERYVOLTAGE		3
#define PRIMARYCURRENT		0
#define SECONDARYCURRENT	1
#define BATTERYTEMP			2

#define VOLTAGE_DIV_RATIO	3.34

#include <limits.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "canbus.h"
#include "ADC.h"
#include "PWM.h"
#include "i2c.h"

typedef struct
{
	float primary;
	float secondary;
}loadData_t;

typedef enum
{
	UnderVoltage,
	NormalVoltage
}batteryVoltage_t;

typedef enum
{
	OverTemp,
	NormalTemp
}batteryTemp_t;

typedef enum
{
	LeadAcid,
	Lithium
}batteryType_t;

typedef struct  
{
	batteryTemp_t tempState;
	uint8_t temp;
	batteryVoltage_t voltageState;
	float voltage;
	batteryType_t type;
}battery_t;

extern unsigned char CANECUdata[];
extern unsigned short canlosstimeout;
extern unsigned short lambdatimeout;


#endif /* ATMEGA_H_ */