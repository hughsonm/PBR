#include "main.h"

battery_t battery;
loadData_t load;

unsigned char CANECUdata[8], CANECUdataTx[8];
unsigned short canlosstimeout, lambdatimeout;
uint32_t	undervoltagetimeout=0;
uint8_t currentReadCount;
uint16_t debugCount, canTxCount = 0;

// ADS Globals
extern	int16_t	ADSReadings[4];
extern	int32_t	LoadMilliAmps[4];
int32_t	LoadOffsetsMicroAmps[4] = {198,233,129,167};


void batteryVoltageCallback(const uint8_t d)
{
	battery.voltage = ((((float)d)*(VOLTAGE_DIV_RATIO)*5.0/255.0)); //Using 8 bit resolution to test functionality, resistor tolerances affect it
	if (battery.voltage > 11.5) //Put case for if Engine is off?
	{
		battery.voltageState = NormalVoltage;
	}
	else
	{
		battery.voltageState = UnderVoltage;
	}
}

void primaryCurrentCallback(const uint8_t d)
{
	load.primary = (((((float)d)/255.0)*5.0)-2.5)/0.066; 
}

void secondaryCurrentCallback(const uint8_t d)
{
	load.secondary = (((((float)d)/255.0)*5.0)-2.5)/0.066;
}


void setup()
{
	uint8_t i;
	for (i = 0; i < 8; i++) // explicitly zero all can data
	{
		CANECUdata[i] = 0;
	}
	for (i = 0; i < 8; i++) // explicitly zero all can data
	{
		CANECUdataTx[i] = 0x00;
	}
	
	//DDRE = 0b11111000;
	//DDRB = 0b00000001;
	
	PWMSetup();
	//ADCSetup();
	adc_init(BATTERYVOLTAGE);
	adc_set_conversion_callback(&batteryVoltageCallback);
	adc_enable();	
	canSetup();
	i2c_init();
	//Enable CAN STB pin, Set CAN TX as output
	DDRD = 0b10100000;
	//Enable CAN
	PORTD = 0b00000000;
	
	//Default LED states
	Output(LED1,ENABLE);
	Output(LED2,DISABLE);
	Output(LED3,DISABLE);
	Output(LED4,DISABLE);
	Output(LED5,DISABLE);
	Output(LED6,DISABLE);

}

int main(void)
{
	setup();
	
	Output(Primary,ENABLE);
	_delay_ms(SECONDARY_ON_DELAY);
	Output(Secondary,ENABLE);
	
	PWMDuty(Fan,1);
	//PWMDuty(FuelPump,0.5);
	//_delay_ms(10000);
	PWMDuty(FuelPump,0);
	//PWMDuty(Fan,0);
	
	
	uint8_t FuelPumpEnable = 0, FanEnable = 0, EngineRun = FALSE;
	volatile uint8_t ADCIndex = 0;
	float FuelPumpDuty = 0, FanDuty = 0;
	
    while(1)
    {
		//Parse newest CAN data from ECU
//		FanEnable = ((CANECUdata[0] & 0b00000011)==0b00000010)?TRUE:FALSE;
//		FanDuty = ((CANECUdata[0]&0b11111100)>>2)/63;
//		FuelPumpEnable = ((CANECUdata[1] & 0b00000011)==0b00000010)?TRUE:FALSE;
//		FuelPumpDuty = ((CANECUdata[1]&0b11111100)>>2)/63;
//		EngineRun = ((CANECUdata[2] & 0b00000001)==0b00000001)?TRUE:FALSE;
		
		//Parse newest CAN data from ECU
		FanEnable =			   (CANECUdata[CAN_PWR_FAN_EN])?TRUE:FALSE;
		FanDuty =      ((float)(CANECUdata[CAN_PWR_FAN_LV]))/100.0;
		FuelPumpEnable =       (CANECUdata[CAN_PWR_PUMP_EN])?TRUE:FALSE;
		FuelPumpDuty = ((float)(CANECUdata[CAN_PWR_PUMP_LV]))/100.0;
		EngineRun =            (CANECUdata[CAN_PWR_ENG_ON])?TRUE:FALSE;
		battery.type =         (CANECUdata[CAN_PWR_BAT_TP])?Lithium:LeadAcid;

		//Fan Control
		if(FanEnable)
		{
			PWMDuty(Fan,FanDuty);
			ToggleLED(LED4_PIN);
		}
		else
		{
			PWMDuty(Fan,0);
			Output(LED4,DISABLE);
		}
		
		//Fuel Pump Control
		if(FuelPumpEnable)
		{
			//PWMDuty(FuelPump,FuelPumpDuty);
			PWMDuty(FuelPump,FuelPumpDuty);
			ToggleLED(LED3_PIN);
		}
		else
		{
			PWMDuty(FuelPump,0);
			Output(LED3,DISABLE);
		}
		
		
		
		//Lambda Control
		if((!EngineRun)&&(lambdatimeout > LAMBDA_TO_PERIOD))
		{
			lambdatimeout = 0;
			Output(Lambda,DISABLE);
		}
		else
		{
			Output(Lambda,ENABLE);
		}
		
		//CAN Timeout
		if (canlosstimeout >= CANLOSS_PERIOD)
		{
			canlosstimeout = 0;
			uint8_t i;
			for (i = 0; i < 8; i++) //explicitly zero all can data
			{
				CANECUdata[i] = 0;
			}
			
			Output(LED2,DISABLE);
		}
		
		undervoltagetimeout = (battery.voltageState==UnderVoltage)?undervoltagetimeout+1:0;
		if(undervoltagetimeout >= UNDERVOLTAGE_PERIOD)
		{
			// Under-voltage has gone on for too long. Cut power.
			ToggleLED(LED6_PIN);
			Output(Secondary,DISABLE);
			Output(Primary,DISABLE);
			Output(Aux,DISABLE);
			Output(Lambda,DISABLE);
		}
		else
		{
			// Let power flow.
			Output(Aux,ENABLE);
			Output(LED6,DISABLE);
			Output(Primary,ENABLE);
			Output(Secondary,ENABLE);
		}

		for(uint8_t ii = 0; ii<4;ii++)
		{
			int32_t rdg = (int32_t)(ADSReadings[ii]);
			LoadMilliAmps[ii] = 6*rdg - 12*LoadOffsetsMicroAmps[ii];
		}
		
		//Take current readings every so often and transmit all data
		if(currentReadCount > ADC_READ_PERIOD)
		{			
			switch(ADCIndex)
			{
				case 0:					
					adc_disable();
					adc_init(PRIMARYCURRENT);
					adc_set_conversion_callback(&primaryCurrentCallback);
					adc_enable();
					break;					
				case 1:
					adc_disable();
					adc_init(SECONDARYCURRENT);
					adc_set_conversion_callback(&secondaryCurrentCallback);
					adc_enable();
					break;					
				case 2:
					adc_disable();
					adc_init(BATTERYVOLTAGE);
					adc_set_conversion_callback(&batteryVoltageCallback);
					adc_enable();
					break;
			}
			ADCIndex++;
			if(ADCIndex >= N_ADC_CHANNELS)
			{
				ADCIndex=0;
			}
			currentReadCount = 0;
		}
		
		//Periodically send Power Data CAN Packet
		if(canTxCount>CAN_TX_PERIOD)
		{
			CANECUdataTx[0] = (uint8_t)(battery.voltage*10);
			CANECUdataTx[1] = (uint8_t)(battery.temp);
			CANECUdataTx[2] = (uint8_t)(load.primary*10);
			CANECUdataTx[3] = (uint8_t)(load.secondary*10);
			writeTXMOB(CAN_PWRDATA_MOB,CAN_PWRDATA_ID,&CANECUdataTx[0], CAN_PWRDATA_DLC);
			canTxCount = 0;
		}
		
		//Increment timeouts and add loop delay
		canlosstimeout++;
		lambdatimeout++;
		currentReadCount++;
		debugCount++;
		canTxCount++;
		
		if(debugCount>DEBUG_PERIOD)
		{
			debugCount = 0;
		}
		//_delay_ms(10);
	}
}

/*
int main(void)
{
	//------------------------------
	// SETUP, move to function later
	//------------------------------
	
	//Enable LED and Primary Power GPIO
	DDRB = 0b01111111;
	//Enable Secondary Power GPIO, Aux Drive, Fan Drive, Lambda Drive
	DDRE = 0b11111000;
	//Enable PWM Drive GPIO
	//Enable ADC
	//Enable I2C
	//Enable CAN
	
	

	//------------------------
	// PWM Timer Configuration
	//------------------------
	
	//Set TOP for mode 14
	short high = 0x02FE;
	ICR3 = high;
	
	//Enable OC3B, select mode 14
	TCCR3A = 0b10100010;
	TCCR3B = 0b00011101; //[2:0] clk/1024 from prescaler
	TCCR3C = 0b00000000; //Not used for PWM modes
	//OCR3B = (0xF*high)/63; //Set duty cycle
	OCR3B = 0; //Set duty cycle to 0%
	
	//Start up delay
	_delay_ms(1000);

	//Test
	PORTB = 0b00010000;
	_delay_ms(50);
	PORTE = 0b11101000;
	PORTB = 0b00000001;
	OCR3A = (0x17F*high)/63;
	OCR3B = (0x17F*high)/63; //Set duty cycle
	_delay_ms(10000);
	PORTE = 0b00000000;
	PORTB = 0b00000000;
	OCR3A = 0; //Set duty cycle 0%
	OCR3B = 0; //Set duty cycle 0%
	_delay_ms(50);
	
	while(1)
	{
		
//		TCCR3B = 0b00011000;
//		_delay_ms(10000);

		//PWM Testing

		TCCR3B = 0b00011011;//clk/64 =  
		_delay_ms(10000);
		TCCR3B = 0b00011101;//clk/1024 = 20 Hz
		_delay_ms(10000);
	
		
		//LED Blink test program
		PORTB = 0b00010000;
		_delay_ms(50);
		PORTB = 0b00000000;
		_delay_ms(50);
	}

}
*/

		
		
		//if(((CANECUdata[0] & 0b00000011) == 0b10)&&((CANECUdata[2] & 0b00000010) != 0b10))
		//{
		//OCR3C = (((CANECUdata[0]&0b11111100)>>2)*high)/63;
		//test = (((CANECUdata[0]&0b11111100)>>2)*high)/63;
		//}
		//else
		//{
		//OCR3C = 0;
		//}
		//if((CANECUdata[1] & 0b00000011) == 0b10)
		//{
		//OCR3A = (((CANECUdata[1]&0b11111100)>>2)*high)/63;
		//}
		//else
		//{
		//OCR3A = 0;
		//}
		//_delay_ms(10);
		//canlosstimeout++;
		//lambdatimeout++;
		//if (canlosstimeout >= 100)
		//{
		//canlosstimeout = 0;
		//CANECUdata[1] = 0;
		//}
		//if (!(CANECUdata[2] & 0b00000001))
		//{
		//if (lambdatimeout >= 10000)
		//{
		//lambdatimeout = 0;
		//PORTE &= 0b11101111;
		//}
		//}
		//else
		//{
		//PORTE |= 0b00010000;
		//}