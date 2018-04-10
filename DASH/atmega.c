/*
 * atmega.c
 *
 * Created: 2014-02-08 17:40:19
 *  Author: Brandon Hill
 */ 

#include "atmega.h"
#include "spi.h"
#include "canbus.h"
#include "output.h"
#include "timer.h"
#include "gear.h"

extern u8 VehicleData[LEN_VEH_DATA];
extern u8 CANTXData[DASH2ECU_DLC];
extern u8 SendCANData;
extern u32 Tick_ms;

/*
VehicleData Bytes:

0: Coolant Temp [degrees C]
1: Oil Pressure [PSI]
2: Fuel Level [%]
3: Gear Position
4-5: Engine Speed [RPM]
6: Throttle Position [%]
7: Fuel Temp [degrees C]
8: Lambda [ratio, scaled by 100]
9: Vehicle Speed [km/h]
10: Battery Voltage [V * 10]
11: Battery Temp [degrees C]
12: Vehicle Current [A * 10]
13: Fan Current [A * 10]
14: Fuel Pump Current [A * 10]
15: Aux Current [A * 10]
16: Lambda Current [A * 10]
*/

void setup()
{            
	u8 ii;
	for (ii = 0; ii < LEN_VEH_DATA; ii++) // explicitly zero all can data
	{
		VehicleData[ii] = 0;
	}	
	//debugLEDSetup();
	spi_init();	//how we talk to the led drivers
	button_init();	
	output_setup();	
	can_setup();
	timer_init();
}

int main(void)
{
	setup();
	update_dash();
		
    while(1)
    {		
		if(Tick_ms > DASH_UPDATE_MS)
		{			
			update_dash();
			Tick_ms = 0;
		}
		if (SendCANData)
		{
			writeTXMOB(DASH2ECU_MOB,DASH2ECU_ID,CANTXData,DASH2ECU_DLC);
			SendCANData = 0;
		}
	}
}