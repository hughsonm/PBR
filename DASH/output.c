/*
 * output.c
 *
 * Created: 2014-03-08 23:40:17
 *  Author: Brandon Hill
 */ 

#include "output.h"
#include "gear.h"
#include "as1107.h"
#include "types.h"
#include "canbus.h"
#include <stdio.h>

dashState Dashboard;
//static u16 Thresh_RPM[7] = {1858, 3716, 5573, 7431, 9289, 11147, 13000};
static u16 Thresh_RPM[7][6] = {
	2167, 4333, 6500, 8667,10833,13000,
	2167, 4333, 6500, 8667,10833,13000,
	1958, 3917, 5875, 7833, 9792,11750,
	1900, 3800, 5700, 7600, 9500,11400,
	1833, 3667, 5500, 7333, 9167,11000,
	1833, 3667, 5500, 7333, 9167,11000,
	1833, 3667, 5500, 7333, 9167,11000
	};
u8 DashLEDs[7];
u8 DashMat[8];
s16	CursorThresh[N_CURSOR_THRESH] = {
	-ACCEL_MAX_ABS,
	-ACCEL_MAX_ABS*3/4,
	-ACCEL_MAX_ABS/2,
	-ACCEL_MAX_ABS/4,
	0,
	ACCEL_MAX_ABS/4,
	ACCEL_MAX_ABS/2,
	ACCEL_MAX_ABS*3/4,
	ACCEL_MAX_ABS};

//extern const u16 GEAR[10];
extern u32 RunTime;


void output_setup(void)
{
	u8 system;
	led_init(LED_CS);
	led_init(MAT_CS);

	Dashboard.gear = 0;
	for (system = W_TEMP_LED; system <= BATT_V_LED; system++)
	{
		Dashboard.sys_status[system] = STAT_OK;
	}
	Dashboard.rpm_level = 0;
	Dashboard.brightness = MAX_BRIGHTNESS/2;
	Dashboard.disp_mode = get_rotary_pos();
	Dashboard.blink = 0;
	PORTA |= (DBG_CAN_LED) | (DBG_TIC_LED) | (DBG_RUN_LED);
	DDRA |= (DBG_CAN_LED) | (DBG_TIC_LED) | (DBG_RUN_LED);
}

void calc_dash_led(void)
{
	u8 led_num;
	u8 system_status;
	u8 led_state = RGB_WHITE;
	u8 rpm_lev;
	// Set the status LEDs based on each system's warning level.
	for(led_num = 0; led_num < N_STAT_LEDS; led_num++)
	{
		system_status = Dashboard.sys_status[led_num];
		switch (system_status)
		{
		case STAT_OK:
			led_state = RGB_GREEN;
			break;
		case STAT_WARN:
			led_state = RGB_YELLOW;
			break;
		case STAT_FAIL:
			led_state = RGB_RED;
			break;
		default:	
			led_state = RGB_WHITE;
		}
		DashLEDs[led_num] = led_state;
	}

	Dashboard.gear = VehicleData[GEAR_POSITION];
	Dashboard.gear = (Dashboard.gear > MAX_DISP_GEAR)?MAX_DISP_GEAR:Dashboard.gear;
	// When to blink dash:
	// Not in neutral AND
	// A higher gear does exist AND
	// RPM is above the 'shift now' threshold
	Dashboard.blink = (Dashboard.rpm_level>=RPM_UP_LVL) && (Dashboard.gear < MAX_DISP_GEAR) && (Dashboard.gear != 0);
	rpm_lev = Dashboard.rpm_level;

	DashLEDs[RPM_LEFT] = 0;
	DashLEDs[RPM_MID] = 0; 
	DashLEDs[RPM_RIGHT] = 0;	
	switch(rpm_lev)
	{
	case 6:			
		DashLEDs[RPM_MID] |= LED_2;
	case 5:
		DashLEDs[RPM_MID] |= LED_1|LED_3;
	case 4:
		DashLEDs[RPM_MID] |= LED_0|LED_4;
	case 3:
		DashLEDs[RPM_LEFT] |= LED_3;
		DashLEDs[RPM_RIGHT] |= LED_0;
	case 2:
		DashLEDs[RPM_LEFT] |= LED_2;
		DashLEDs[RPM_RIGHT] |= LED_1;
	case 1:
		DashLEDs[RPM_LEFT] |= LED_1;
		DashLEDs[RPM_RIGHT] |= LED_2;
	case 0:
		DashLEDs[RPM_LEFT] |= LED_0;
		DashLEDs[RPM_RIGHT] |= LED_3;
		break;
	default:
		DashLEDs[RPM_LEFT] = 0;
		DashLEDs[RPM_MID] = 0;
		DashLEDs[RPM_RIGHT] = 0;
	}
	
	// Are we running? If so, turn on the button's LED.
	if(VEHICLE_RPM > 1800)
	{
		PORTA &= ~(START_LED_PIN);// | DBG_RUN_LED);
	} else
	{
		PORTA |= (START_LED_PIN);// | DBG_RUN_LED);
	}
}

void calc_MatrixLED(void)
{
	// Dash Matrix Calculations >>
	u8 *row_ptr1;
	u8 *row_ptr2;
	s16 acc_xy[2];
	u8 cursor[2];
	u8 dim,tt,rr;

	static u8 letter_pos = 0;
	static u8 switch_pos = 0;	
	char msg[MAX_DISP_LEN];				
	
	u8 data1,data2,fontIdx,length;	

	static u32 end_time = 0;
	
	switch (Dashboard.disp_mode)
	{
	case DMODE_STR:
		length = (uint8_t)sprintf(msg," PBR18 ");
		break;
	case DMODE_ACC:
		/*sprintf(msg," U A LIL BITCH ");
		length = 15;
		break;
		*/
	case DMODE_LEVEL:
		length = sprintf(msg," Sam was faster ");
		break;
	}
	
	switch (Dashboard.disp_mode)
	{			
	case DMODE_GEAR:
		if (Dashboard.gear == 0x00)
		{
			fontIdx = 'N';
		}
		else 
		{
			if((Dashboard.rpm_level>=RPM_UP_LVL) && (Dashboard.gear < MAX_DISP_GEAR))
			{
				fontIdx = '^';
			}
			else
			{
				fontIdx = Dashboard.gear + '0';
			}
		} 

		for (u8 pp = 0; pp < 8; pp++)
		{
			DashMat[pp] = font[fontIdx][pp];
		}
		break;
	case DMODE_ACC:
		acc_xy[0] = ACCEL_16_LAT;
		acc_xy[1] = ACCEL_16_FB;
				
		cursor[0] = 7;
		cursor[1] = 7;
		// Find out where the cursor shall sit
		for(dim = 0;dim<2;dim++)
		{
			for(tt=1;tt<N_CURSOR_THRESH-1;tt++)
			{
				if(acc_xy[dim] < CursorThresh[tt])
				{
					cursor[dim] = tt-1;
					break;
				}
			}
		}
		// The screen's origin is the top-left, not the bottom-left,
		// so the y value has to get flipped.
		cursor[1] = 7-cursor[1];
		
		// Zero all the rows, except where the cursor should go.
		for(rr = 0;rr<8;rr++)
		{
			DashMat[rr] = 0;
			if(cursor[1] == rr)
			{
				DashMat[rr] = 0x01<<cursor[0];
			}
		}
		break;	
	case DMODE_LEVEL:	
	case DMODE_STR:		
		if (RunTime - end_time > REFRESH_SCREEN && (letter_pos < length))
		{
			row_ptr1 = & (font[0 + msg[letter_pos]][0]);
			row_ptr2 = & (font[0 + msg[letter_pos + 1]][0]);
			for (u8 pp = 0; pp < 8; pp++)
			{
				data1 = row_ptr1[pp] >> (switch_pos) & 0xff;
				data2 = row_ptr2[pp] << (8 - switch_pos) & 0xff;
				DashMat[pp] = data1 + data2;
			}
			if(switch_pos == 8)
			{
				switch_pos = 0;
				letter_pos++;
			}
			switch_pos++;
			end_time = RunTime;
		}
		if(letter_pos >= length)
		{
			letter_pos = 0;
		}
		break;
	}    		
}

void disp_dash_led(u8 *led_states, u8 ch_sel)
{
	/*
	Look at DashLEDs and send those data to the as1107
	*/
	static uint8_t	led_on = 0;
	static uint32_t	last_toggle_time = 0;
	if (ch_sel == LED_CS)
	{
		u8 addr, cmd;
		// Manual fast-blink routine
		if(Dashboard.blink)
		{
			if((RunTime - last_toggle_time) > DASH_BLINK_MS) led_on = !led_on;
		}
		else
		{
			led_on = 1;
		}
		// Send out led commands
		for(addr = DIG0; addr <= DIG6; addr++)
		{
			cmd = (led_on)?(led_states[addr - DIG0]):(0x00);
			led_cmd(LED_CS, addr, cmd);
		}		
	}
	if (ch_sel == MAT_CS) 
	{
		for (u8 ii = 0; ii < 8; ii++)
		{
			led_cmd(MAT_CS, DIG0 + ii, DashMat[ii]);
		}
	}
}

static void set_rpm_level(void)
{
	u8 ii;
	u16 rpm = VEHICLE_RPM;
	Dashboard.rpm_level = 6;
	for (ii = 0; ii < 6; ii++)
	{
		if(rpm < Thresh_RPM[Dashboard.gear][ii])
		{
			Dashboard.rpm_level = ii;
			break;
		}
	}
}

static void set_system_status(void)
{
	// Analyze Water Temp
	Dashboard.sys_status[W_TEMP] = STAT_OK;
	if (VehicleData[COOLANT_TEMP] > WATER_GOOD)
	{
		Dashboard.sys_status[W_TEMP] = STAT_WARN;
	}
	if (VehicleData[COOLANT_TEMP] > WATER_BAD)
	{
		Dashboard.sys_status[W_TEMP] = STAT_FAIL;
	}
	
	// Analyze Oil Pressure
	Dashboard.sys_status[OIL_PR] = STAT_OK;
	if (VehicleData[OIL_PRESSURE] < OIL_BAD)
	{
		Dashboard.sys_status[OIL_PR] = STAT_FAIL;
	}
	
	// Analyze Fuel Level
	Dashboard.sys_status[FUEL_L] = STAT_OK;
	if(VehicleData[FUEL_LEVEL] < FUEL_GOOD)
	{
		Dashboard.sys_status[FUEL_L] = STAT_WARN;
	}
	if (VehicleData[FUEL_LEVEL] < FUEL_BAD)
	{
		Dashboard.sys_status[FUEL_L] = STAT_FAIL;
	}
	
	// Analyze Battery Voltage
	Dashboard.sys_status[BATT_V] = STAT_OK;
	if (VehicleData[BATT_VOLTAGE] < VOLTAGE_GOOD)
	{
		Dashboard.sys_status[BATT_V] = STAT_WARN;
	}
	if (VehicleData[BATT_VOLTAGE] < VOLTAGE_BAD)
	{
		Dashboard.sys_status[BATT_V] = STAT_FAIL;
	}
}

void disp_graphic(u8 graphic_num) {
	u8 *row_ptr = &(graphics[graphic_num][0]);
		for (u8 jj = 0; jj < 8; jj++)
		{
			led_cmd(MAT_CS, DIG0 + jj, row_ptr[jj]);
		}
		_delay_ms(1000);
}

static void set_brightness(void)
{
	led_cmd(LED_CS, INTENCON, DUTY_MAX + Dashboard.brightness - MAX_BRIGHTNESS);
	led_cmd(MAT_CS, INTENCON, DUTY_MAX + Dashboard.brightness - MAX_BRIGHTNESS);
}

void update_dash(void)
{
	set_rpm_level();
	set_system_status();
	set_brightness();	
	calc_dash_led();	
	calc_MatrixLED();
	disp_dash_led(DashLEDs, LED_CS);
	disp_dash_led(DashMat, MAT_CS);	
}


