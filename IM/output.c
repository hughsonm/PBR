/*
 * output.c
 *
 * Created: 2014-03-08 23:40:17
 *  Author: Brandon Hill
 */ 

#include "IM_2017.h"
#include "output.h"
#include "as1107.h"
#include "types.h"
#include "canbus.h"


indModuleState IndModule;
extern	u8 CANTXData[IM_CTRL_DLC];
extern	carState Car;
extern	u32	RunTime;
u8	IMLEDs[2][5]; //0/1 for top/bottom. 0-3 for digits, 4 for RGB
const u8 DataIDMap[2][4] = 
{	
	{ENGINE_SPEED, VEHICLE_SPEED, THROTTLE_POS, LAMBDA},
	{COOLANT_TEMP, OIL_PRESSURE, FUEL_LEVEL, BATT_VOLTAGE},
	// These bottom four values should not be changed. 
	// They correspond to the warning lights on the dash
};

void output_setup(void)
{
	led_init(DATA1_CS);
	led_init(DATA2_CS);
	// Initialize the IndModule
	IndModule.data_id[0] = 0;
	IndModule.data_id[1] = 0;
	IndModule.brightness = 8;
	IndModule.show_map = 0;
}

void decrease_brightness(u8 amount)
{
	if(IndModule.brightness >= amount)
	{
		IndModule.brightness = IndModule.brightness - amount;
	}
}

void increase_brightness(u8 amount)
{
	IndModule.brightness = IndModule.brightness + amount;
	if(IndModule.brightness > MAX_BRIGHTNESS)
	{
		IndModule.brightness = MAX_BRIGHTNESS;
	}
}

void check_critical_systems(void)
{
	//PORTA = 0b0000001;
	//PORTA = PORTA<<1;
	//PORTA = PORTA<<1;
	carDatum *datum;
	u8 sys_val, make_blink = 0;
	// COOLANT_TEMP, OIL_PRESSURE, FUEL_LEVEL, BATT_VOLTAGE	
	datum = &(Car.car_data[COOLANT_TEMP]);
	sys_val = Car.val_arr[datum->arr_idx];
	if(sys_val > WATER_BAD)
	{
		if(!(IndModule.is_critical[0][0]))
		{//If this system hasn't been flagged as critical yet,
			//flag it as critical, set the display to this datum, make it blink
			IndModule.is_critical[0][0] = IS_CRITICAL;
			IndModule.data_id[0] = 0;
			make_blink = 1;	
		}
	} else
	{
		// Clear the is_critical flag, stop the blinking
		IndModule.is_critical[0][0] = NOT_CRITICAL;
	}
	
	datum = &Car.car_data[OIL_PRESSURE];
	sys_val = Car.val_arr[datum->arr_idx];
	if(sys_val < OIL_BAD)
	{
		if(!(IndModule.is_critical[0][1]))
		{//If this system hasn't been flagged as critical yet,
			//flag it as critical, set the display to this datum, make it blink
			IndModule.is_critical[0][1] = IS_CRITICAL;
			IndModule.data_id[0] = 1;
			make_blink = 1;
		}
	} else
	{
		// Clear the is_critical flag
		IndModule.is_critical[0][1] = NOT_CRITICAL;
	}
	
	datum = &Car.car_data[FUEL_LEVEL];
	sys_val = Car.val_arr[datum->arr_idx];
	if(sys_val < FUEL_BAD)
	{
		if(!(IndModule.is_critical[0][2]))
		{//If this system hasn't been flagged as critical yet,
			//flag it as critical, set the display to this datum, make it blink
			IndModule.is_critical[0][2] = IS_CRITICAL;
			IndModule.data_id[0] = 2;
			make_blink = 1;
		}
	} else
	{
		// Clear the is_critical flag
		IndModule.is_critical[0][2] = NOT_CRITICAL;
	}
	
	datum = &Car.car_data[BATT_VOLTAGE];
	sys_val = Car.val_arr[datum->arr_idx];
	if(sys_val < VOLTAGE_BAD)
	{
		if(!(IndModule.is_critical[0][3]))
		{//If this system hasn't been flagged as critical yet,
			//flag it as critical, set the display to this datum, make it blink
			IndModule.is_critical[0][3] = IS_CRITICAL;
			IndModule.data_id[0] = 3;
			make_blink = 1;
		}
	} else
	{
		// Clear the is_critical flag
		IndModule.is_critical[0][3] = NOT_CRITICAL;
	}	
	// scan each system in the bottom display.
	/* If the system is critical:
	- flag the data type in is_critical
	- Change the data type in IndModule to the critical system
	- set that bank to blinking	  */
	
	/*	
	If the system isn't critical:
	- flag the system as not critical in is_critical
	- make sure no banks are blinking
	*/
	if(make_blink)
	{
		led_cmd(DATA2_CS, FEATURE, BLINK_EN);
	} else
	{
		led_cmd(DATA2_CS, FEATURE, 0x00);
	}
	return;
}

void calc_im_led(void)
{
	/* Check the data type for the top and bottom display and:
		- Set the correct LED colour in IMLEDs[0/1][4]
		- decode the value for that data type and put it in IMLEDs[0/1][0-3] (us u16_to_digits to do the conversion). RPM is a special case.
	*/
	u8 bank, d_type, ii;
	u16 val=0;
	s16 sval;
	u16 datum_div;
	carDatum *datum_ptr;
	
	for(bank = 0; bank < 2; bank++)
	{
		d_type = IndModule.data_id[bank];
		// Find out which data value is to be displayed
		// Get the data from the carDatum
		datum_ptr = &(Car.car_data[DataIDMap[bank][d_type]]);
		switch((datum_ptr->type) & SIZE_BITS)
		{
			case SIZE_32:
				// None of the values are 32 bit yet
				break;				
			case SIZE_16:
				// Read the MSB
				val = Car.val_arr[datum_ptr->arr_idx];
				val = val<<8;
				// Read the LSB
				val |= Car.val_arr[datum_ptr->arr_idx + 1];
				break;
			case SIZE_8:
				val = Car.val_arr[datum_ptr->arr_idx];
				break;
			default:
				val = 0xFFFF;
				break;
		}
		switch((datum_ptr->type & SIGN_BIT))
		{
			case UNSIGNED:
				// do nothing
				break;
			case SIGNED:
				if(val & 0x80)
				{
					sval = (s16)val;
					sval = -sval;
					val = (u16)sval;	
				}
				
		}
		// Represent that data correctly
		// Put that data in IMLEDs
		datum_div = 1;
		for (ii = 1; ii < datum_ptr->n_digits; ii++)
		{
			datum_div *= 10;
		}
		
		u16_to_digits(val, &(IMLEDs[bank][0]), datum_div, datum_ptr->decimal);
		switch(d_type)
		{
			case 0:
				IMLEDs[bank][4] = RGB_WHITE;
				break;
			case 1:
				IMLEDs[bank][4] = RGB_RED;
				break;
			case 2:
				IMLEDs[bank][4] = RGB_GREEN;
				break;
			case 3:
				IMLEDs[bank][4] = RGB_BLUE;
				break;
			default:
				break;
		}
	}
	
	// Overwrite the auxiliary bank (0)
	// with the MAP signal if we just changed it
	if(IndModule.show_map)
	{
		if((RunTime - IndModule.show_start) < MAP_SHOWTIME_MS)
		{
			u16_to_digits(CANTXData[MAP_SIG], &(IMLEDs[BANK_AUX][BANK_D0]), 1, 0);
		}
		else
		{
			IndModule.show_map = 0;
		}
	}
	
	return;
}

void disp_im_led(void)
{
	u8 chip_sel, led_num;
	/*
	Write IMLEDs to the as1107s
	*/
	chip_sel = DATA1_CS;
	for(led_num = 0; led_num < 5; led_num++)
	{
		led_cmd(chip_sel, (DIG0 + led_num), IMLEDs[0][led_num]);
	}
	led_cmd(chip_sel, INTENCON, DUTY_MAX + IndModule.brightness - MAX_BRIGHTNESS);
	chip_sel = DATA2_CS;
	for(led_num = 0; led_num < 5; led_num++)
	{
		led_cmd(chip_sel, (DIG0 + led_num), IMLEDs[1][led_num]);
	}
	led_cmd(chip_sel, INTENCON, DUTY_MAX + IndModule.brightness - MAX_BRIGHTNESS);
	
}

void update_display(void)
{
	//check_critical_systems();
	calc_im_led();
	disp_im_led();
}

u8 debug_toggle(u8 pin)
{
	pin = (1<<pin);
	pin &= 0b00000111;
	PORTA ^= pin;
	return(pin);
}