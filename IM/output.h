/*
 * output.h
 *
 * Created: 2014-03-08 23:42:20
 *  Author: Brandon Hill
 */ 

#ifndef OUTPUT_H_
#define OUTPUT_H_

#include "as1107.h"
#include "types.h"

#define UPDATE_PERIOD_MS 200

#define N_STAT_LEDS 4
#define W_TEMP 0
#define OIL_PR 1
#define FUEL_L 2
#define BATT_V 3

#define STAT_OK		0
#define STAT_WARN	1
#define STAT_FAIL	2

#define WATER_GOOD	95
#define WATER_BAD	100

#define OIL_BAD		10

#define FUEL_GOOD	50
#define FUEL_BAD	10

#define VOLTAGE_GOOD	120
#define VOLTAGE_BAD		100

#define RGB_GREEN	0b01000000
#define	RGB_BLUE	0b00000100
#define RGB_RED		0b00000010
#define RGB_YELLOW	(RGB_RED|RGB_GREEN)
#define RGB_PURPLE	(RGB_RED|RGB_BLUE)
#define RGB_CYAN	(RGB_GREEN|RGB_BLUE)
#define RGB_WHITE	(RGB_BLUE|RGB_GREEN|RGB_RED)

#define BANK_AUX	0
#define BANK_CRIT	1

#define BANK_D0		0
#define BANK_D1		1
#define BANK_D2		2
#define BANK_D3		3

#define MAX_BRIGHTNESS 15
#define MAP_SHOWTIME_MS	1000

typedef struct {
	u16 bank_vals[2]; 	// Values to be displayed on the 7 segment displays
	u8 data_id[2];	// Type of data being displayed on the top and bottom 7 seg
	u8 is_critical[2][4];
	u8 brightness;
	u8 show_map;
	u32 show_start;
	} indModuleState;
	
#define NOT_CRITICAL	0
#define IS_CRITICAL		(!NOT_CRITICAL)



void output_setup(void);
void update_display(void);
void increase_brightness(u8 amount);
void decrease_brightness(u8 amount);
u8 debug_toggle(u8 pin);


#endif /* OUTPUT_H_ */
