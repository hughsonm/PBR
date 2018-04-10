/*
 * as1107.h
 *
 * Created: 2014-03-08 23:41:39
 *  Author: Brandon Hill
 */ 

#include "atmega.h"
#include "types.h"

#ifndef AS1107_H_
#define AS1107_H_

/*
Slave Select Lines 
*/
#define LED_CS 0
#define MAT_CS 4

/*
/ Wiring definitions
*/
#define LED_0 0b01000000
#define LED_1 0b00100000
#define LED_2 0b00010000
#define LED_3 0b00001000
#define LED_4 0b00000100

enum regNames {	NOOP, 
				DIG0,
				DIG1, 
				DIG2,
				DIG3, 
				DIG4,
				DIG5, 
				DIG6,
				DIG7,
				DECMODE,
				INTENCON,
				SCANLIM,
				SHUTDOWN,
				NA,			//Nothing... Not Available... Undefined
				FEATURE,
				DISPLAYTEST};

//****************************************
// SHUTDOWN Functions
//****************************************
#define SHD_DF      0x00    // Shutdown, clears FEATURE
#define SHD_UN      0x80    // Shutdown, FEATURE unchanged
#define NORM_DF     0x01    // Normal, clears FEATURE
#define NORM_UN     0x81    // Normal, FEATURE unchanged

//****************************************
// DEC_EN Functions
//****************************************
#define NO_DEC      0x00    // No decode for digits 7:0
#define DEC_0       0x01    // Decode digit 0
#define DEC_1       0x02
#define DEC_2       0x04
#define DEC_3       0x08
#define DEC_4       0x10
#define DEC_5       0x20
#define DEC_6       0x40
#define DEC_7       0x80
#define DEC_ALL     0xFF    // Decode all digits

//****************************************
// TEST_REG Functions
//****************************************
#define NORM_OP     0x00    // Normal Operation
#define TEST_OP     0x01    // Display Test Mode

//****************************************
// INT_CON Functions
// Determines duty cycle as a fraction x/16
//****************************************
#define DUTY_1      0x00
#define DUTY_2      0x01
#define DUTY_3      0x02
#define DUTY_4      0x03
#define DUTY_5      0x04
#define DUTY_6      0x05
#define DUTY_7      0x06
#define DUTY_8      0x07
#define DUTY_9      0x08
#define DUTY_10     0x09
#define DUTY_11     0x0A
#define DUTY_12     0x0B
#define DUTY_13     0x0C
#define DUTY_14     0x0D
#define DUTY_15     0x0E
#define DUTY_MAX    0x0F

//****************************************
// SC_LIM Functions
// Determines which digits to display
//****************************************
#define ONLY_0      0x00    // Only display dig 0
#define ONLY_0_1    0x01    // Only display dig 0:1
#define ONLY_0_2    0x02    // Only display dig 0:2
#define ONLY_0_3    0x03
#define ONLY_0_4    0x04
#define ONLY_0_5    0x05
#define ONLY_0_6    0x06
#define ONLY_0_7    0x07

//****************************************
// FEATURE Bit Functions
//****************************************
#define LED_CLK_EN  0x01    // 0-Use internal osc 1-Use CLK pin
#define REG_RES     0x02    // 0-No reset 1-default control regs on power-up
#define DEC_SEL     0x04    // 0-Code-B decoding 1-HEX decoding
#define LED_SPI_EN  0x08    // Only for AS1106. For AS1107, SPI always on
#define BLINK_EN    0x10    // 0-Disable Blinking 1-Enable Blinking
#define BLINK_FREQ  0x20    // blink freq = 0-1s, 1- 2s
#define FEAT_SYNC   0x40    // Blink on rising edge of CSN
#define BLINK_START 0x80    // 0-Blink when disp turned off 1- when disp on


void blockingDataSPIWrite(char reg, char data);
void blockingrgbSPIWrite(char reg, char data);
u8 led_cmd(u8 ch_sel, u8 address, u8 data);
void led_init(u8 ch_sel);

void dataSSOn();
void dataSSOff();

void rgbSSOn();
void rgbSSOff();

void ss_low(u8 pin);
void ss_hi(u8 pin);
#endif /* AS1107_H_ */