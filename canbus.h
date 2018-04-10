/*
 * canbus.h
 *
 * Created: 2014-03-08 23:42:01
 *  Author: Brandon Hill
 */ 

#ifndef CANBUS_H_
#define CANBUS_H_

#include "main.h"

void can_rx_inter();

void canSetup();

void canEnable();
void canDisable();
void canReset();



// CAN Packet Definitions
#define CAN_PWRCTRL_ID	0x00001000
#define CAN_PWRCTRL_DLC	6
#define CAN_PWRCTRL_MOB	0

#define CAN_PWRDATA_ID	0x00002002
#define CAN_PWRDATA_DLC	4
#define CAN_PWRDATA_MOB 1



#define CAN_PWR_PUMP_EN	0
#define CAN_PWR_PUMP_LV	1
#define CAN_PWR_FAN_EN	2
#define CAN_PWR_FAN_LV	3
#define CAN_PWR_ENG_ON	4
#define CAN_PWR_BAT_TP	5

void zeroMOB(char mobnum);
void writeRXMOB(uint8_t mobnum, uint32_t indentifier);
void writeTXMOB(uint8_t mobnum, uint32_t indentifier, unsigned char* CANdata, uint8_t length);

#endif /* CANBUS_H_ */