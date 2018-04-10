/*
 * canbus.h
 *
 * Created: 2014-03-08 23:42:01
 *  Author: Brandon Hill
 */ 

#ifndef CANBUS_H_
#define CANBUS_H_

#include "atmega.h"

// Rx Data Index Definitions:

#define LEN_VEH_DATA 19

extern u8 VehicleData[LEN_VEH_DATA];



#define COOLANT_TEMP		0
#define OIL_PRESSURE		1
#define FUEL_LEVEL			2
#define GEAR_POSITION		3
#define ENGINE_SPEED		4
// ENGINE_SPEED takes two bytes
#define THROTTLE_POS		6
#define FUEL_TEMP			7
#define LAMBDA				8
#define VEHICLE_SPEED		9
#define BATT_VOLTAGE		10
#define BATT_TEMP			11
#define PRIMARY_CURRENT		12
#define SECONDARY_CURRENT	13
#define ACCEL_LAT			14
// Each ACCEL_ takes two bytes
#define ACCEL_FB			16
//
#define ACCEL_UD			18
//


#define VEHICLE_RPM		((((u16)(VehicleData[ENGINE_SPEED])) << 8)|((u16)(VehicleData[ENGINE_SPEED + 1])))
#define ACCEL_16_LAT	(-((u16)( (((u16)(VehicleData[ACCEL_LAT]))<<8) | ((u16)(VehicleData[ACCEL_LAT+1])) )))		//((s16)((u16)(VehicleData[ACCEL_LAT]<<8)|(u16)(VehicleData[ACCEL_LAT+1]))
#define ACCEL_16_FB		((u16)( (((u16)(VehicleData[ACCEL_FB]))<<8) | ((u16)(VehicleData[ACCEL_FB+1])) ))//((s16)((u16)(VehicleData[ACCEL_FB]<<8)|(u16)(VehicleData[ACCEL_FB+1]))
#define ACCEL_MAX_ABS	3720

// Tx (Dash to ECU) Data Index Definitions
#define LAUNCH_CONTROL_REQ	0
// Rx (ECU to Dash) Data Index Definitions
#define LAUNCH_CONTROL_ACT	0


#define MAPS0			0
#define MAPS1			1
#define MAPS2			2
#define MAPS3			3

#define SIGNAL_OFF		0x00
#define SIGNAL_ON		0xFF

#define LAUNCH_STATE_OFF	0
#define LAUNCH_STATE_REV	1
#define	LAUNCH_STATE_RUN	2

//////////////////////////////
// CAN bus identifiers
//////////////////////////////
// Max ID is 0x1FFFFFFF
#define VEH_DATA1_MOB	0
#define VEH_DATA1_ID	0x00002000
#define VEH_DATA1_DLC	8

#define VEH_DATA2_MOB	1
#define VEH_DATA2_ID	0x00002001
#define VEH_DATA2_DLC	2

#define POWER_DATA_MOB	2
#define POWER_DATA_ID	0x00002002
#define POWER_DATA_DLC	4

#define DASH2ECU_MOB	3
#define DASH2ECU_ID	0x00003000
#define DASH2ECU_DLC	1

#define BRIGHTNESS_MOB	4
#define BRIGHTNESS_ID	0x00006000
#define BRIGHTNESS_DLC	1

#define DAM1_MOB		5
#define DAM1_ID			0x00005002
#define DAM1_DLC		6

#define ECU2DASH_MOB	6
#define ECU2DASH_ID		0x00003001
#define ECU2DASH_DLC	1

#define	MOB_OFF		0x00
#define	MOB_TX_EN	0x01
#define MOB_RX_EN	0x02
#define MOB_BUFF_RX	0x03

#define DLC_BITS	0x0F

void can_rx_inter(u8 mobnum);

void can_setup(void);

void canEnable();
void canDisable();
void canReset();

void zeroMOB(char mobnum);
void writeRXMOB(u8 mobnum, u32 indentifier, u8 dlc);
void writeTXMOB(u8 mobnum, u32 indentifier, u8 *data_ptr, u8 data_len);

#endif /* CANBUS_H_ */

