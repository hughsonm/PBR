/*
 * canbus.h
 *
 * Created: 2014-03-08 23:42:01
 *  Author: Brandon Hill
 */ 

#ifndef CANBUS_H_
#define CANBUS_H_

#include "IM_2017.h"

typedef struct{
	u8 type;	// contains information about size and sign
	u8 arr_idx;	// where in the large array of raw data is this value
	//u16 mult;	// scaling factor for small numbers
	u8 n_digits;// maximum number of base-10 digits value requires
	u8 decimal;	// where the decimal place goes	
	} carDatum;
	
#define SIGN_BIT	0x80
#define SIGNED		0x80
#define UNSIGNED	0x00
#define SIZE_BITS	0b00000111
#define SIZE_8		1
#define SIZE_16		2
#define SIZE_32		4

typedef struct{
	u8 val_arr[26];
	carDatum car_data[19];
	} carState;

// Rx Data Index Definitions:
#define COOLANT_TEMP		0	// 1 byte
#define OIL_PRESSURE		1	// 1 byte
#define FUEL_LEVEL			2	// 1 byte
#define GEAR_POSITION		3	// 1 byte
#define ENGINE_SPEED		4	// 2 bytes
#define THROTTLE_POS		5	// 1 byte
#define FUEL_TEMP			6	// 1 byte
#define LAMBDA				7	// 1 byte
#define VEHICLE_SPEED		8	// 1 byte
#define BATT_VOLTAGE		9	// 1 byte
#define BATT_TEMP			10	// 1 byte
#define PRIMARY_CURRENT		11	// 1 byte
#define SECONDARY_CURRENT	12	// 1 byte
#define ACCEL_LAT			13	// 2 bytes signed
#define ACCEL_FB			14	// 2 bytes signed
#define ACCEL_UD			15	// 2 bytes signed
#define EULER_HD			16	// 2 bytes signed
#define EULER_RL			17	// 2 bytes signed
#define EULER_PT			18	// 2 bytes signed
#define BRAKE_PR_FR_L		19	
#define BRAKE_PR_FR_R		20
#define ROTOR_TEMP_3		21
#define	ROTOR_TEMP_4		22
#define W_SPEED_REAR_L		23
#define W_SPEED_REAR_R		24

#define MAP_SIG	0

#define TRAC_DOWN	0x00
#define TRAC_UP		0xFF

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

#define IM_CTRL_MOB		3
#define IM_CTRL_ID		0x00004000
#define IM_CTRL_DLC		1
#define N_ENG_MAPS		4

#define DAM1_MOB		4
#define DAM1_ID			0x00005002
#define DAM1_DLC		6

#define DAM2_MOB		5
#define DAM2_ID			0x00005001
#define DAM2_DLC		6

#define BRIGHTNESS_MOB	6
#define BRIGHTNESS_ID	0x00006000
#define BRIGHTNESS_DLC	1
	

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