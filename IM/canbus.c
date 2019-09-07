/*
 * canbus.c
 *
 * Created: 2014-03-08 23:38:59
 *  Author: Brandon Hill
 */ 

/*
 * Currently ~1ms between CAN messages from the ECU
 */
#include "IM_2017.h"
#include "canbus.h"
#include "output.h"

s16 DAMData[6];
u8 CANTXData[IM_CTRL_DLC];
u8 SendCANData = 0x00;

carState Car;

indModuleState IndModule;

void init_car(void)
{
	Car.car_data[COOLANT_TEMP].type = SIZE_8;
	Car.car_data[COOLANT_TEMP].arr_idx = 0;
	Car.car_data[COOLANT_TEMP].n_digits = 3;
	Car.car_data[COOLANT_TEMP].decimal = 2;
	Car.car_data[OIL_PRESSURE].type = SIZE_8;
	Car.car_data[OIL_PRESSURE].arr_idx = 1;
	Car.car_data[OIL_PRESSURE].n_digits = 3;
	Car.car_data[OIL_PRESSURE].decimal = 2;
	Car.car_data[FUEL_LEVEL].type = SIZE_8;
	Car.car_data[FUEL_LEVEL].arr_idx = 2;
	Car.car_data[FUEL_LEVEL].n_digits = 3;
	Car.car_data[FUEL_LEVEL].decimal = 2;
	Car.car_data[GEAR_POSITION].type = SIZE_8;
	Car.car_data[GEAR_POSITION].arr_idx = 3;
	Car.car_data[GEAR_POSITION].n_digits = 1;
	Car.car_data[GEAR_POSITION].decimal = 0;
	Car.car_data[ENGINE_SPEED].type = SIZE_16;
	Car.car_data[ENGINE_SPEED].arr_idx = 4;
	Car.car_data[ENGINE_SPEED].n_digits = 5;
	Car.car_data[ENGINE_SPEED].decimal = 1;
	Car.car_data[THROTTLE_POS].type = SIZE_8;
	Car.car_data[THROTTLE_POS].arr_idx = 6;
	Car.car_data[THROTTLE_POS].n_digits = 3;
	Car.car_data[THROTTLE_POS].decimal = 2;
	Car.car_data[FUEL_TEMP].type = SIZE_8;
	Car.car_data[FUEL_TEMP].arr_idx = 7;
	Car.car_data[FUEL_TEMP].n_digits = 3;
	Car.car_data[FUEL_TEMP].decimal = 2;
	Car.car_data[LAMBDA].type = SIZE_8;
	Car.car_data[LAMBDA].arr_idx = 8;
	Car.car_data[LAMBDA].n_digits = 3;
	Car.car_data[LAMBDA].decimal = 0;
	Car.car_data[VEHICLE_SPEED].type = SIZE_8;
	Car.car_data[VEHICLE_SPEED].arr_idx = 9;
	Car.car_data[VEHICLE_SPEED].n_digits = 3;
	Car.car_data[VEHICLE_SPEED].decimal = 2;
	Car.car_data[BATT_VOLTAGE].type = SIZE_8;
	Car.car_data[BATT_VOLTAGE].arr_idx = 10;
	Car.car_data[BATT_VOLTAGE].n_digits = 4;
	Car.car_data[BATT_VOLTAGE].decimal = 2;
	Car.car_data[BATT_TEMP].type = SIZE_8;
	Car.car_data[BATT_TEMP].arr_idx = 11;
	Car.car_data[BATT_TEMP].n_digits = 3;
	Car.car_data[BATT_TEMP].decimal = 2;
	Car.car_data[PRIMARY_CURRENT].type = SIZE_8;
	Car.car_data[PRIMARY_CURRENT].arr_idx = 12;
	Car.car_data[PRIMARY_CURRENT].n_digits = 3;
	Car.car_data[PRIMARY_CURRENT].decimal = 1;
	Car.car_data[SECONDARY_CURRENT].type = SIZE_8;
	Car.car_data[SECONDARY_CURRENT].arr_idx = 13;
	Car.car_data[SECONDARY_CURRENT].n_digits = 3;
	Car.car_data[SECONDARY_CURRENT].decimal = 1;
	Car.car_data[ACCEL_LAT].type = SIZE_16|SIGNED;
	Car.car_data[ACCEL_LAT].arr_idx = 14;
	Car.car_data[ACCEL_LAT].n_digits = 4;
	Car.car_data[ACCEL_LAT].decimal = 0;
	Car.car_data[ACCEL_FB].type = SIZE_16|SIGNED;
	Car.car_data[ACCEL_FB].arr_idx = 16;
	Car.car_data[ACCEL_FB].n_digits = 4;
	Car.car_data[ACCEL_FB].decimal = 0;
	Car.car_data[ACCEL_UD].type = SIZE_16|SIGNED;
	Car.car_data[ACCEL_UD].arr_idx = 18;
	Car.car_data[ACCEL_UD].n_digits = 4;
	Car.car_data[ACCEL_UD].decimal = 0;
	Car.car_data[EULER_HD].type = SIZE_16|SIGNED;
	Car.car_data[EULER_HD].arr_idx = 20;
	Car.car_data[EULER_HD].n_digits = 4;
	Car.car_data[EULER_HD].decimal = 1;
	Car.car_data[EULER_RL].type = SIZE_16|SIGNED;
	Car.car_data[EULER_RL].arr_idx = 22;
	Car.car_data[EULER_RL].n_digits = 4;
	Car.car_data[EULER_RL].decimal = 1;
	Car.car_data[EULER_PT].type = SIZE_16|SIGNED;
	Car.car_data[EULER_PT].arr_idx = 24;
	Car.car_data[EULER_PT].n_digits = 4;
	Car.car_data[EULER_PT].decimal = 1;
};


ISR(CANIT_vect)
{
	u16 int_flags;
	u8	mob_check, old_page = CANPAGE;
	
	//PINA = 0b00000001;//Toggle a debug LED
	//canlosstimeout = 0;
	//writeRXMOB(1,0x1000);
	//writeTXMOB(0,0x2000,SENDdata);
	
	// Find out which MOb(s) triggered the interrupt,
	// and service those.
	int_flags = CANSIT1;
	int_flags = int_flags<<8;
	int_flags |= CANSIT2;
	for(mob_check = 0; mob_check < 15; mob_check++)
	{
		if(int_flags & (1<<mob_check))
		{
			// Interrupt was requested for this MOb
			CANPAGE = (mob_check<<MOBNB0);
			// Determine whether it's a RX interrupt
			if (CANSTMOB & (1<<RXOK))
			{
				can_rx_inter(mob_check);
			}
			// Clear this MOB's interrupt flags
			CANSTMOB = 0;
		}
	}
	CANPAGE = old_page;
	CANGIT = 0;
}

void can_rx_inter(u8 mobnum)
{
	u8 ii;
	u8 data_length = (CANCDMOB & DLC_BITS);	
	u8 offset = 0;				
	switch(mobnum)
	{	
		case VEH_DATA1_MOB:
			for (ii = 0; ii < data_length; ii++)
			{
				Car.val_arr[ii] = CANMSG;
			}
			writeRXMOB(VEH_DATA1_MOB, VEH_DATA1_ID, VEH_DATA1_DLC);
			break;
		case VEH_DATA2_MOB:
			offset = VEH_DATA1_DLC;
			for (ii = 0; ii < data_length; ii++)
			{
				Car.val_arr[ii+offset] = CANMSG;	
			}
			writeRXMOB(VEH_DATA2_MOB, VEH_DATA2_ID, VEH_DATA2_DLC);
			break;
		case POWER_DATA_MOB:
			offset = VEH_DATA1_DLC + VEH_DATA2_DLC;
			for (ii = 0; ii < data_length; ii++)
			{
				Car.val_arr[ii + offset] = CANMSG;
			}
			writeRXMOB(POWER_DATA_MOB, POWER_DATA_ID, POWER_DATA_DLC);
			break;
		case DAM1_MOB:
			offset = VEH_DATA1_DLC + VEH_DATA2_DLC + POWER_DATA_DLC;
			for(ii = 0; ii < data_length; ii++)
			{
				Car.val_arr[ii + offset] = CANMSG;
			}
			writeRXMOB(DAM1_MOB, DAM1_ID, DAM1_DLC);
			break;
		case DAM2_MOB:
			offset = VEH_DATA1_DLC + VEH_DATA2_DLC + POWER_DATA_DLC + DAM1_DLC;
			for(ii = 0; ii < data_length; ii++)
			{
				Car.val_arr[ii + offset] = CANMSG;
			}
			writeRXMOB(DAM2_MOB, DAM2_ID, DAM2_DLC);
			break;
		default:
			break;
	}
}

void activate_transceiver(void)
{
	DDRB |= (1<<DDB4);
	PORTB &= ~(1<<PB4);
}

void can_setup()
{
	cli();
	init_car();
	u8 t_quantum = 2;	// Time quantum is 2 clock cycles. Minimum value is 2
	// TQ = 1/(16Mhz / 2) = .125 us -> 8 TQ per us
	u8 sync_jump = 1;	// Synchronization jump width is 1 time quantum
	u8 prop_time = 3;	// Propagation time segment is 3 time quanta
	//u8 phase_seg_1 = 2;	// Phase segment 1 is 2 time quanta
	//u8 phase_seg_2 = 2;	// Phase segment 2 is 2 time quanta
	/*
	Bit time is:
	sync segment		1TQ (fixed)
	propagation time	3TQ
	phase segment 1		2TQ
	phase segment 2		2TQ	
	-----------------------
	total				8TQ = 1us	
	*/
	u8 can_prs = 250;	
	/*
	CAN timer ticks every 2 ms, rolls over every 131 seconds.
	When a message comes in, it is stamped with a 16-bit time.
	*/
	 
	
	canReset();
	CANTCON = (can_prs - 1);
	u8 mob;
	for (mob=0; mob<15; mob++)
	{
		zeroMOB(mob);
	}
	// Set baud rate prescaler
	CANBT1 = (t_quantum - 1)<<BRP0;	
	// Set the Jump width and propagation time
	CANBT2 = ((sync_jump - 1)<<SJW0) | ((prop_time - 1)<<PRS0);
	// Set the length of the phase segments	
	CANBT3 = 0x13;//((phase_seg_1 - 1)<<PHS10) | ((phase_seg_2)<<PHS20);
	// Turn on majority sampling
	//CANBT3 |= (0<<SMP);

	CANGIE = (1<<ENIT) | (1<<ENRX);
	// ENIT enables CAN interrupts whose enable bits are set.
	// Allow RX interrupts only. 		
	CANIE2 = (1<<IEMOB0)|(1<<IEMOB1)|(1<<IEMOB2)|(1<<IEMOB4)|(1<<IEMOB5);
	CANIE1 = 0;
	//Enable interrupts from mob 0, 1, 2, 4, 5
		
	canEnable();
	writeRXMOB(VEH_DATA1_MOB, VEH_DATA1_ID, VEH_DATA1_DLC);
	writeRXMOB(VEH_DATA2_MOB, VEH_DATA2_ID, VEH_DATA2_DLC);
	writeRXMOB(POWER_DATA_MOB, POWER_DATA_ID, POWER_DATA_DLC);
	writeRXMOB(DAM1_MOB, DAM1_ID, DAM1_DLC);
	writeRXMOB(DAM2_MOB, DAM2_ID, DAM2_DLC);
	activate_transceiver();
	sei();
}

inline void canEnable()
{
	DDRD |= (1<<DD5);	//TXCAN output
	DDRD &= ~(1<<DD6);	//RXCAN input
	CANGCON |=  (1<<ENASTB); // Enable CAN.
	while(!(CANGSTA & (1<<ENFG))) {/*Wait for CAN controller to activate*/} 
}

inline void canDisable()
{
	CANGCON &=  ~(1<<ENASTB); // Disable CAN. 
}

inline void canReset()
{
	CANGCON = (1<<SWRES);	// Software reset request	
}

void zeroMOB(char mobnum)
{
	CANPAGE = (mobnum<<MOBNB0);    // Select MOb

	CANSTMOB = 0;		//Don't fucking touch this!!
	CANCDMOB = 0;
	
	CANIDT4 = 0x00;         // Delete Id.
	CANIDT3 = 0x00;
	CANIDT2 = 0x00;
	CANIDT1 = 0x00;
	
	CANIDM4 = 0x00;         // Delete mask.
	CANIDM3 = 0x00;
	CANIDM2 = 0x00;
	CANIDM1 = 0x00;
	
	u8 data;
	
	for (data=0; data<8; data++)     // Delete data
	{
		CANMSG = 0x00;
	}
}

void writeRXMOB(u8 mobnum, u32 indentifier, u8 dlc)
{
	u8 old_page = CANPAGE;
	CANPAGE = (mobnum<<MOBNB0);    // Select MOb
	
	CANIDT1 = (u8) (indentifier >> 21);		// Set the 29 bit format CAN ID.
	CANIDT2 = (u8) (indentifier >> 13);
	CANIDT3 = (u8) (indentifier >> 5);
	CANIDT4 = (u8) (indentifier << 3)|(0<<RTRTAG)|(0<<RB1TAG)|(0<<RB0TAG);	
	
	/*
	CAN ID mask registers can be used to allow one MOb to respond to several
	different identifiers. The CAN controller will only compare the bits
	specified in the ID mask registers to the incoming CAN identifier to
	determine whether there was a hit or miss. For example, clearing all the
	mask bits would make this MOb respond to every CAN identifier.
	*/
	CANIDM1 = 0xFF;
	CANIDM2 = 0xFF;
	CANIDM3 = 0xFF;
	// RTRTAG comparison is always true
	// Actually check to make sure the IDE bit is correct
	CANIDM4 = 0b11111000|(0<<RTRMSK)|(0<<IDEMSK);

	//CANSTMOB !!!!
	// setup for receive, long ID
	// expecting 8 bytes
	CANCDMOB = (MOB_RX_EN<<CONMOB0)|(1<<IDE)|(dlc<<DLC0);
	CANPAGE = old_page;
	//debug_toggle(1);
}

void writeTXMOB(u8 mobnum, u32 indentifier, u8 *data_ptr, u8 data_len)
{
	/*
	calling this function sends only one CAN frame.
	*/
	u8 ii, old_page;
	old_page = CANPAGE;
	CANPAGE = (mobnum<<MOBNB0);    // Select MOb
	
	CANIDT1 = (u8)(indentifier >> 21);		// Set the 29 bit format CAN ID.
	CANIDT2 = (u8)(indentifier >> 13);
	CANIDT3 = (u8)(indentifier >> 5);
	CANIDT4 = (u8)(indentifier << 3)|(0<<RTRTAG)|(0<<RB1TAG)|(0<<RB0TAG);
	
	CANIDM1 = 0xFF;
	CANIDM2 = 0xFF;
	CANIDM3 = 0xFF;
	CANIDM4 = 0b11111000;
	
	if (data_len > 8)
	{
		data_len = 8;
	}
	
	for (ii = 0; ii < data_len; ii++)     // Fill data in
	{
		CANMSG = data_ptr[ii];
	}
	
	// enable for transmission, long ID
	CANCDMOB = (MOB_TX_EN<<CONMOB0)|(1<<IDE)|(data_len<<DLC0);
	CANPAGE = old_page;
	//debug_toggle(2);
}