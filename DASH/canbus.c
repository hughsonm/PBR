/*
 * canbus.c
 *
 * Created: 2014-03-08 23:38:59
 *  Author: Brandon Hill
 */ 

/*
 * Currently ~1ms between CAN messages from the ECU
 */

#include "canbus.h"
#include "output.h"

u8 VehicleData[LEN_VEH_DATA];
u8 CANTXData[ECU2DASH_DLC] = {SIGNAL_OFF}; // Launch control starts as off
u8 LaunchRXData[DASH2ECU_DLC] = {SIGNAL_OFF};
u8 SendCANData = 0x00;
extern dashState Dashboard;

ISR(CANIT_vect)
{
	u16 int_flags;
	u8	mob_check, old_page = CANPAGE;
	//canlosstimeout = 0;
	//writeRXMOB(1,0x1000);
	//writeTXMOB(0,0x2000,SENDdata);
	
	// Find out which MOb(s) triggered the interrupt,
	// and service those.
	// (Toggle CAN blinker) PINA = DBG_CAN_LED;
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
	u8 delta_bright;					
	switch(mobnum)
	{
		case VEH_DATA1_MOB:
			for (ii = 0; ii < data_length; ii++)
			{
				VehicleData[ii] = CANMSG;
			}
			writeRXMOB(VEH_DATA1_MOB, VEH_DATA1_ID, VEH_DATA1_DLC);
			break;
		case VEH_DATA2_MOB:
			for (ii = 0; ii < data_length; ii++)
			{
				VehicleData[ii+LAMBDA] = CANMSG;	
			}
			writeRXMOB(VEH_DATA2_MOB, VEH_DATA2_ID, VEH_DATA2_DLC);
			break;
		case POWER_DATA_MOB:
			for (ii = 0; ii < data_length; ii++)
			{
				VehicleData[ii+BATT_VOLTAGE] = CANMSG;
			}
			writeRXMOB(POWER_DATA_MOB, POWER_DATA_ID, POWER_DATA_DLC);
			break;
		case BRIGHTNESS_MOB:
			delta_bright = CANMSG;
			if(delta_bright)
			{				
				if(++Dashboard.brightness > MAX_BRIGHTNESS)
				{
					Dashboard.brightness = MAX_BRIGHTNESS;
				}
			}
			else if(Dashboard.brightness)
			{
				Dashboard.brightness--;
			}			
			writeRXMOB(BRIGHTNESS_MOB, BRIGHTNESS_ID, BRIGHTNESS_DLC);			
			break;				
		case DAM1_MOB:
			for(ii = 0; ii < data_length; ii++)
			{
				VehicleData[ii + ACCEL_LAT] = CANMSG;
			}
			writeRXMOB(DAM1_MOB, DAM1_ID, DAM1_DLC);
			break;
		case ECU2DASH_MOB:
			// First byte is the requested state. I don't really care about that.
			LaunchRXData[LAUNCH_CONTROL_ACT] = CANMSG;
			if(LaunchRXData[LAUNCH_CONTROL_ACT])
			{
				PORTA &= ~(LAUNCH_LED_PIN);
			}else
			{
				PORTA |= (LAUNCH_LED_PIN);
			}
			// Second byte is the actual launch state, from the ECU.
			writeRXMOB(ECU2DASH_MOB, ECU2DASH_ID, ECU2DASH_DLC);
		default:
			break;
	}				
}

void activate_transceiver(void)
{
	DDRD |= (1<<DDD4);
	PORTD &= ~(1<<PD4);
}

void set_maps(void)
{
	u8 maps = PINE;
	maps = maps>>4;
	maps ^= 0b00001111;
	maps &= 0b00001111;
	if(maps == 0b00000001)
	{
		maps = MAPS3;	
	} else if(maps == 0b00000010)
	{
		maps = MAPS2;		
	} else if(maps == 0b00000100)
	{
		maps = MAPS1;
	} else if(maps == 0b00001000)
	{
		maps = MAPS0;
	}
	//CANTXData[MAPS_CONTROL] = maps;
}

void can_setup()
{
	cli();
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
	// Set baud rate pre-scaler
	CANBT1 = (t_quantum - 1)<<BRP0;	
	// Set the Jump width and propagation time
	CANBT2 = ((sync_jump - 1)<<SJW0) | ((prop_time - 1)<<PRS0);
	// Set the length of the phase segments	
	//CANBT3 = ((phase_seg_1 - 1)<<PHS10) | ((phase_seg_2)<<PHS20);
	// Turn on majority sampling
	//CANBT3 |= (1<<SMP);
	CANBT3 = 0x13;

	CANGIE = (1<<ENIT) | (1<<ENRX);
	// ENIT enables CAN interrupts whose enable bits are set.
	// Allow RX interrupts only. 		
	CANIE2 = (1<<IEMOB0)|(1<<IEMOB1)|(1<<IEMOB2)|(1<<IEMOB4)|(1<<IEMOB5);
	CANIE1 = (1<<IEMOB6);
	//Enable interrupts from mob 0, 1, 2, 4, 6
		
	canEnable();
	writeRXMOB(VEH_DATA1_MOB, VEH_DATA1_ID, VEH_DATA1_DLC);
	writeRXMOB(VEH_DATA2_MOB, VEH_DATA2_ID, VEH_DATA2_DLC);
	writeRXMOB(POWER_DATA_MOB, POWER_DATA_ID, POWER_DATA_DLC);
	writeRXMOB(BRIGHTNESS_MOB, BRIGHTNESS_ID, BRIGHTNESS_DLC);
	writeRXMOB(DAM1_MOB, DAM1_ID, DAM1_DLC);
	writeRXMOB(ECU2DASH_MOB,ECU2DASH_ID,ECU2DASH_DLC);
	
	activate_transceiver();
	DDRA |= DBG_CAN_LED; // Set the debug LEDs as outputs
	//set_maps();
	sei();
}

inline void canEnable()
{
	CANGCON |=  (1<<ENASTB); // Enable CAN.
	while(!(CANGSTA & (1<<ENFG))) {/*Wait for CAN controller to activate*/} 
	DDRD |= (1<<DD5);
	DDRD &= ~(1<<DD6);
	//D5, D6 Tx, Rx
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
	CANIDM4 = 0b11111000|(0<<RTRMSK)|(1<<IDEMSK);

	//CANSTMOB !!!!
	// setup for receive, long ID
	dlc = (dlc > 8)?8:dlc;
	CANCDMOB = (MOB_RX_EN<<CONMOB0)|(1<<IDE)|(dlc<<DLC0);
	CANPAGE = old_page;
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
	CANCDMOB = (MOB_TX_EN<<CONMOB0)|(1<<IDE)|(data_len<<DLC0); //8 bytes to be sent
	CANPAGE = old_page;
}