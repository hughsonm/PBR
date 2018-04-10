/*
 * canbus.c
 *
 * Created: 2014-03-08 23:38:59
 *  Author: Brandon Hill
 */ 

#include "canbus.h"

ISR(CANIT_vect)
{
	canlosstimeout = 0;
	//writeRXMOB(1,0x1000);
	
	
	can_rx_inter();
	//writeRXMOB(0,0x1000);
	CANGIT = 0;
}

void can_rx_inter()
{
	int i;
	unsigned char save_page;		// Pre-interrupt CANPAGE value storage.
	unsigned char isr_flags;		// Temporary interrupt flags value.
	//unsigned int actual_rx_dlc;
	
 	save_page = CANPAGE;			// Save the pre-interrupt CANPAGE value
	CANPAGE = (0<<MOBNB0); //|(0<<MOBNB1)|(0<<MOBNB2)|(0<<MOBNB3)|(0<<AINC)|(0<<INDX2)|(0<<INDX1)|(0<<INDX0);   // select mob0, AINC =0 ENABLED
	
 	isr_flags = CANSTMOB;			// Save the MOb interrupt flags state.
	

	if(isr_flags & (1 << RXOK))   // Check if RXOK flag has been set					
	{
		
	//PORTC |= CANCDMOB;
	//actual_rx_dlc = MOB_DLC_LIMIT(BF_MOB_DLC);  //Fucking genius shit right here, DON'T FUCKING TOUCH THIS, OR I WILL FUCK YOU UP THE ASS!!
		Output(LED2,ENABLE);
		for(i=0;i<8;i++)
		{
			CANECUdata[i] = CANMSG;   // Move data received
		}
		
		CANSTMOB = 0;
		CANCDMOB = (1<<CONMOB1)|(0<<CONMOB0)|(0<<RPLV)|(1<< IDE)|(1<<DLC3)|(0<<DLC2)|(0<<DLC1)|(0<<DLC0);
		CANGIT = 0;
	}
	CANCDMOB = (1<<CONMOB1)|(0<<CONMOB0)|(0<<RPLV)|(1<< IDE)|(1<<DLC3)|(0<<DLC2)|(0<<DLC1)|(0<<DLC0);
	CANPAGE = save_page;  // Restore the pre-interrupt CANPAGE value					
}

inline void canSetup()
{
	cli();
	
	canReset();
		
	for (char mob=0; mob<15; mob++)
	{
		zeroMOB(mob);
	}

	// Baud rate, 1Mb/sec, 16MHz Crystal
	CANBT1 = 0x02;
	CANBT2 = 0x04;
	CANBT3 = 0x13;
	
	
	// Baud rate, 1Mb/sec, 8MHz Crystal
	// CANBT1 = 0x00;
	// CANBT2 = 0x04;						
	// CANBT3 = 0x12;
	
	CANGIE = (1<<ENIT) | (1<<ENRX) ; //(1 << ENTX);
	//Enable CANIT int
	// on RX & TX interrupt
		
	CANIE2 = (1<<IEMOB0);//(1<<IEMOB1);
	CANIE1 = 0;
	//Enable interrupts from mob 0, 1
		
	canEnable();

	writeRXMOB(CAN_PWRCTRL_MOB,CAN_PWRCTRL_ID);
	sei();
}

inline void canEnable()
{
	CANGCON |=  (1<<ENASTB); // Enable CAN. 
}

inline void canDisable()
{
	CANGCON &=  ~(1<<ENASTB); // Disable CAN. 
}

inline void canReset()
{
	CANGCON = (1<<SWRES);				// Software reset request	
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
	
	for (char data=0; data<8; data++)     // Delete data
	{
		CANMSG = 0x00;
	}
}

void writeRXMOB(uint8_t mobnum, uint32_t indentifier)
{
	uint8_t oldPage = CANPAGE;
	CANPAGE = (mobnum<<MOBNB0);    // Select MOb
	
	CANIDT1 = (uint8_t) (indentifier >> 21);		// Set the 29 bit format CAN ID.
	CANIDT2 = (uint8_t) (indentifier >> 13);
	CANIDT3 = (uint8_t) (indentifier >> 5);
	CANIDT4 = (uint8_t) (indentifier << 3)|(1<<RTRTAG)|(0<<RB1TAG)|(1<<RB0TAG);
	//(1<<RTRTAG) For data frame, (0<<RTRTAG) For Remote frame 
	//I think this is opposite!!
	//CHANGED RTRTAG DUNNO IF WILL WORK. 
	
	CANIDM1 = 0xFF;
	CANIDM2 = 0xFF;
	CANIDM3 = 0xFF;
	CANIDM4 = 0b11111001;

	//CANSTMOB !!!!
	CANCDMOB = (1<<CONMOB1)|(0<<CONMOB0)|(1<<IDE) // setup for recive, long ID
				|(1<<DLC3)|(0<<DLC2)|(0<<DLC1)|(0<<DLC0); // expecting 8 bytes
				
	CANPAGE = oldPage;
}

void writeTXMOB(uint8_t mobnum, uint32_t indentifier, unsigned char* CANdata, uint8_t length)
{
	uint8_t oldPage = CANPAGE;
	length = (length>8)?8:length;
	
	//zeroMOB((char)mobnum);
	CANPAGE = (mobnum<<MOBNB0);    // Select MOb
	
	CANIDT1 = (uint8_t) (indentifier >> 21);		// Set the 29 bit format CAN ID.
	CANIDT2 = (uint8_t) (indentifier >> 13);
	CANIDT3 = (uint8_t) (indentifier >> 5);
	CANIDT4 = (uint8_t) (indentifier << 3)|(0<<RTRTAG)|(0<<RB1TAG)|(0<<RB0TAG);
	
	CANIDM1 = 0xFF;
	CANIDM2 = 0xFF;
	CANIDM3 = 0xFF;
	CANIDM4 = 0b11111000|(1<<IDEMSK);
	
	uint8_t data;
	for (data=0; data<length; data++)     // Fill data in
	{
		CANMSG = CANdata[data];
	}
	 // enable for transmission, long ID
	CANCDMOB = (0<<CONMOB1)|(1<<CONMOB0)|(1<<IDE)|(length << DLC0); //8 bytes to be sent
	
	CANPAGE = oldPage;
}