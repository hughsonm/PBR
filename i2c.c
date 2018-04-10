/*
 * i2c.c
 *
 * Created: 2017-10-25 10:13:23 AM
 *  Author: maxak
 */ 

#include "main.h"
#include "i2c.h"

int16_t	ADSReadings[4] = {0,0,0,0};
int32_t	LoadMilliAmps[4] = {0,0,0,0};

ISR(TWI_vect)
{
	static	uint8_t	ads = 0;
	static	uint8_t	channel = 0;
	static	uint8_t	bytes = 0;	
	//static	uint8_t	last_ten[20];
	//static	uint8_t	ten_idx=0;
	uint16_t	data;
	uint8_t		temp;
	// INT flag not automatically cleared!!
	// SCL is stretched as long as INT flag is not written with 1.
	// Writing INT flag with 1 starts the next I2C operation.
	
	// State machine gets kicked by asking for a start command
	// First, send slaw, point to config, set mux to desired channel
	// Then, issue new start command, send slaw, point to conv,
	// Then, issue new start command, send slar, get two conversion bytes.
	// Issue stop command
	// Increment requested channel, set machine back to zero, then kick it.
	//if(ten_idx<20) last_ten[ten_idx++] = TWSR&0b11111000;
	switch (ads)
	{
	case 0: // First, send slaw, point to config, set mux to desired channel	
		switch (TWSR & (0b11111000))
		{
			case I2C_STARTOK:
			case I2C_REPSTARTOK: // Put SLA+W in TWDR			
				temp = TWCR;
				temp |= (1<<TWINT);
				temp &= ~((1<<TWSTO)|(1<<TWSTA)|(1<<TWEA));
				TWDR = ((ADS_SLA<<1)|I2C_RW_WRITE);
				TWCR = temp;
				break;				
			case I2C_SLAWACKOK: // Send contents of pointer register in a data byte (point to config register)
				temp = TWCR;
				temp |= (1<<TWINT);
				temp &= ~((1<<TWSTO)|(1<<TWSTA)|(1<<TWEA));
				bytes = 0;
				TWDR = ADS_REG_CONF;
				TWCR = temp;
				break;
			case I2C_SNDACKOK: // Send 2 bytes to fill the config register.
				// ADC measures (channel - GND). 1600 samples per second. Full scale = 1.024 V (max load current around 9 A).
				data = (1<<ADS_CF_MUX2)|(channel<<ADS_CF_MUX0)|(1<<ADS_CF_DR2)|(1<<ADS_CF_PGA1)|(1<<ADS_CF_PGA0);
				// ADS receive most significant byte first.
				TWDR = (bytes++)?((uint8_t)(data)):((uint8_t)(data>>8));
				temp = TWCR;
				temp |= (1<<TWINT);
				temp &= ~((1<<TWSTO)|(1<<TWSTA)|(1<<TWEA));
				if(bytes > 2)
				{// Done filling config. Ask for start-stop.
					ads = 1;
					bytes = 0;					
					temp |= (1<<TWSTA)|(1<<TWSTO);				
				}
				TWCR = temp;
				break;	
			case I2C_SNDNACKOK: // Something went wrong. Reset.
				ads = 0;
				bytes = 0;
				channel = 0;
				temp = TWCR;
				temp |= (1<<TWSTO)|(1<<TWSTA)|(1<<TWINT);
				temp &= ~((1<<TWEA));	
				TWCR = temp;
				break;	
			default:
				break;
		}
		break;	
	case 1: // Then, send slaw, point to conv,
		switch (TWSR & (0b11111000))
		{
			case I2C_STARTOK:
			case I2C_REPSTARTOK: // Put SLA+W in TWDR				
				TWDR = (ADS_SLA<<1) | I2C_RW_WRITE;
				temp = TWCR;
				temp |= (1<<TWINT);
				temp &= ~((1<<TWSTA)|(1<<TWSTO)|(1<<TWEA));
				TWCR = temp;
				break;			
			case I2C_SLAWACKOK: // Fill pointer register with conversion address			
				TWDR = ADS_REG_CONV;
				temp = TWCR;
				temp |= (1<<TWINT);
				temp &= ~((1<<TWSTA)|(1<<TWSTO)|(1<<TWEA));
				TWCR = temp;
				break;
			case I2C_SNDACKOK: // End this communication. Stop-start to read conversion register.				
				temp = TWCR;
				temp &= ~((1<<TWEA));
				temp |= (1<<TWINT)|(1<<TWSTA)|(1<<TWSTO);
				ads = 2;
				TWCR = temp;				
			default:
				break;
		}
		break;		
	case 2: // Then, send slar, get two conversion bytes.
		switch (TWSR & (0b11111000))
		{
			case I2C_STARTOK:
			case I2C_REPSTARTOK: // Send SLA+R, expect ACK.
				TWDR = (ADS_SLA<<1) | I2C_RW_READ;
				temp = TWCR;
				temp |= (1<<TWINT);
				temp &= ~((1<<TWSTO)|(1<<TWSTA)|(1<<TWEA));
				TWCR = temp;
				break;
			case I2C_SLARACKOK: // Start pulling out bytes from conversion register. Acknowledge receipt.
				temp = TWCR;
				temp |= (1<<TWINT)|(1<<TWEA);
				temp &= ~((1<<TWSTO)|(1<<TWSTA));
				bytes = 0;
				TWCR = temp;				
				break;
			case I2C_RCVACKOK: // Byte has been received. May still need one more.
				if(bytes++)
				{// MSB has already been extracted. TWDR now holds LSB. Result is left-justified, so divide.
					ADSReadings[channel] |= (((int16_t)TWDR));
					ADSReadings[channel] /= 16;				
				}
				else
				{// This is the first conversion byte (most significant byte).
					ADSReadings[channel] = (((int16_t)TWDR)<<8);					
				}				
				temp = TWCR;
				temp |= (1<<TWINT)|(1<<TWEA);
				temp &= ~((1<<TWSTA)|(1<<TWSTO));
				if(bytes >= 2)
				{// Both bytes have been extracted. Reset state machine, change channel, then stop-start.
					temp |= (1<<TWINT)|(1<<TWSTO)|(1<<TWSTA);
					temp &= ~(1<<TWEA);
					if(++channel >= 4) channel = 0;
					ads = 0;
					bytes = 0;
				}
				TWCR = temp;
				break;
			default:
				break;
		}
		break;
	default:
		break;
	}
}

/*
4 Channels run to the ADS1015
0:	FAN_SENSE
1:	FP_SENSE
2:	ACC_SENSE
3:	LAMBDA_SENSE
*/

void i2c_init(void)
{
	/*
	Note: TWBR should be 10 or higher if the TWI operates in Master mode. If TWBR is lower than 10, the
	master may produce an incorrect output on SDA and SCL for the reminder of the byte. The problem
	occurs when operating the TWI in Master mode, sending Start + SLA + R/W to a slave.
	A slave does not need to be connected to the bus for the condition to happen.
	*/
	// f_{SCL} = CLK_{io} / (16 + 2*TWBR*4^(TWPS))
	TWBR = 12;
	TWSR &= ~((1<<TWPS1)|(1<<TWPS0));
	//		   = 16MHz / (16 + 2*12*4^0)
	//         = 16MHz / (16 + 24)
	//		   = 16MHz / 40 = 400 kHz ('Full Speed')
	// Ask for a start command, which kicks off the i2c
	// interrupt-based state machine.
	TWCR |= (1<<TWEN)|(1<<TWIE)|(1<<TWSTA);	
}
