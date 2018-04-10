/*
 * ads1220.h
 *
 *  Created on: Aug 11, 2017
 *      Author: maxak
 */

#include "main.h"

#ifndef ADS1220_H_
#define ADS1220_H_

// ADS1220 Register Definitions
#define	ADS_REG_CFG0	0x00
#define	ADS_REG_CFG1	0x01
#define	ADS_REG_CFG2	0x02
#define	ADS_REG_CFG3	0x03

// CFG0 Bit Definitions
#define	ADS_CFG0_MUX	4
#define ADS_CFG0_GAIN	1
#define ADS_CFG0_BYP	0

// CFG1 Bit Definitions
#define ADS_CFG1_DR		6
#define ADS_CFG1_MODE	3
#define ADS_CFG1_CM		2
#define ADS_CFG1_TS		1
#define ADS_CFG1_BCS	0

// CFG2 Bit Definitions
#define ADS_CFG2_VREF	6
#define ADS_CFG2_5060	4
#define ADS_CFG2_PSW	3
#define ADS_CFG2_IDAC	0

// CFG3 Bit Definitions
#define ADS_CFG2_I1MUX	5
#define ADS_CFG2_I2MUX	2
#define ADS_CFG2_DRDYM	1

// ADS1220 Command Definitions
#define	ADS_RST		0b00000110
#define ADS_STSYNC	0b00001000
#define	ADS_PWRDWN	0b00000011
#define	ADS_RDATA	0b00010000
#define	ADS_RREG(nn, addr)	(0b00100000 | (addr<<2) | (nn-1))
#define	ADS_WREG(nn, addr)	(0b01000000 | (addr<<2) | (nn-1))

// Function Declarations
uint8_t SPI_ADS_Init(void);
void ADS_InitHW(void);

uint8_t	SPI_SendReceiveData(SPI_TypeDef* SPIx, uint8_t data);


/*
 *  8.5.3.1 RESET (0000 011x)
Resets the device to the default values. Wait at least (50 µs + 32 · t(CLK)) after the RESET command is sent
before sending any other command.
8.5.3.2 START/SYNC (0000 100x)
In single-shot mode, the START/SYNC command is used to start a single conversion, or (when sent during an
ongoing conversion) to reset the digital filter, and then restarts a single new conversion. When the device is set
to continuous conversion mode, the START/SYNC command must be issued one time to start converting
continuously. Sending the START/SYNC command while converting in continuous conversion mode resets the
digital filter and restarts continuous conversions.
8.5.3.3 POWERDOWN (0000 001x)
The POWERDOWN command places the device into power-down mode. This command shuts down all internal
analog components, opens the low-side switch, turns off both IDACs, but holds all register values. In case the
POWERDOWN command is issued while a conversion is ongoing, the conversion completes before the
ADS1220 enters power-down mode. As soon as a START/SYNC command is issued, all analog components
return to their previous states.
8.5.3.4 RDATA (0001 xxxx)
The RDATA command loads the output shift register with the most recent conversion result. This command can
be used when DOUT/DRDY or DRDY are not monitored to indicate that a new conversion result is available. If a
conversion finishes in the middle of the RDATA command byte, the state of the DRDY pin at the end of the read
operation signals whether the old or the new result is loaded. If the old result is loaded, DRDY stays low,
indicating that the new result is not read out. The new conversion result loads when DRDY is high.
8.5.3.5 RREG (0010 rrnn)
The RREG command reads the number of bytes specified by nn (number of bytes to be read – 1) from the
device configuration register, starting at register address rr. The command is completed after nn + 1 bytes are
clocked out after the RREG command byte. For example, the command to read three bytes (nn = 10) starting at
configuration register 1 (rr = 01) is 0010 0110.
8.5.3.6 WREG (0100 rrnn)
The WREG command writes the number of bytes specified by nn (number of bytes to be written – 1) to the
device configuration register, starting at register address rr. The command is completed after nn + 1 bytes are
clocked in after the WREG command byte. For example, the command to write two bytes (nn = 01) starting at
configuration register 0 (rr = 00) is 0100 0001. The configuration registers are updated on the last SCLK falling
edge.
 */

#endif /* ADS1220_H_ */
