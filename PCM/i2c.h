/*
 * i2c.h
 *
 * Created: 2017-10-25 10:13:35 AM
 *  Author: maxak
 */ 


#ifndef I2C_H_
#define I2C_H_

#define	I2C_RW_READ		0x01
#define I2C_RW_WRITE	0x00

// Status Codes
#define I2C_STARTOK		0x08
#define I2C_REPSTARTOK	0x10
#define I2C_SLAWACKOK	0x18
#define I2C_SLAWNACKOK	0x20
#define I2C_SNDACKOK	0x28
#define I2C_SNDNACKOK	0x30
#define I2C_ARBLOSS		0x38
#define I2C_SLARACKOK	0x40
#define I2C_SLARNACKOK	0x48
#define I2C_RCVACKOK	0x50
#define I2C_RCVNACKOK	0x58

// ADS Register Names
#define ADS_REG_CONV	0x00
#define ADS_REG_CONF	0x01
#define ADS_REG_LO		0x02
#define ADS_REG_HI		0x03

// Register Bit Definitions
// Conversion Register
#define	ADS_CF_OS	15
#define	ADS_CF_MUX2	14
#define	ADS_CF_MUX1	13
#define	ADS_CF_MUX0	12
#define	ADS_CF_PGA2	11
#define	ADS_CF_PGA1	10
#define	ADS_CF_PGA0	9
#define	ADS_CF_MODE	8
#define	ADS_CF_DR2	7
#define	ADS_CF_DR1	6
#define	ADS_CF_DR0	5
#define	ADS_CF_CMODE	4
#define	ADS_CF_CPOL	3
#define	ADS_CF_CLAT	2
#define	ADS_CF_CQ1	1
#define	ADS_CF_CQ0	0

#define ADS_SLA	0b01001000

#define ADS_N_BITS	12

void i2c_init(void);


#endif /* I2C_H_ */