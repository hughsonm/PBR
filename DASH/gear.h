/*
 * gear.h
 *
 * Created: 2014-03-08 23:41:49
 *  Author: Brandon Hill
 */ 

#ifndef GEAR_H_
#define GEAR_H_

#include "atmega.h"
#include "types.h"
#define SEGA	0x0001
#define SEGB	0x0002
#define SEGC	0x0004
#define SEGD	0x0008
#define SEGE	0x0010
#define SEGF	0x0020
#define SEGG	0x0040
#define SEGH	0x0080
#define SEGK	0x0100
#define SEGM	0x0200
#define SEGN	0x0400
#define SEGP	0x0800
#define SEGU	0x8000
#define SEGR	0x2000
#define SEGS	0x1000
#define SEGT	0x4000


void gear_on(void);
void gear_off(void);
void gear_init(void);
void disp_gear(u8 gear_num);
void blockingGearSPIWrite(char gearStatus);


void gearSSOn();
void gearSSOff();

#endif /* GEAR_H_ */