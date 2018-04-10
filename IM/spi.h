/*
 * spi.h
 *
 * Created: 2014-03-08 23:42:35
 *  Author: Brandon Hill
 */ 

#ifndef SPI_H_
#define SPI_H_

#include "types.h"

void spi_init(void);

void wait_spi(void);

u8 spi_byte(u8 data);

#endif /* SPI_H_ */