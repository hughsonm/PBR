/*
 * gps.h
 *
 *  Created on: Apr 25, 2018
 *      Author: maxak
 */

#ifndef GPS_H_
#define GPS_H_
#include "main.h"
void UART_GPS_Init(void);


#define	GPS_ID_LENGTH	6
#define N_RMC_COMMAS	12

typedef struct
{
	uint8_t	freshness;
	uint16_t utc_time[4]; 	//{hour,minute,second,milliseconds}
	uint8_t	fix_valid;		// tells us whether the gps module has found its position
	int16_t	latitude[4];	//{degrees,minutes,decimal portion of minutes,direction}
	int16_t	longitude[4];	//{degrees,minutes,decimal portion of minutes,direction}
	int16_t	knots[2];		//{integer portion, fraction portion(tenths)}
	int16_t heading;		//0-360 degree heading
	int16_t	date[3];		//{day,month,year(two-digit)}
}gpsData_t;
// Sample GMRC
//$GPRMC,232218.000,A,4948.4644,N,09708.1211,W,1.63,20.72,240418,,,D*4F
void GPS_ParseRMCString(uint8_t * str, gpsData_t * gps);

#endif /* GPS_H_ */
