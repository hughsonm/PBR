#include "main.h"
#include "misc.h"
#include "gps.h"
#include "stm32f4xx_usart.h"
#include <string.h>

gpsData_t	GPSData;
uint8_t	GPSRxString[100];
uint8_t	GPSRxStringReady = 0;

volatile uint32_t rx_count = 0;



void UART4_IRQHandler(void)
{
	static uint16_t money_count = 0;
	static uint8_t	string_pos = 0,ignore_msg=1;
	static uint8_t	mrc_string[7] = "$GPRMC";
	//static uint8_t  rx_string[100];
	static uint8_t	comma_count = 0;
	volatile uint16_t status;

	if(USART_GetFlagStatus(UART4,USART_FLAG_RXNE))
	{
		uint8_t rx_byte = (uint8_t)(0x00FF & USART_ReceiveData(UART4));
		if(!GPSRxStringReady)
		{
			if(rx_byte == '$')
			{
				ignore_msg = 0;
				string_pos = 0;
				comma_count = 0;
			}
			if(!ignore_msg)
			{
				if(string_pos <= (GPS_ID_LENGTH-1))
				{
					GPSRxString[string_pos++] = rx_byte;
				}
				else
				{
					if(string_pos == GPS_ID_LENGTH)
					{
						money_count++;
						GPSRxString[string_pos] = '\0';
						if((strcmp(mrc_string,GPSRxString)))
						{
							ignore_msg = 1;
						}

					}
					GPSRxString[string_pos++] = rx_byte;
					if(rx_byte==',') comma_count++;
					if(comma_count >= N_RMC_COMMAS)
					{
						GPSRxString[string_pos++] = '\0';
						GPSRxStringReady = 1;
						ignore_msg = 1;
						//GPS_ParseRMCString(GPSRxString,string_pos,&GPSData);
					}
				}
			}
		}
		// Data occupy bits 0-7. Parity bit is in bit 8.
		// I don't care about the parity bit. Get rid of it.

	}
	if(USART_GetFlagStatus(UART4,USART_FLAG_ORE))
	{
		/* This bit is set by hardware when the word currently
		 * being received in the shift register is ready to be
		 * transferred into the RDR register while RXNE=1. An
		 * interrupt is generated if RXNEIE=1 in the USART_CR1
		 * register. It is cleared by a software sequence
		 * (an read to the USART_SR register followed by a read
		 * to the USART_DR register).
*/
		status = UART4->DR;
	}
	USART_ClearFlag(UART4,USART_FLAG_RXNE);
}


void GPS_WritePMTKString(char * str, uint8_t length)
{
	uint8_t pmtk_idx = 0;
	for(pmtk_idx = 0;pmtk_idx < length;pmtk_idx++)
	{
		while(!USART_GetFlagStatus(UART4,USART_FLAG_TXE));
		USART_SendData(UART4,str[pmtk_idx]);
	}
}

void GPS_ParseRMCString(uint8_t * str, gpsData_t * gps)
{
	uint8_t datum_count = 0;
	uint8_t idx = 0;
	uint8_t	last_comma_idx = 0,last_period_idx = 0,curr_comma_idx = 0;
	int16_t power_of_ten = 1;
	rx_count++;
	gps->freshness = 1;
	while(str[idx] != '\0')
	{
		if(str[idx] == '.') last_period_idx = idx;
		if(str[idx] == ',')
		{
			curr_comma_idx = idx;
			if((curr_comma_idx-last_comma_idx)>1)
			{
				switch(datum_count)
				{
					case 0:
						break;
					case 1:
						gps->utc_time[0] = ((str[last_comma_idx+1]-'0')*10+(str[last_comma_idx+2]-'0'));
						gps->utc_time[1] = ((str[last_comma_idx+3]-'0')*10+(str[last_comma_idx+4]-'0'));
						gps->utc_time[2] = ((str[last_comma_idx+5]-'0')*10+(str[last_comma_idx+6]-'0'));
						gps->utc_time[3] = ((str[last_comma_idx+8]-'0')*100+(str[last_comma_idx+9]-'0')*10+(str[last_comma_idx+10]-'0'));
						break;
					case 2:
						gps->fix_valid = (str[last_comma_idx+1]=='A');
						break;
					case 3:
						gps->latitude[0] = ((str[last_comma_idx+1]-'0')*10+(str[last_comma_idx+2]-'0'));
						gps->latitude[1] = ((str[last_comma_idx+3]-'0')*10+(str[last_comma_idx+4]-'0'));
						gps->latitude[2] = ((str[last_comma_idx+6]-'0')*1000 + (str[last_comma_idx+7]-'0')*100 + (str[last_comma_idx+8]-'0')*10 + (str[last_comma_idx+9]-'0')*1);
						break;
					case 4:
						gps->latitude[3] = (str[last_comma_idx+1] == 'N')?1:-1;
						break;
					case 5:
						gps->longitude[0] = ((str[last_comma_idx+1]-'0')*100+(str[last_comma_idx+2]-'0')*10 + (str[last_comma_idx+3]-'0')*1);
						gps->longitude[1] = ((str[last_comma_idx+4]-'0')*10+(str[last_comma_idx+5]-'0')*1);
						gps->longitude[2] = ((str[last_comma_idx+7]-'0')*1000 + (str[last_comma_idx+8]-'0')*100 + (str[last_comma_idx+9]-'0')*10 + (str[last_comma_idx+10]-'0')*1);
						break;
					case 6:
						gps->longitude[3] = (str[last_comma_idx+1]=='E')?1:-1;
						break;
					case 7:
						power_of_ten = 1;
						gps->knots[0] = 0;
						for (uint8_t dig_idx = last_period_idx-1;dig_idx>last_comma_idx;dig_idx--)
						{
							gps->knots[0] += (str[dig_idx]-'0')*power_of_ten;
							power_of_ten *= 10;
						}
						gps->knots[1] = (str[last_period_idx+1]-'0');
						break;
					case 8:
						power_of_ten = 1;
						gps->heading = 0;
						for (uint8_t dig_idx = last_period_idx-1;dig_idx>last_comma_idx;dig_idx--)
						{
							gps->heading += (str[dig_idx]-'0')*power_of_ten;
							power_of_ten *= 10;
						}
						break;
					case 9:
						gps->date[0] = ((str[last_comma_idx+1]-'0')*10 + (str[last_comma_idx+2]-'0')*1);
						gps->date[1] = ((str[last_comma_idx+3]-'0')*10 + (str[last_comma_idx+4]-'0')*1);
						gps->date[2] = ((str[last_comma_idx+5]-'0')*10 + (str[last_comma_idx+6]-'0')*1);
						break;
					case 10:
						break;
					case 11:
						break;
				}
			}
		datum_count++;
		last_comma_idx = curr_comma_idx;
		}//$GPRMC,232218.000,A,4948.4644,N,09708.1211,W,1.63,20.72,240418,,,D*4F
		idx++;
	}
}
void UART_GPS_Init(void)
{
	/*
	 * Pins to care about:
	 * MCU2GPS on PC10 - USART3Tx/UART4Tx
	 * GPS2MCU on PC11 - USART3Rx/UART4Rx
	 * GPSPPS  on PC9  - 1Hz pulse coming from the GPS module
	 * GPSEN   on PC12 - Active high enable for gps. Pulled high on gps module
	 * GPSFIX  on Pc13 - Blinks at 1 Hz while searching for satellites, then once every 15s once fix is achieved.
	 * USART configuration:
	 * 9600 baud, no parity, 1 stop bit
	 * Set up a rx interrupt.
	 * Let's use UART4. The gps chip uses UART, not USART.
	 */

	// Set pin directions and enable hardware
	USART_InitTypeDef UART_InitStruct;
	GPIO_InitTypeDef UART_GPIO_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	UART_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	UART_GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	UART_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	UART_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	UART_GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOC, &UART_GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	UART_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &UART_GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);


	UART_InitStruct.USART_BaudRate = 9600;
	UART_InitStruct.USART_WordLength = USART_WordLength_8b;
	UART_InitStruct.USART_StopBits = USART_StopBits_1;
	UART_InitStruct.USART_Parity = USART_Parity_No;
	UART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	UART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(UART4, &UART_InitStruct);
	USART_Cmd(UART4,ENABLE);

	char pmtk_string[60];
	int str_len = sprintf(pmtk_string,"$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
	// Tell the GPS chip to only spit out RMC data
	GPS_WritePMTKString(pmtk_string,str_len);

	// Tell the GPS chip to update every 100 ms (10 Hz).
	// Default is 1 Hz update. Reducing output to just rmc data is necessary
	// in order to get 10 Hz updates. Otherwise, gps chip will reject
	// frequency change.
	str_len = sprintf(pmtk_string,"$PMTK220,100*2F\r\n");
	GPS_WritePMTKString(pmtk_string,str_len);

	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);

	NVIC_InitTypeDef  NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_Init(&NVIC_InitStructure);

	// Send $PMTK220,100*2F<CR><LF>


}
