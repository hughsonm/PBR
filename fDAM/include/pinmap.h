#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"


#define PM_ADS_CS1	GPIOB,GPIO_Pin_12
#define PM_ADS_CS2	GPIOC,GPIO_Pin_6
#define	PM_ADS_SCK
#define PM_ADS_MISO
#define PM_ADS_MISO
#define	PM_LED_CAN	GPIOC, GPIO_Pin_1

#define PM_SD_CD	GPIOA,GPIO_Pin_3
#define	PM_SD_CS	GPIOA,GPIO_Pin_4
#define	PM_SD_SCK	GPIOA,GPIO_Pin_5
#define	PM_SD_MISO	GPIOA,GPIO_Pin_6
#define	PM_SD_MOSI	GPIOA,GPIO_Pin_7

