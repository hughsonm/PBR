/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pinmap.h"
#include <stdio.h>
#include <stdlib.h>
// STM32F4 Includes
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"

// Application Level Includes
#include "misc.h"
#include "BNO055.h"
#include "ads1220.h"
#include "adc.h"
#include "counter.h"
#include "gps.h"
#include "sd.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define USE_CAN1
/* Uncomment the line below if you will use the CAN 2 peripheral */
// #define USE_CAN2

#ifdef  USE_CAN1
  #define CANx                       CAN1
  #define CAN_CLK                    RCC_APB1Periph_CAN1
  #define CAN_RX_PIN                 GPIO_Pin_8
  #define CAN_TX_PIN                 GPIO_Pin_9
  #define CAN_GPIO_PORT              GPIOB
  #define CAN_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define CAN_AF_PORT                GPIO_AF_CAN1
  #define CAN_RX_SOURCE              GPIO_PinSource8
  #define CAN_TX_SOURCE              GPIO_PinSource9

#else /* USE_CAN2 */
  #define CANx              CAN2
  #define CAN_CLK           (RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2)
#endif  /* USE_CAN1 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//__IO uint32_t ret = 0; /* for return of the interrupt handling */
extern uint8_t		BNODoneReading;
extern bnoData_t	BNOData;
//extern uint16_t		WheelCount[2];
extern uint32_t		PulsePeriods[2];
extern uint8_t		WheelsAreSpinning;
extern gpsData_t	GPSData;
extern uint8_t 		GPSRxString[100];
extern uint8_t		GPSRxStringReady;
/* Private function prototypes -----------------------------------------------*/
static void NVIC_Config(void);
static void CAN_Config(CanTxMsg* TxMessage,uint32_t CAN_ID, uint8_t CAN_DLC);
static void Delay(void);
void Init_RxMes(CanRxMsg *RxMessage);

#define BLOCK_

void ToggleLED_Timer(void*);
void ToggleLED_Timer2(void*);

// Task Prototypes
void BNO055(void*);
void CAN(void*);
void LOGGING(void*);
void ANALOGIN(void*);
void COUNTERS(void*);
void ADCBYSPI(void*);
void GPS(void*);

void initHW();

xQueueHandle bnoDataQueue;

xQueueHandle irDataQueue;

xQueueHandle wheelDataQueue;

xQueueHandle gpsDataQueue;

int main(void)
{

  RCC_ClocksTypeDef Clocks;
  RCC_GetClocksFreq(&Clocks);

  //Initialize GPIO
  initHW();

  //Initialize BNO055
  volatile uint32_t count;
  count = 0;
  while(count<STARTUP_PAUSE_US){count++;}
//  SPI_ADS_Init();
  //TODO: Initialize Data Logging


  //Watchdog Init
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  // IWDG counter clock
  IWDG_SetPrescaler(IWDG_Prescaler_32);

  IWDG_SetReload(0x3FF);

  // Reload IWDG counter
  IWDG_ReloadCounter();

  // Enable IWDG (the LSI oscillator will be enabled by hardware)
  //IWDG_Enable();

  //Create IPC Variables
  bnoDataQueue = xQueueCreate(10, sizeof(bnoData_t));
  irDataQueue = xQueueCreate(10,sizeof(uint16_t));
  wheelDataQueue = xQueueCreate(10, sizeof(uint32_t));
  gpsDataQueue = xQueueCreate(10,sizeof(gpsData_t));
  //Create tasks

  /*
   * CAN Task:
   * Periodically send out data on the CAN bus.
   */
//  xTaskCreate(
//      CAN,                 				/* Function pointer */
//      "Task1",                          /* Task name - for debugging only*/
//      configMINIMAL_STACK_SIZE*2,         /* Stack depth in words */
//      (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
//      tskIDLE_PRIORITY + 1UL,           /* Task priority*/
//      NULL                              /* Task handle */
//  );
  /*
   * BNO055 Task:
   * Periodically query the accelerometer via I2C.
   */
//  xTaskCreate(
//      BNO055,                 			/* Function pointer */
//      "Task2",                          /* Task name - for debugging only*/
//      configMINIMAL_STACK_SIZE,         /* Stack depth in words */
//      (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
//      tskIDLE_PRIORITY + 1UL,           /* Task priority*/
//      NULL                              /* Task handle */
//  );

  /*
   * LOGGING Task:
   * Periodically take a snapshot of current fDAM data
   * and log to an SD card via SDIO
   */
//  xTaskCreate(
//        LOGGING,						  /* Function pointer */
//        "Task3",                          /* Task name - for debugging only*/
//        1024					,         /* Stack depth in words */
//        (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
//        tskIDLE_PRIORITY + 4UL,           /* Task priority*/
//        NULL                              /* Task handle */
//    );

  /*
   * ANALOGIN Task:
   * Periodically read the analog inputs.
   */
  xTaskCreate(
        ANALOGIN,       		          /* Function pointer */
        "Task4",                          /* Task name - for debugging only*/
        configMINIMAL_STACK_SIZE,         /* Stack depth in words */
        (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
        tskIDLE_PRIORITY + 2UL,           /* Task priority*/
        NULL                              /* Task handle */
    );

  /*
   * ADCBYSPI Task:
   * Periodically use SPI to read the external ADC's
   */
  xTaskCreate(
        ADCBYSPI,       		          /* Function pointer */
        "Task5",                          /* Task name - for debugging only*/
        configMINIMAL_STACK_SIZE,         /* Stack depth in words */
        (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
        tskIDLE_PRIORITY + 2UL,           /* Task priority*/
        NULL                              /* Task handle */
    );
  /*
   * COUNTERS Task:
   * At regular time interval, check the external counters,
   * then reset them to zero. This measures wheel speed.
   */

  xTaskCreate(
      COUNTERS,		                 /* Function pointer */
	  "Task5",                          /* Task name - for debugging only*/
	  configMINIMAL_STACK_SIZE,         /* Stack depth in words */
	  (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
	  tskIDLE_PRIORITY + 4UL,           /* Task priority*/
	  NULL                              /* Task handle */
    );

  xTaskCreate(
  	  GPS,
	  "Task6",
	  configMINIMAL_STACK_SIZE,
	  (void*) NULL,
	  tskIDLE_PRIORITY + 1UL,
	  NULL
  );

  if (bnoDataQueue == 0 || irDataQueue == 0 || wheelDataQueue == 0) {
    while(1);
  }
  // Start the scheduler
  NVIC_Config();
  vTaskStartScheduler();

  while(1);
}

void CAN(void *pvParameters){

  bnoData_t bnoDataRecv;
  uint32_t	wheelDataRecv;
  uint16_t	wheelUnpackedRecv[2];
  uint16_t	irDataRecv;
  uint8_t	irUnpackedRecv[2];
  gpsData_t	gpsDataRecv;
  uint8_t	didSendSomething=0;
  portBASE_TYPE status;
  CanTxMsg LinAccelTx, EulerTx, AccelTx, GyroTx, MiscTx, GPSLatTx,GPSLonTx;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = CAN_TX_PERIOD_MS;

  //Initialize CAN
  //NVIC_Config();
  CAN_Config(&LinAccelTx,	ID_LINACCEL,	8);
  CAN_Config(&EulerTx,		ID_EULER,		6);
  CAN_Config(&AccelTx,		ID_ACCEL,		6);
  CAN_Config(&GyroTx,		ID_GYRO,		6);
  CAN_Config(&MiscTx,		ID_MISC,		8);
  CAN_Config(&GPSLatTx,		ID_GPSLAT,		8);
  CAN_Config(&GPSLonTx,		ID_GPSLON,		8);

  while (1) {
	  didSendSomething = 0;

	  /*
	   * There are only three CAN mailboxes that can get used at once.
	   * I need some way to handle the 'all mailboxes busy' case.
	   * If all mailboxes are full, CAN_Transmit won't send anything,
	   * and it will return a flag saying so.
	   * I could just look at that flag and block until my message finds a mailbox.
	   * I could also have some priority ring that dynamically changes message priority
	   * I think the blocking option is the correct one for this case. Blocking is bad,
	   * but it will make sure that if new data are available to be sent, they WILL be sent.
	   */

      status = xQueueReceive(bnoDataQueue, &bnoDataRecv,0);// portMAX_DELAY);
      if ((BNO_COMP_MODE(BNOData.config,BNO_MODE_NDOF)) && (pdTRUE == status))
      {
          LinAccelTx.Data[0] = ((bnoDataRecv.linear.x)&0xFF00)>>8;
          LinAccelTx.Data[1] = ((bnoDataRecv.linear.x)&0x00FF);
          LinAccelTx.Data[2] = ((bnoDataRecv.linear.y)&0xFF00)>>8;
          LinAccelTx.Data[3] = ((bnoDataRecv.linear.y)&0x00FF);
          LinAccelTx.Data[4] = ((bnoDataRecv.linear.z)&0xFF00)>>8;
          LinAccelTx.Data[5] = ((bnoDataRecv.linear.z)&0x00FF);
          LinAccelTx.Data[6] = bnoDataRecv.config;
          LinAccelTx.Data[7] = bnoDataRecv.temp;
          while(CAN_TxStatus_NoMailBox == CAN_Transmit(CANx, &LinAccelTx));

          EulerTx.Data[0] = ((bnoDataRecv.euler.heading)&0xFF00)>>8;
          EulerTx.Data[1] = ((bnoDataRecv.euler.heading)&0x00FF);
          EulerTx.Data[2] = ((bnoDataRecv.euler.roll)&0xFF00)>>8;
          EulerTx.Data[3] = ((bnoDataRecv.euler.roll)&0x00FF);
          EulerTx.Data[4] = ((bnoDataRecv.euler.pitch)&0xFF00)>>8;
          EulerTx.Data[5] = ((bnoDataRecv.euler.pitch)&0x00FF);
          while(CAN_TxStatus_NoMailBox == CAN_Transmit(CANx, &EulerTx));

          AccelTx.Data[0] = ((bnoDataRecv.accel.x)&0xFF00)>>8;
          AccelTx.Data[1] = ((bnoDataRecv.accel.x)&0x00FF);
          AccelTx.Data[2] = ((bnoDataRecv.accel.y)&0xFF00)>>8;
          AccelTx.Data[3] = ((bnoDataRecv.accel.y)&0x00FF);
          AccelTx.Data[4] = ((bnoDataRecv.accel.z)&0xFF00)>>8;
          AccelTx.Data[5] = ((bnoDataRecv.accel.z)&0x00FF);
          while(CAN_TxStatus_NoMailBox == CAN_Transmit(CANx, &AccelTx));

          //while(CAN_TxStatus_Ok != CAN_TransmitStatus(CANx, 2));

          GyroTx.Data[0] = ((bnoDataRecv.gyro.x)&0xFF00)>>8;
          GyroTx.Data[1] = ((bnoDataRecv.gyro.x)&0x00FF);
          GyroTx.Data[2] = ((bnoDataRecv.gyro.y)&0xFF00)>>8;
          GyroTx.Data[3] = ((bnoDataRecv.gyro.y)&0x00FF);
          GyroTx.Data[4] = ((bnoDataRecv.gyro.z)&0xFF00)>>8;
          GyroTx.Data[5] = ((bnoDataRecv.gyro.z)&0x00FF);
          while(CAN_TxStatus_NoMailBox == CAN_Transmit(CANx, &GyroTx));
          didSendSomething = 1;
      }

      status = xQueueReceive(wheelDataQueue, &wheelDataRecv,0);
      if(pdTRUE == status)
      {
    	  Counter_UnpackFreqs(wheelDataRecv,wheelUnpackedRecv);
          MiscTx.Data[4] = (wheelUnpackedRecv[0] & 0xFF00)>>8;
          MiscTx.Data[5] = (wheelUnpackedRecv[0] & 0x00FF);
          MiscTx.Data[6] = (wheelUnpackedRecv[1] & 0xFF00)>>8;
          MiscTx.Data[7] = (wheelUnpackedRecv[1] & 0x00FF);
          while(CAN_TxStatus_NoMailBox == CAN_Transmit(CANx, &MiscTx));
    	  didSendSomething = 1;
      }

      status = xQueueReceive(irDataQueue, &irDataRecv,0);
      if(pdTRUE == status)
      {
    	  Analog_UnpackReadings(irDataRecv,irUnpackedRecv);
    	  MiscTx.Data[2] = irUnpackedRecv[0];
    	  MiscTx.Data[3] = irUnpackedRecv[1];
    	  while(CAN_TxStatus_NoMailBox == CAN_Transmit(CANx, &MiscTx));
    	  didSendSomething = 1;
      }

      status = xQueueReceive(gpsDataQueue,&gpsDataRecv,0);
      if(pdTRUE == status)
      {
    	  GPSLatTx.Data[0] = (uint8_t)(gpsDataRecv.latitude[0]>>8);
    	  GPSLatTx.Data[1] = (uint8_t)(gpsDataRecv.latitude[0]);
    	  GPSLatTx.Data[2] = (uint8_t)(gpsDataRecv.latitude[1]>>8);
		  GPSLatTx.Data[3] = (uint8_t)(gpsDataRecv.latitude[1]);
		  GPSLatTx.Data[4] = (uint8_t)(gpsDataRecv.latitude[2]>>8);
		  GPSLatTx.Data[5] = (uint8_t)(gpsDataRecv.latitude[2]);
		  GPSLatTx.Data[6] = (uint8_t)(gpsDataRecv.latitude[3]>>8);
		  GPSLatTx.Data[7] = (uint8_t)(gpsDataRecv.latitude[3]);

		  GPSLonTx.Data[0] = (uint8_t)(gpsDataRecv.longitude[0]>>8);
		  GPSLonTx.Data[1] = (uint8_t)(gpsDataRecv.longitude[0]);
		  GPSLonTx.Data[2] = (uint8_t)(gpsDataRecv.longitude[1]>>8);
		  GPSLonTx.Data[3] = (uint8_t)(gpsDataRecv.longitude[1]);
		  GPSLonTx.Data[4] = (uint8_t)(gpsDataRecv.longitude[2]>>8);
		  GPSLonTx.Data[5] = (uint8_t)(gpsDataRecv.longitude[2]);
		  GPSLonTx.Data[6] = (uint8_t)(gpsDataRecv.longitude[3]>>8);
		  GPSLonTx.Data[7] = (uint8_t)(gpsDataRecv.longitude[3]);
		  while(CAN_TxStatus_NoMailBox == CAN_Transmit(CANx, &GPSLatTx));
		  while(CAN_TxStatus_NoMailBox == CAN_Transmit(CANx, &GPSLonTx));
		  didSendSomething = 1;
      }

      if(didSendSomething) GPIO_ToggleBits(PM_LED_CAN);
      vTaskDelayUntil(&xLastWakeTime, xPeriod/portTICK_RATE_MS);
  }
}

void BNO055(void *pvParameters){

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = CAN_TX_PERIOD_MS;
	BNO_Init(Normal,NDOF);
	// Turn the I2C ISR BNO read state machine on
	I2C_ITConfig(I2C1,I2C_IT_EVT|I2C_IT_BUF,ENABLE);
	while (1) {
		// Kickstart the I2C process.
		I2C_GenerateSTART(I2C1,ENABLE);
		/*
		 *  The BNO read process is handled by non-blocking interrupts.
		 *  This means that the operating system can switch tasks without
		 *  worrying about leaving a BNO read operation hanging. The flag
		 *  BNODoneReading will tell this task when fresh data are ready.
		 */
		while(!BNODoneReading);
		BNODoneReading=0;
		if(BNO_COMP_MODE(BNOData.config,BNO_MODE_NDOF) && BNOData.temp != 0 )
		{
			GPIO_ToggleBits(GPIOC, GPIO_Pin_0);
		}
		xQueueSendToBack(bnoDataQueue, &BNOData.config, 0);
		// Kickdog
		// I2C comms with BNO sometimes break down. If that happens, the dog won't get kicked and a reset will occur.
		IWDG_ReloadCounter();
		vTaskDelayUntil(&xLastWakeTime, xPeriod/portTICK_RATE_MS);
	}
}


void ADCBYSPI(void *pvParameters)
{
	// The external ADCs appear to not be too functional right now...
	volatile uint32_t	spi_adc_val=0;
	uint8_t success;

	success = SPI_ADS_Init();


	while(1)
	{
		//success = SPI_ADS_Init();

		if(!(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)))
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_12);
			spi_adc_val =
					(SPI_SendReceiveData(SPI2,0x00)<<16) |
					(SPI_SendReceiveData(SPI2,0x00)<<8)  |
					(SPI_SendReceiveData(SPI2,0x00));
			GPIO_SetBits(GPIOB,GPIO_Pin_12);
		}
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

void LOGGING(void *pvParameters){

	int16_t result;
	char log_buffer[1024] = {4,5,6,7,8,9,10,11,12,13,14,15};

	//Initialize the data logging stuff

	// SD card is hooked up to pins PA3 to PA7(cd,nss,ck,miso,mosi)
	// Set up SPI on those pins. It's SPI1.
	// Gonna need some heavy state machine stuff to control the talking to the sd card.
	SD_InitHW();
	result = SD_InitCard();
	while(1){
		// Get the current time
		if(!result)	result = SD_DiskRead(log_buffer,0,2);
		if(!result) result = SD_DiskWrite(log_buffer,0,1);
		if(!result) result = SD_DiskRead(log_buffer, 0, 2);
		// Log a selection of data to the SD card
		vTaskDelay(10000 / portTICK_RATE_MS);
	}

}

void ANALOGIN(void *pvParameters){

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = ADC_PERIOD_MS;
	uint16_t adc_packed;
	uint8_t	adcs[2];

	Analog_Init();
	Analog_StartConversions();
	while(1){
		// Read the value from ADC1 and put in in the IR queue.
		adcs[0] = ADC_GetConversionValue(ADC1);
		adcs[1] = ADC_GetConversionValue(ADC2);
		adc_packed = Analog_PackReadings(adcs);
		xQueueSendToBack(irDataQueue, &adc_packed,0);
		// Wait around for a little bit.
		vTaskDelayUntil(&xLastWakeTime, xPeriod/portTICK_RATE_MS);
	}

}


void GPS(void * pvParameters){
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	UART_GPS_Init();
	while(1)
	{
		// Has a fresh pack of GPS data arrived?
		// If yes, queue it for CAN transmission

		if(GPSRxStringReady)
		{
			// Send it.
			GPS_ParseRMCString(GPSRxString,&GPSData);
			GPSRxStringReady = 0;
			xQueueSendToBack(gpsDataQueue, &GPSData,0);
		}

		vTaskDelayUntil(&xLastWakeTime, 80/portTICK_RATE_MS);
	}
}

void COUNTERS(void *pvParameters){

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = WS_TX_PERIOD_MS;
	uint16_t	wheel_freq[2];
	uint32_t	pulse_prd_capture;
	uint32_t	wheel_packed;

	Counter_Init();
	while(1)
	{

		//wheel_freq[0] = (((uint16_t)WheelCount[0])*1000)/WS_COUNT_PERIOD_MS;
		//wheel_freq[1] = (((uint16_t)WheelCount[1])*1000)/WS_COUNT_PERIOD_MS;
		if(WheelsAreSpinning)
		{
			pulse_prd_capture = PulsePeriods[0];
			if(pulse_prd_capture) wheel_freq[0] = (uint16_t)(WS_TICK_FREQ/pulse_prd_capture);
			pulse_prd_capture = PulsePeriods[1];
			if(pulse_prd_capture) wheel_freq[1] = (uint16_t)(WS_TICK_FREQ/pulse_prd_capture);
		}
		else
		{
			wheel_freq[0] = 0;
			wheel_freq[1] = 0;
		}
		wheel_packed = Counter_PackFreqs(wheel_freq);
		xQueueSendToBack(wheelDataQueue, &wheel_packed, 0);
		vTaskDelayUntil(&xLastWakeTime, xPeriod/portTICK_RATE_MS);
	}

}

void initHW()
{

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOC,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    //Enable CAN Transceiver
    GPIO_ResetBits(GPIOB,GPIO_Pin_10);

}

static void NVIC_Config(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
/*
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
*/
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;

}

void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t ubCounter = 0;

  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    RxMessage->Data[ubCounter] = 0x00;
  }
}

static void CAN_Config(CanTxMsg* TxMessage, uint32_t CAN_ID, uint8_t CAN_DLC)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CAN_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT);

  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);

  /* CAN configuration ********************************************************/
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

  /* CAN register init */
  CAN_DeInit(CANx);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

  /* CAN Baudrate = 1 MBps (CAN clocked at 30 MHz) */
//  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;//CAN_BS1_4tq;//CAN_BS1_6tq;
//  CAN_InitStructure.CAN_BS2 = CAN_BS1_2tq;//CAN_BS2_3tq;//CAN_BS2_8tq;
//  CAN_InitStructure.CAN_Prescaler = 1;//2;
//  CAN_Init(CANx, &CAN_InitStructure);

  CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;//CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;//CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 2;
  CAN_Init(CANx, &CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  if(CAN_ID == 0x5003)
      CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  /* Transmit Structure preparation */
  CAN_DLC = (CAN_DLC>8)?8:CAN_DLC;
  TxMessage->StdId = 0x321;
  TxMessage->ExtId = CAN_ID;
  TxMessage->RTR = CAN_RTR_DATA;
  TxMessage->IDE = CAN_ID_EXT;
  TxMessage->DLC = CAN_DLC;
  for(uint8_t idx = 0; idx < CAN_DLC; idx++) TxMessage->Data[idx] = 0;

  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CANx, CAN_IT_FMP1, ENABLE);
}

static void Delay(void)
{
  uint16_t nTime = 0x0000;

  for(nTime = 0; nTime <0xFFF; nTime++)
  {
  }
}

