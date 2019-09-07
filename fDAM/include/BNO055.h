#include "stm32f4xx.h"
#include <stdlib.h>


#define I2C_Speed 50000 //50kHz
#define I2C_BNO_ADDRESS 0x28 //COM3 = LOW -> 0x28

//BNO Page 0 Register definitions
#define PWR_MODE          	0x3E
#define OPR_MODE          	0x3D
#define TEMP              	0x34
#define QUA_Data_z_MSB    	0x27
#define QUA_Data_z_LSB    	0x26
#define QUA_Data_y_MSB    	0x25
#define QUA_Data_y_LSB    	0x24
#define QUA_Data_x_MSB    	0x23
#define QUA_Data_x_LSB    	0x22
#define QUA_Data_w_MSB    	0x21
#define QUA_Data_w_LSB    	0x20
#define EUL_Pitch_MSB     	0x1F
#define EUL_Pitch_LSB     	0x1E
#define EUL_Roll_MSB      	0x1D
#define EUL_Roll_LSB      	0x1C
#define EUL_Heading_MSB   	0x1B
#define EUL_Heading_LSB   	0x1A
#define SYS_TRIGGER		  	0x3F
#define INT_STA				0x37	// Contains flags. Cleared on read.
//BNO Page 1 Register Definitions
#define INT_EN				0x10	//
#define INT_MSK				0x0F	//



#define ACC_DATA_X_LSB    0x08
#define MAG_DATA_X_LSB    0x0E
#define GYR_DATA_X_LSB    0x14
#define LIA_Data_X_LSB    0x28
#define GRV_Data_X_LSB    0x2E

// SYS_TRIGGER 	bit definitions
#define CLK_SEL				7
#define RST_INT				6
#define	RST_SYS				5
#define Self_Test			0

// INT_EN		bit definitions
#define ACC_NM				7
#define ACC_AM				6
#define ACC_HIGH_G			5
#define GYR_HIGH_RATE		3
#define GYRO_AM				2

// OPR_MODE		definitions
#define BNO_COMP_MODE( data, mode )	((data&0x0F)==(mode&0x0F))	// Only care about last bits
#define	BNO_MODE_CONFIG		0b00000000
#define BNO_MODE_NDOF		0b00001100
#define BNO_MODE_ACCONLY	0b00000001
#define BNO_MODE_MAGONLY	0b00000010
#define BNO_MODE_GYROONLY	0b00000011
#define BNO_MODE_ACCMAG		0b00000100
#define BNO_MODE_ACCGYRO	0b00000101
#define BNO_MODE_MAGGYRO	0b00000110
#define BNO_MODE_AMG		0b00000111
#define BNO_MODE_IMU		0b00001000
#define BNO_MODE_COMPASS	0b00001001
#define BNO_MODE_M4G		0b00001010
#define BNO_MODE_NDOF_NOFMC	0b00001011
#define BNO_MODE_NDOF		0b00001100



//BNO Vector Types
typedef enum
{
  Acceleration,
  MagneticField,
  AngularVelocity,
  LinearAcceleration,
  GravityVector
}BNO_Vector_t;

//BNO Power Modes
typedef enum
{
  Normal,
  LowPower,
  Suspend
}BNO_PwrMode_t;

//BNO Operation Modes
typedef enum
{
  CONFIGMODE,
  ACCONLY,
  MAGONLY,
  GYROONLY,
  ACCMAG,
  ACCGYRO,
  MAGGYRO,
  AMG,
  IMU,
  COMPASS,
  M4G,
  NDOF_FMC_OFF,
  NDOF
}BNO_OprMode_t;

// BNO Read ISR State Machine Definitions
typedef enum
{
	SMCOLDSTART,
	SMMASTTRANS,
	SMSENDREG,
	SMRESTART,
	SMMASTRECV,
	SMREADREG,
	SMHOTSTART
}BNO_Read_SM_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}vec3_t;

typedef struct
{
  int16_t pitch;
  int16_t roll;
  int16_t heading;
}vec3Eul_t;

typedef struct
{
  int16_t w;
  int16_t x;
  int16_t y;
  int16_t z;
}vec4_t;

typedef struct
{
    uint8_t config, temp;
    vec3_t linear,accel,gyro;
    vec3Eul_t euler;
}bnoData_t;

typedef struct _bno_datum_t
{
	uint8_t data_len;
	uint8_t *data_dest;
	uint8_t	BNO_addr;
	struct _bno_datum_t *next;
}BNO_datum_t;

extern void I2C_BNO_Init(void);
extern void BNO_Init(BNO_PwrMode_t PwrMode, BNO_OprMode_t OprMode);//void BNO_Init(void);
extern uint8_t BNO_RecvTest(void);
extern uint8_t BNO_Recv(uint8_t regAddress);
extern uint8_t BNO_MultiRecv(uint8_t regAddress, uint8_t numBytes);
extern void BNO_MultiRecvPacked(uint8_t regAddress, uint8_t numBytes, uint8_t* recvByte);
extern void BNO_Send(uint8_t regAddress, uint8_t data);
extern vec3Eul_t BNO_GetEuler(void); //Units will need to be divided by 16 to get in degrees
extern vec4_t BNO_GetQuaternion(void);
extern vec3_t BNO_GetVector(BNO_Vector_t type); //Results need to be divided to obtain proper units

//Still need to write

//Change BNO_Init to select the power and operation mode with an enum
//Get rid of unneeded functions



