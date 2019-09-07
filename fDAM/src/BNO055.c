#include <BNO055.h>
#include "main.h"
#include "misc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

BNO_datum_t	*BNOList = NULL;
uint8_t	BNODoneReading = 0;
bnoData_t BNOData;

uint8_t recvByte;

void I2C1_EV_IRQHandler(void)
{
	// Check where in the linked list of BNO values we are.

	// Check where in the I2C state machine we are.

	static	BNO_Read_SM_t	I2C_sm = SMCOLDSTART;
	static	uint8_t		n_bytes_req, n_bytes_rec;
	static	BNO_datum_t	*datum;

	switch(I2C_sm)
	{
	case SMCOLDSTART: // I asked for a start condition
		datum = BNOList; // get the head of the list
		if(I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))
		{
			// Send BNO addr with write bit
			I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Transmitter);
			// Set I2C_sm to 2 (sending address with write bit to slave)
			I2C_sm = SMMASTTRANS;
		}

		break;
	case SMMASTTRANS:	// I asked for an I2C send
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
			// Send the register address to the BNO.
			I2C_SendData(I2C1,datum->BNO_addr); //Register
			// Set sm to 3(sending the data -- a register address -- to the slave)
			I2C_sm = SMSENDREG;
		}
		break;
	case SMSENDREG:	// I asked to send a byte
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			// Ask for a start condition
			I2C_GenerateSTART(I2C1,ENABLE);
			// Set sm to 4(asking for a start condition)
			I2C_sm = SMRESTART;
		}
		break;
	case SMRESTART: // I asked for a start condition
		if(I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))
		{
			// Send BNO's address with read bit
			I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Receiver);
			// Set sm to 5 (asking to select mode)
			I2C_sm = SMMASTRECV;
		}
		break;
	case SMMASTRECV: // I asked to select receiver mode
		// If we're only receiving one byte, we need to configure
		// NACK before clearing the ADDR flag. Reading the status
		// registers clears the ADDR flag.
		/*
		 * In case a single byte has to be received,
		 *  the Acknowledge disable is made during EV6
		 *  (before ADDR flag is cleared) and
		 *  the STOP condition generation is made after EV6.
		 *  EV6 is the I2C_CheckEvent line.
		 */
		n_bytes_rec = 0;
		n_bytes_req = datum->data_len;
		if(n_bytes_req == 1) I2C_AcknowledgeConfig(I2C1,DISABLE);
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
			// Now we're in a position to start receiving bytes
			if(n_bytes_req == 1)
			{
				// Configure NACK and STOP if the next byte is the last one
				// in this transaction.
				I2C_GenerateSTOP(I2C1, ENABLE);
			}
			I2C_sm = SMREADREG;
		}
		break;
	case SMREADREG:// Now, we're waiting for bytes to be received.
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))// New byte has been read.
		{
			// Put that byte where the BNO_datum wants us to put it.
			*(datum->data_dest + n_bytes_rec) = I2C_ReceiveData(I2C1);
			n_bytes_rec++;
			/*
			 * To generate the NACK pulse after the last received data byte,
			 * the ACK bit must be cleared just after reading the second last data byte
			 * (after second last RxNE event).
			 * In order to generate the Stop/Restart condition, software must set the
			 * STOP/START bit after reading the second last data byte
			 * (after the second last RxNE event).
			 */
			if((n_bytes_req - n_bytes_rec) == 1) // We just read the second-to-last byte
			{
				I2C_AcknowledgeConfig(I2C1,DISABLE);
				I2C_GenerateSTOP(I2C1, ENABLE);
			}
			if(n_bytes_rec >= n_bytes_req) // we're done reading bytes for this datum
			{
				/* I just read out the final byte.
				 * The NACK and STOP were requested.
				 * After the STOP condition, the I2C hardware automatically goes back into
				 * slave mode.
				 * They may not have occurred yet, though.
				 * Enter a mode where we wait for the stop condition.
				 * I need to block here, because the master won't generate an interrupt
				 * on STOP. The switch to slave mode is signalled by MSL going low.
				 */
				while(I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL));
				/*
				 * Then, I can check what to do next and re-enable ACK
				 */
				// Re-enable ACK
				I2C_AcknowledgeConfig(I2C1,ENABLE);
				if(datum->next == NULL)
				{
					BNODoneReading = 1;
					datum = BNOList;
					I2C_sm = SMCOLDSTART;
				}
				else
				{
					datum = datum->next;
					I2C_sm = SMHOTSTART;
					// Generate a start condition to get this ball rolling again.
					I2C_GenerateSTART(I2C1,ENABLE);
				}
			}
		}
		break;
	case SMHOTSTART:
		// Just like condition 1, but datum is already defined for us.
		//datum = BNOList;
		if(I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))
		{
			// Send BNO addr with write bit
			I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Transmitter);
			// Set I2C_sm to 2 (sending address with write bit to slave)
			I2C_sm = SMMASTTRANS;
		}
		break;
	default:
		break;
	}
}



void I2C_BNO_Init(void)
{
  //Set up Init Structs for I2C and I2C Pins
  I2C_InitTypeDef I2C_InitStruct;
  GPIO_InitTypeDef I2C_GPIO_InitStruct;
  NVIC_InitTypeDef	NVIC_InitStructure;
  //Remap I2C pins to PB8 and PB9
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  //GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE);

  //Enable I2C peripheral
  I2C_DeInit(I2C1);
  I2C_Cmd(I2C1,ENABLE);

  //Peripheral clock Enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //Check this for F4

  //Pin configuration
  I2C_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //Check this for F4
  I2C_GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
  I2C_GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; //Added for F4
  I2C_GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  I2C_GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;     // enable pull up resistors

  GPIO_Init(GPIOB, &I2C_GPIO_InitStruct);

  //Connect I2C1 pins to AF
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); // SCL
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

  //I2C configuration
  I2C_InitStruct.I2C_ClockSpeed = I2C_Speed;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0x00;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable; //BNO Must recv ack
  I2C_InitStruct.I2C_AcknowledgedAddress =  I2C_AcknowledgedAddress_7bit;

  I2C_Init(I2C1, &I2C_InitStruct);
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void BNO_Append(BNO_datum_t **head,uint8_t addr, uint8_t len, uint8_t *dest)
{
	BNO_datum_t *right;
	BNO_datum_t *temp = (BNO_datum_t *)malloc(sizeof(BNO_datum_t));
	temp->BNO_addr = addr;
	temp->data_dest = dest;
	temp->data_len = len;
	temp->next = NULL;
	right = *head;
	if(right == NULL)
	{
		*head = temp;
	}
	else
	{
		while(right->next != NULL) right = right->next;
		right->next = temp;
	}
}

void BNO_Init(BNO_PwrMode_t PwrMode, BNO_OprMode_t OprMode)
{
	// Set up the BNO_datum linked list
	BNO_Append(&BNOList,	OPR_MODE,			1,	&(BNOData.config));
	BNO_Append(&BNOList,	TEMP,				1,	&(BNOData.temp));
	BNO_Append(&BNOList,	LIA_Data_X_LSB,		6,	(uint8_t *)&(BNOData.linear.x));
	BNO_Append(&BNOList,	EUL_Heading_LSB,	6,	(uint8_t *)&(BNOData.euler.pitch));
	BNO_Append(&BNOList,	ACC_DATA_X_LSB,		6,	(uint8_t *)&(BNOData.accel.x));
	BNO_Append(&BNOList,	GYR_DATA_X_LSB,		6,	(uint8_t *)&(BNOData.gyro.x));
	// Set up the I2C interface
	I2C_BNO_Init();
	BNO_Send(PWR_MODE,PwrMode);
	BNO_Send(OPR_MODE,OprMode);
}

/*
void BNO_Init(void)
{
  //Start I2C Communication to BNO
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  //Configure BNO Operation Mode
  I2C_SendData(I2C1,0x3D); //Register
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_SendData(I2C1,0b00001100); //NDOF Mode
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  //Stop I2C Communication to BNO
  I2C_GenerateSTOP(I2C1, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}
*/

void BNO_Send(uint8_t regAddress, uint8_t data)
{
  //Start I2C Communication to BNO
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  //Configure BNO Mode
  I2C_SendData(I2C1,regAddress); //Register
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_SendData(I2C1,data); //Data
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  //Stop I2C Communication to BNO
  I2C_GenerateSTOP(I2C1, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));

}

uint8_t BNO_RecvTest(void)
{
  uint8_t recvByte;

  //Wait for bus to be free
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

  //Start I2C Communication to BNO
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select write mode and select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  //Send register to read
  I2C_SendData(I2C1,OPR_MODE); //Register
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  //Start Repeat
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select read mode and select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  //NACK to stop BNO from sending data
  I2C_AcknowledgeConfig(I2C1,DISABLE);

  //Stop I2C Communication with BNO
  I2C_GenerateSTOP(I2C1, ENABLE);
  //while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));

  //Read single byte from BNO
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  recvByte = I2C_ReceiveData(I2C1);

  //Re-enable ACK
  I2C_AcknowledgeConfig(I2C1,ENABLE);

  return recvByte;
}

uint8_t BNO_Recv(uint8_t regAddress)
{
  uint8_t recvByte;

  //Wait for bus to be free
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

  //Start I2C Communication to BNO
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select write mode and select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  //Send register to read
  I2C_SendData(I2C1,regAddress); //Register
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  //Start Repeat
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select read mode and select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  //NACK to stop BNO from sending data
  I2C_AcknowledgeConfig(I2C1,DISABLE);

  //Stop I2C Communication with BNO
  I2C_GenerateSTOP(I2C1, ENABLE);
  //while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));

  //Read single byte from BNO
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  recvByte = I2C_ReceiveData(I2C1);

  //Re-enable ACK
  I2C_AcknowledgeConfig(I2C1,ENABLE);

  return recvByte;
}

uint8_t BNO_MultiRecv(uint8_t regAddress, uint8_t numBytes)//, u8* recvByte)
{
  uint8_t recvByte;

  //Wait for bus to be free
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

  //Start I2C Communication to BNO
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select write mode and select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  //Send register to read
  I2C_SendData(I2C1,regAddress); //Register
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  //Start Repeat
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select read mode and select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  while(numBytes > 0)
  {
      if (numBytes == 1)
      {
          //NACK to stop BNO from sending data
          I2C_AcknowledgeConfig(I2C1,DISABLE);

          //Stop I2C Communication with BNO
          I2C_GenerateSTOP(I2C1, ENABLE);
          //while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
      }

      //Read single byte from BNO
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
      recvByte = I2C_ReceiveData(I2C1);
      //*recvByte = I2C_ReceiveData(I2C1);
      //recvByte++;

      numBytes--;

  }
  //Re-enable ACK
  I2C_AcknowledgeConfig(I2C1,ENABLE);

  return recvByte;
}

void BNO_MultiRecvPacked(uint8_t regAddress, uint8_t numBytes, uint8_t* recvByte)
{
  //Wait for bus to be free
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

  //Start I2C Communication to BNO
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select write mode and select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  //Send register to read
  I2C_SendData(I2C1,regAddress); //Register
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  //Start Repeat
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));

  //Select read mode and select BNO as I2C Slave
  I2C_Send7bitAddress(I2C1,I2C_BNO_ADDRESS<<1,I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  while(numBytes > 0)
  {
      if (numBytes == 1)
      {
          //NACK to stop BNO from sending data
          I2C_AcknowledgeConfig(I2C1,DISABLE);

          //Stop I2C Communication with BNO
          I2C_GenerateSTOP(I2C1, ENABLE);
          //while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
      }

      //Read single byte from BNO
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
      *recvByte = I2C_ReceiveData(I2C1);
      recvByte++;

      numBytes--;
  }
  //Re-enable ACK
  I2C_AcknowledgeConfig(I2C1,ENABLE);
}

vec3Eul_t BNO_GetEuler(void)
{
  uint8_t multi[6] = {0,0,0,0,0,0};
  uint16_t tempHL,tempHM,tempRL,tempRM,tempPL,tempPM;
  vec3Eul_t Euler;
  //u16 dummy,dummy2,dummy3;

  BNO_MultiRecvPacked(EUL_Heading_LSB,6,&multi[0]);

  //Heading
  tempHM = multi[1]<<8;
  tempHL = multi[0];
  Euler.heading = (tempHM | tempHL); //Divide by 16 to get into degrees, may need to change data type to keep percision if we want to make the conversion on the STM

  //Roll
  tempRM = multi[3]<<8;
  tempRL = multi[2];
  Euler.roll = (tempRM | tempRL);

  //Pitch
  tempPM = multi[5]<<8;
  tempPL = multi[4];
  Euler.pitch = (tempPM | tempPL);

  /*
  dummy = Euler.heading;
  dummy2 = Euler.roll;
  dummy3 = Euler.pitch;
  */

  return Euler;
}


vec4_t BNO_GetQuaternion(void)
{
  uint8_t multi[8] = {0,0,0,0,0,0,0,0};
  uint16_t tempWL,tempWM,tempXL,tempXM,tempYL,tempYM,tempZL,tempZM;
  vec4_t Quaternion;
  uint16_t dummy,dummy2,dummy3,dummy4;

  BNO_MultiRecvPacked(0x00,8,&multi[0]); //Change to QUA_Data_w_LSB to get actual data

  //W
  tempWM = multi[1]<<8;
  tempWL = multi[0];
  Quaternion.w = tempWM | tempWL;

  //X
  tempXM = multi[3]<<8;
  tempXL = multi[2];
  Quaternion.x = tempXM | tempXL;

  //Y
  tempYM = multi[5]<<8;
  tempYL = multi[4];
  Quaternion.y = tempYM | tempYL;

  //Z
  tempZM = multi[7]<<8;
  tempZL = multi[6];
  Quaternion.z = tempZM | tempZL;

  dummy = Quaternion.w;
  dummy2 = Quaternion.x;
  dummy3 = Quaternion.y;
  dummy4 = Quaternion.z;

  return Quaternion;
}


vec3_t BNO_GetVector(BNO_Vector_t type)
{
  uint8_t regAddress = 0;
  uint8_t multi[6] = {0,0,0,0,0,0};
  uint16_t tempXL,tempXM,tempYL,tempYM,tempZL,tempZM;
  vec3_t vector;
  uint16_t dummy,dummy2,dummy3;

  switch(type)
  {
    case Acceleration:
      BNO_MultiRecvPacked(ACC_DATA_X_LSB,6,&multi[0]);
      break;
    case MagneticField:
      BNO_MultiRecvPacked(MAG_DATA_X_LSB,6,&multi[0]);
      break;
    case AngularVelocity:
      BNO_MultiRecvPacked(GYR_DATA_X_LSB,6,&multi[0]);
      break;
    case LinearAcceleration:
      BNO_MultiRecvPacked(LIA_Data_X_LSB,6,&multi[0]);
      break;
    case GravityVector:
      BNO_MultiRecvPacked(GRV_Data_X_LSB,6,&multi[0]);
      break;
  }

  //Heading
  tempXM = multi[1]<<8;
  tempXL = multi[0];
  vector.x = (tempXM | tempXL);

  //Roll
  tempYM = multi[3]<<8;
  tempYL = multi[2];
  vector.y = (tempYM | tempYL);

  //Pitch
  tempZM = multi[5]<<8;
  tempZL = multi[4];
  vector.z = (tempZM | tempZL);

  dummy = vector.x;
  dummy2 = vector.y;
  dummy3 = vector.z;

  return vector;
}

