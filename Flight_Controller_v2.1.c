/*
 * Flight_Controller_Final.c
 *
 * Author : Rhugved Chaudhari
 */ 

#include<avr/io.h>
#include<avr/interrupt.h>
#include<math.h>

#define GyroConfigReg 27
#define AccConfigReg 28
#define AccConfig2Reg 29
#define AccStAddr 59
#define WhoAmIReg 117
#define PWR_MGMT_1_REG 107
#define PWR_MGMT_2_REG 108

#define PI 3.1416
#define CALIBRATION_SAMPLE_NOS 100

#define MPU_Addr 0b01101000
#define MPU_READ 1
#define MPU_WRITE 0
#define MPU_BITRATE 50000 // in bits/sec
#define MCKFreq 8000000

#define ESC_MCKFreq 8000000 // Master Clock Frequency
#define USARTBAUD 62500 // Baud rate of USART Communication
#define UBRR0Val (MCKFreq / 16 / USARTBAUD) - 1 // Baud Rate registers value

#define CHECKSUM_OFFSET 0x50
#define TWI_IDLE 0			//TWI Idle
#define READ_MPU_DATA 1		//Byte Read
#define WRITE_MPU_DATA 2	//Byte Write

#define STRT_BYTE 0
#define SLA_W_SENT 1		//Slave Address + Read Bit Sent
#define SLA_R_SENT 2		//Slave Address + Write Bit Sent
#define REG_ADDR_SENT 3
#define BYTE_SENT 4
#define STOP_BYTE 5
#define REPEAT_STRT_BYTE 6
#define BYTE_REQUESTED 7
#define RECEPTION_DONE 8
#define WRITING_DONE 9

#define SEND_BYTE 0
#define SEND_REG_ADDR 1

#define SEND_STRT 0
#define SEND_REPEAT_STRT 1

#define NO_ERROR 0
#define ERROR_OCCURED 1

#define MOV_AVG_BLK_SIZE 320

#define COMP_FILTER_GYRO_GAIN 0.98F
#define COMP_FILTER_ACC_GAIN (1.0F - COMP_FILTER_GYRO_GAIN)
#define ANGLE_CLAMP_VAL 1.6F

#define RXD_CHECKSUM_OFFSET 0
#define VUART_TXD_ACTIVE 2
#define VUART_TXD_INACTIVE 3
#define RXD_IDLE 4
#define RXD_CHECK_STRT_BIT 5
#define RXD_STRT_BIT_RECEIVED 6
#define RXD_NO_OF_BYTES_TO_RECEIVE 2
#define CONSEC_SAMPLE_FOR_A_BIT 8
#define CONSEC_SAMPLE_1HIGHBIT 6
#define CONSEC_SAMPLE_2HIGHBIT 13
#define CONSEC_SAMPLE_3HIGHBIT 20
#define CONSEC_SAMPLE_FOR_STRT_BIT (CONSEC_SAMPLE_FOR_A_BIT / 2)

#define LOWEST_STAT_ON_FREQ 210
#define NORMAL_STAT_ON_FREQ 5500
#define HIGHEST_STAT_ON_FREQ 6200

uint8_t STAT_ON_FREQ_INC_DEC = 4;

uint8_t Kp_X = 30;
#define Ki_X 0
uint8_t Kd_X = 45;

uint8_t Kp_Y = 30;
#define Ki_Y 0
uint8_t Kd_Y = 45;

#define MAX_PID_VAL 500    
#define MIN_PID_VAL -500   

#define MAX_PID_STEP_VAL 30

#define Receiver_STRT_BYTE 0x0F
#define Receiver_STOP_BYTE 0xF0

#define JoyStick_REVERSE 0
#define JoyStick_NEUTRAL 1
#define JoyStick_FORWARD 2

struct tagESC_Data
{
	uint16_t OCR1A_VAl_forStatONFreq;
	uint16_t OCR02AB_Val_forDutyCycle;
};

struct tagESC_Data ESC_Data[4];

uint8_t GetMPUOrientation(void);
uint8_t ReadMPUData(uint8_t RegAddr, uint8_t NoOfBytes);
void MPU_Init(void);
void SendSlaveAddr(uint8_t RdOrWr);
void SendByte(uint8_t Data, uint8_t ISRegAddr);
void SendStrtByte(uint8_t ISRepeatedStrt);
void SendStopByte(void);
void ReceiveByte(void);
void RequestFirstByte(void);
void WriteMPUData(uint8_t RegAddr, uint8_t NoOfBytes);
void EnableTWIInterrupt(void);
void DisableTWIInterrupt(void);
void VirtualUARTInit(void);
void Delay_ms(uint16_t NoOfms);
void Send_VUART_Data(int16_t AngX, int16_t AngY);
void Send_VUART_Byte(uint8_t Byte);
void StartTimer1(void);
void StopTimer1(void);
void Timer1Init_for_VUART(void);
void StartTimer0(void);
void StopTimer0(void);
void Timer0_Init(void);
void USARTInit(void);
void SendESCData(struct tagESC_Data *ESC_Data);
void DisableDataRegEmptyInterrupt(void);
void EnableDataRegEmptyInterrupt(void);
void AcclRetarESC(uint16_t StartStatOnFreq, uint16_t EndStatOnFreq);
void AutoCalibrate(void);
void ExecuteStepwisePID(void);
void PID(void);
void EnableRXCompleteInterrupt(void);
void DisableRXCompleteInterrupt(void);
void ExecutePID(void);
void AccelarateMotors(void);
void TwoMotorSpeedControl(void);
void AccelarateMotorsAndRoll(void);
void AccelarateMotorsAndRollFixed(void);

uint8_t TWIStatus = 0;
uint8_t DutyCycle;
uint8_t DeltaStatONFreq;
uint16_t StatOnFreq = 0;
uint16_t StatOnFreq_ToSend[4];
uint8_t TWIStatRegVal = 0; // Value of (TWSR & 0xF8) will be stored in this variable
uint8_t CrntSDAStatus = 0;
uint8_t TWIErrorOccurd = NO_ERROR;
uint8_t RdWrRegAddr = 0;
uint8_t ByteBuf[14];
uint8_t NoOfBytesToRead = 0;
uint8_t NoOfBytesWrote = 0;
uint8_t NoOfBytesToWrite = 0;
uint8_t TC0_OVF_Cnt = 0;
uint8_t GyroAngCnt = 0;
uint8_t IsGyroAngXLessThanZero = 1;
uint8_t IsGyroAngYLessThanZero = 1;

uint8_t TWCR_OR_VAL, TWCR_AND_VAL;

float MovAvgX_BlkSum = 0;
int16_t MovAvgX_BlkVals[MOV_AVG_BLK_SIZE];
uint16_t MovAvgX_BlkIndx = 0;

float TempFloat = 0;

float MovAvgY_BlkSum = 0;
int16_t MovAvgY_BlkVals[MOV_AVG_BLK_SIZE];
uint16_t MovAvgY_BlkIndx = 0;

int16_t MPUAngValCnt = 0;

uint8_t glb_ISRegAddr;
uint8_t glb_TWIStatRegVal;

uint8_t VUART_BitIndx = 0;
uint8_t VUART_ByteIndx = 0;
uint16_t VUART_Byte = 0;
uint8_t VUART_Data_Bytes[4];
uint8_t Max_No_Of_Bytes;
uint8_t VUART_TXD_Status;
uint8_t VUART_RXD_Status;
uint8_t ConsecutiveEqualSampleCnt = 0;
uint8_t ConsecutiveHighSampleCnt = 0;

volatile uint8_t NoOfBytesReceived = 0;
char TempAccData[8];

uint8_t AccConfig2Val = 0;
float AccX, AccY, AccZ;
float AccAngX, AccAngY, AccAngZ;
float AccAngXOffset = 0, AccAngYOffset = 0;

float GyroX, GyroY, GyroZ, NewGyroValX = 0, NewGyroValY = 0;
float GyroXOffset = 0, GyroYOffset = 0, GyroZOffset = 0;
float GyroAngX, GyroAngY, GyroAngZ;

float FinalAngX, FinalAngY;
float ClampedAccAngX = 0, ClampedFinalAngX = 0, ClampedGyroAngX = 0;
float ClampedAccAngY = 0, ClampedFinalAngY = 0, ClampedGyroAngY = 0;
float FinalAngXOffset = 0, FinalAngYOffset = 0;
float TimeElapsed = 0;

float P_Val_X = 0, I_Val_X = 0, D_Val_X = 0;
int16_t PID_Diff_X = 0, PrevPID_Diff_X = 0, PrevPID_Val_X = 0, PID_Val_X = 0;
float ErrorAngX = 0, DesiredAngX = 0, PrevErrorAngX = 0;

float P_Val_Y = 0, I_Val_Y = 0, D_Val_Y = 0;
int16_t PID_Diff_Y = 0, PrevPID_Diff_Y = 0, PrevPID_Val_Y = 0, PID_Val_Y = 0;
float ErrorAngY = 0, DesiredAngY = 0, PrevErrorAngY = 0;

float CheckFloat = 0, TCNT0Val = 0;

uint8_t SendStatONFreqFlag = 0;
uint8_t SendBytes[19];
uint8_t SendByteIndx = 0;

uint16_t DelayVal;
uint16_t Delay = 0;

uint16_t SendValCnt = 0;

void USARTInit(void)
{
	// Set the Baud rate by writing into the registers the baud rate Prescaler value
	UBRR0 = UBRR0Val;

	/* 
		Enable the Transmitter 
		Enable the Receiver
		Set the bit UCSZ02 to 0  
		(Setting the bits UCSZ0(2:0) in the registers (UCSR0B & UCSR0C) to 0b011 sets the character size to 8-bit
	*/ 
	UCSR0B = 0b00011000;
	
	/*
		<<Set Frame Format>>
		
		Set the Mode to Asynchronous USART (UMSEL0(1:0) = 0b00)
		Disable the Parity mode (UPM0(1:0) = 0b00)
		One Stop bit
		Character Size = 8-bit (UCSZ0(2:0) = 0b011)
	*/
	UCSR0C = 0b00000110;
	
	//Enable global interrupts
	sei();
}

void EnableDataRegEmptyInterrupt(void)
{
	UCSR0B |= 0b00100000;
}

void DisableDataRegEmptyInterrupt(void)
{
	UCSR0B &= 0b11011111;
}

void EnableRXCompleteInterrupt(void)
{
	UCSR0B |= 0x80;
}

void DisableRXCompleteInterrupt(void)
{
	UCSR0B &= 0x7F;
}

void SendESCData(struct tagESC_Data *ESC_Data)
{
	SendBytes[0] = 'S' | 0x80;					//Start Byte
	
	uint8_t ESC_ID = 0;
	uint8_t Indx = 1;
	
	for(ESC_ID = 0; ESC_ID < 4; ESC_ID++)
	{
		SendBytes[Indx] = (ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq >> 7) & 0x7F;		//The Higher byte of StatONFreq
		SendBytes[Indx + 1] = ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq & 0x7F;          //The Lower byte of StatONFreq
		SendBytes[Indx + 2] = ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle & 0x7F;		
		SendBytes[Indx + 3] = ((ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq >> 14) | ((ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle & 0x80) >> 1)) & 0x7F;

		Indx += 4;
	}
	
	uint8_t CheckSum = 0;
	for(Indx = 0; Indx < 17; Indx++)
		CheckSum += SendBytes[Indx];
	
	SendBytes[17] = (CheckSum + CHECKSUM_OFFSET) & 0x7F;	//The Checksum byte
	SendBytes[18] = 'E' | 0x80;			    //The End Byte
	SendByteIndx = 0;
	
    EnableDataRegEmptyInterrupt();
	//SendStatONFreqFlag = 1;
	
	//UDR0 = SendBytes[0];
	//SendByteIndx++;
}

ISR(USART_UDRE_vect)
{
	//if(SendStatONFreqFlag == 1)
	{
		UDR0 = SendBytes[SendByteIndx];
		
		SendByteIndx++;
		if(SendByteIndx > 18) 
        {
            SendByteIndx = 0;
            SendStatONFreqFlag = 0;
            DisableDataRegEmptyInterrupt();
        }
	}
}

void AcclRetarESC(uint16_t StartStatOnFreq, uint16_t EndStatOnFreq)
{
	uint16_t DeltaStatOnFreq, StatOnFreq;
	int16_t  IncDec;
	uint8_t DutyCycle;

	if(StartStatOnFreq == EndStatOnFreq)	return;

	if(StartStatOnFreq < EndStatOnFreq)		
	{
		DeltaStatOnFreq = EndStatOnFreq - StartStatOnFreq;
		IncDec = STAT_ON_FREQ_INC_DEC;
	}
	else
	{
		DeltaStatOnFreq = StartStatOnFreq - EndStatOnFreq;
		IncDec = -STAT_ON_FREQ_INC_DEC;	
	}

	StatOnFreq = StartStatOnFreq;
	
	for(uint16_t Cnt = 0; Cnt < DeltaStatOnFreq; Cnt += STAT_ON_FREQ_INC_DEC, StatOnFreq += IncDec)
	{	
		TWIErrorOccurd = NO_ERROR;
		
		//Delay_ms(10);
		if(GetMPUOrientation() == ERROR_OCCURED) continue;
		
		for(uint8_t ESC_ID = 0; ESC_ID < 4; ESC_ID++)
		{
			StatOnFreq_ToSend[ESC_ID] += IncDec;
			
			ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)((float)ESC_MCKFreq / (StatOnFreq / 10.0) / 6);
			
			//if((ESC_ID == 3) && (StatOnFreq > (NORMAL_STAT_ON_FREQ - 250))) ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)((float)ESC_MCKFreq / ((NORMAL_STAT_ON_FREQ - 250) / 10.0) / 6);
			
			DutyCycle = (((StatOnFreq / 10) - 20) * 0.1) + 20;

			if(DutyCycle < 20)			DutyCycle = 20;
			else if(DutyCycle > 90)		DutyCycle = 90;

			ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
		}
		
		//Send_VUART_Data(StatOnFreq_ToSend[3], StatOnFreq_ToSend[1]);
		Send_VUART_Data(FinalAngX * 100, FinalAngY * 100);
		
		Delay_ms(2);
		SendESCData(&ESC_Data[0]);
	}
}

void AcclRetarESCSlow(uint16_t StartStatOnFreq, uint16_t EndStatOnFreq)
{
	uint16_t DeltaStatOnFreq, StatOnFreq;
	int16_t  IncDec;
	uint8_t DutyCycle;

	if(StartStatOnFreq == EndStatOnFreq)	return;

	if(StartStatOnFreq < EndStatOnFreq)
	{
		DeltaStatOnFreq = EndStatOnFreq - StartStatOnFreq;
		IncDec = STAT_ON_FREQ_INC_DEC;
	}
	else
	{
		DeltaStatOnFreq = StartStatOnFreq - EndStatOnFreq;
		IncDec = -STAT_ON_FREQ_INC_DEC;
	}

	StatOnFreq = StartStatOnFreq;
	
	for(uint16_t Cnt = 0; Cnt < DeltaStatOnFreq; Cnt += STAT_ON_FREQ_INC_DEC, StatOnFreq += IncDec)
	{
		TWIErrorOccurd = NO_ERROR;
		
		//Delay_ms(10);
		if(GetMPUOrientation() == ERROR_OCCURED) continue;
		
		for(uint8_t ESC_ID = 1; ESC_ID < 2; ESC_ID++)
		{
			StatOnFreq_ToSend[ESC_ID] += IncDec;
			//if(ESC_ID == 3) ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)((float)ESC_MCKFreq / (4000 / 10.0) / 6);
			//else 
			ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)((float)ESC_MCKFreq / (StatOnFreq / 10.0) / 6);
			
			DutyCycle = (((StatOnFreq / 10) - 20) * 0.1) + 20;

			if(DutyCycle < 20)			DutyCycle = 20;
			else if(DutyCycle > 90)		DutyCycle = 90;

			ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
		}
		
		Send_VUART_Data(StatOnFreq_ToSend[3], StatOnFreq_ToSend[1]);
		//Send_VUART_Data(StatOnFreq_ToSend[3], FinalAngX * 100);
		
		Delay_ms(200);
		if(StatOnFreq_ToSend[1] < 4010) Delay_ms(5000);
					
		SendESCData(&ESC_Data[0]);
	}
}

void MPU_Init(void)
{
	// Set the Bit rate for communication to the MPU
	// The Set the Prescaler to 1
	TWBR = ((MCKFreq / MPU_BITRATE) - 16) / 2;
	TWSR &= 0b11111100;
	
	//Enable Global Interrupt
	sei();
	
	// Enable the TWI Interrupt
	EnableTWIInterrupt();
	
	// Set Accelerometer Sensitivity Scale Factor to 2g
	ByteBuf[0] = 0x00;
	WriteMPUData(AccConfigReg, 1);
	
	//Disable Power management for accelerometer and gyroscope
	ByteBuf[0] = 0x00;
	WriteMPUData(PWR_MGMT_1_REG, 1);
	
	//Disable Power management for accelerometer and gyroscope
	ByteBuf[0] = 0x00;
	WriteMPUData(PWR_MGMT_2_REG, 1);
	
	// Set Gyroscope Sensitivity Scale Factor to 250 degrees / sec
	ByteBuf[0] = 0x00;
	WriteMPUData(GyroConfigReg, 1);
	
	for(uint16_t Indx = 0; Indx < MOV_AVG_BLK_SIZE; Indx++) 
	{
		MovAvgX_BlkVals[Indx] = 0;
		MovAvgY_BlkVals[Indx] = 0;
	}
	
	ReadMPUData(AccConfig2Reg, 1);
	AccConfig2Val = ByteBuf[0];
	
	Timer0_Init();
	StartTimer0();
}

void EnableTWIInterrupt(void)
{
	// Enable the TWI Interrupt
	TWCR |= 0x01;
}

void DisableTWIInterrupt(void)
{
	TWCR &= 0b11111110;
}

void StartTimer1(void)
{
	TCCR1B |= 1;
}

void StopTimer1(void)
{
	TCCR1B &= 0xFE;
}

void StartTimer0(void)
{
	TCCR0B |= 3;
}

void StopTimer0(void)
{
	TCCR1B &= 0xFE;
}

void Timer0_Init(void)
{
	TCCR0A = 0;
	TCCR0B = 0;
	TCNT0 = 0;
	TIMSK0 = 1;
	
	//Enable Global Interrupt
	sei();
}

void Timer1Init_for_VUART(void)
{
	DDRB |= 0x02;
	TCCR1A = 0b01010000;
	TCCR1B = 0b00001000;
	TCNT1 = 0;
	OCR1A = 855;
	TIMSK1 = 0b00000010;
	
	//Enable Global Interrupt
	sei();
}

ISR(TWI_vect)
{   
	if(TWIErrorOccurd != ERROR_OCCURED)
	{
		switch (TWIStatus)
		{
			case READ_MPU_DATA:
			{
				if(CrntSDAStatus == STRT_BYTE)
                {              
					SendSlaveAddr(MPU_WRITE);
				}
				else if(CrntSDAStatus == SLA_W_SENT)			
                {
                    SendByte(RdWrRegAddr, SEND_REG_ADDR);
                }
				else if(CrntSDAStatus == REG_ADDR_SENT)			
                {
                    SendStrtByte(SEND_REPEAT_STRT);
                }
				else if(CrntSDAStatus == REPEAT_STRT_BYTE)
                {
                    SendSlaveAddr(MPU_READ);
                }
				else if(CrntSDAStatus == SLA_R_SENT)			
                {
                    RequestFirstByte();   
				}
				else if((CrntSDAStatus == BYTE_REQUESTED) && (NoOfBytesReceived < NoOfBytesToRead))	
                {
                    ReceiveByte();     
                }
				if(CrntSDAStatus == RECEPTION_DONE)
				{
					SendStopByte();
                    DisableTWIInterrupt();
				}
			
				break;
			}
			case WRITE_MPU_DATA:
			{
				if(CrntSDAStatus == STRT_BYTE)
				{
					SendSlaveAddr(MPU_WRITE);
				}
				else if(CrntSDAStatus == SLA_W_SENT)
				{
					SendByte(RdWrRegAddr, SEND_REG_ADDR);
				}
				else if(CrntSDAStatus == REG_ADDR_SENT)
				{
					SendByte(ByteBuf[NoOfBytesWrote], SEND_BYTE);
				}
				else if((CrntSDAStatus == BYTE_SENT) && (NoOfBytesWrote < NoOfBytesToWrite))	
				{
					SendByte(ByteBuf[NoOfBytesWrote], SEND_BYTE);
				}
				else if(CrntSDAStatus == WRITING_DONE)
				{
                    SendStopByte();
                    DisableTWIInterrupt();
                }
			
				break;
			}
		}
	}
	
	TWCR |= TWCR_OR_VAL;
	TWCR &= TWCR_AND_VAL;
}

void RequestFirstByte(void)
{
	TWIStatRegVal = TWSR & 0xF8;
	
	//Check, if a Slave Address + Read bit has been transmitted correctly and ACK received
	if(TWIStatRegVal == 0x40)
	{
		if(NoOfBytesToRead == 1)
		{
			CrntSDAStatus = BYTE_REQUESTED;
			TWCR_OR_VAL = 0;   
			TWCR_AND_VAL = ~(1 << TWEA);            
		}
		else
		{
			CrntSDAStatus = BYTE_REQUESTED;
			//The first byte will be received and an ACK pulse will be sent
			
			TWCR_OR_VAL = 0b11000000;
			TWCR_AND_VAL = 0xFF;
		}
	}
	else 
    {
        TWIErrorOccurd = ERROR_OCCURED;
		
		TWCR_OR_VAL = 0b10000000;
		TWCR_AND_VAL = 0b10001000;
    }
}

void ReceiveByte(void)
{
	TWIStatRegVal = TWSR & 0xF8;
    
	//Check if a data byte has been received and ACK has been returned
	if(TWIStatRegVal == 0x50)
	{
		ByteBuf[NoOfBytesReceived] = TWDR;
		CrntSDAStatus = BYTE_REQUESTED;
		NoOfBytesReceived++;
 		
		if(NoOfBytesReceived == (NoOfBytesToRead - 1))
		{
			TWCR_OR_VAL = 0;
            TWCR_AND_VAL = ~(1 << TWEA);
		}
		else
		{
			TWCR_OR_VAL = 0b11000000;
			TWCR_AND_VAL = 0xFF;
		}
	}
	//Check if a data byte has been received and NACK has been returned
	else if(TWIStatRegVal == 0x58)
	{
		ByteBuf[NoOfBytesReceived] = TWDR;	
		NoOfBytesReceived++;
		
		if(NoOfBytesReceived == NoOfBytesToRead) 
        {
            CrntSDAStatus = RECEPTION_DONE;
			
			TWCR_OR_VAL = 0b10000000;
			TWCR_AND_VAL = 0xFF;
        }
		else 
        {
            TWIErrorOccurd = ERROR_OCCURED;
			
            TWCR_OR_VAL = 0b10000000;
            TWCR_AND_VAL = 0b10001000;
        }
	}
	else
    {   
        TWIErrorOccurd = ERROR_OCCURED;
		
		TWCR_OR_VAL = 0b10000000;
		TWCR_AND_VAL = 0b10001000;
    }
}

void SendByte(uint8_t Data, uint8_t ISRegAddr)
{
	TWIStatRegVal = TWSR & 0xF8;
	
	//Check, if a Slave Address + (Write bit/Read bit) has been transmitted correctly and ACK received
	//OR
	//Check if the data is transmitted successfully
	if	(
			   ((ISRegAddr == SEND_REG_ADDR) && ((TWIStatRegVal == 0x18) || (TWIStatRegVal == 0x40))) 
			|| ((ISRegAddr == SEND_BYTE) && (TWIStatRegVal == 0x28))
		)
	{
		glb_TWIStatRegVal = 0xEE;
		glb_ISRegAddr = 0xAA;
		
		// Write the Register Address from which byte is to be read
		TWDR = Data;
		// Clear the TWI Interrupt flag (This flag is cleared by writing 1 to it)
		// Enable TWEN to start the transmission
		TWCR_OR_VAL = 0b10000100;
		TWCR_AND_VAL = 0b11001101;
		
		CrntSDAStatus = REG_ADDR_SENT; 
		
		if(ISRegAddr == SEND_BYTE)
		{
			CrntSDAStatus = BYTE_SENT;
			NoOfBytesWrote++;
			
			if(NoOfBytesWrote == NoOfBytesToWrite)
			{
				CrntSDAStatus = WRITING_DONE;
			}
		}
	}
	else 
	{
		glb_TWIStatRegVal = TWIStatRegVal;
		glb_ISRegAddr = ISRegAddr;
		
		TWIErrorOccurd = ERROR_OCCURED;
		
		TWCR_OR_VAL = 0b10000000;
		TWCR_AND_VAL = 0b10001000;
	}
}

void SendSlaveAddr(uint8_t RdOrWr)
{
	TWIStatRegVal = TWSR & 0xF8;
	
	//Check, if a Start or a repeated start condition has been transmitted correctly
	if((TWIStatRegVal == 0x08) || (TWIStatRegVal == 0x10))
	{
		// Write Slave Address + (Read/Write) bit to TWI Data Register
		TWDR = (MPU_Addr << 1) + RdOrWr;
		
		//Clear the TWI Interrupt flag (This flag is cleared by writing 1 to it)
		//Enable TWEN to start the transmission
		TWCR_OR_VAL = 0b10000100;
		TWCR_AND_VAL = 0b11001101;
		
		CrntSDAStatus = SLA_W_SENT;
		
		if(RdOrWr == MPU_READ) CrntSDAStatus = SLA_R_SENT;
	}
	else
	{
		TWIErrorOccurd = ERROR_OCCURED;
		
		TWCR_OR_VAL = 0b10000000;
		TWCR_AND_VAL = 0b10001000;
	}
}

void SendStopByte(void)
{
	CrntSDAStatus = STOP_BYTE;
	// Send Stop Byte
	// Clear the TWI Interrupt flag (This flag is cleared by writing 1 to it)
	TWCR_OR_VAL = 0b10010100;
	TWCR_AND_VAL = 0b11011101;
}

void SendStrtByte(uint8_t ISRepeatedStrt)
{
	CrntSDAStatus = STRT_BYTE;
	
	if(ISRepeatedStrt) 
	{
		CrntSDAStatus = REPEAT_STRT_BYTE;
		
		//Send start condition
		//Clear the TWI Interrupt flag (This flag is cleared by writing 1 to it)
		TWCR_OR_VAL = 0b10100100;
		TWCR_AND_VAL = 0b11101101;
	}
	else
	{
		TWCR |= 0b10100100;
		TWCR &= 0b11101101;
	}
}

uint8_t ReadMPUData(uint8_t RegAddr, uint8_t NoOfBytes)
{
	for(uint8_t Indx = 0; Indx < 14; Indx++)
		ByteBuf[Indx] = 0;
	
	TWIStatus = READ_MPU_DATA;
	RdWrRegAddr = RegAddr;
	NoOfBytesToRead = NoOfBytes;
	NoOfBytesReceived = 0;
	
    EnableTWIInterrupt();
	SendStrtByte(SEND_STRT);
	
	while(NoOfBytesReceived != NoOfBytes)
	{
		if(TWIErrorOccurd == ERROR_OCCURED)
		{
			Send_VUART_Data(3000, 4000);
			Delay_ms(10);
			TWIStatus = TWI_IDLE;
			return ERROR_OCCURED;
		}
	}
	
	TWIStatus = TWI_IDLE;
	return NO_ERROR;
}

void WriteMPUData(uint8_t RegAddr, uint8_t NoOfBytes)
{	
	TWIStatus = WRITE_MPU_DATA;
	RdWrRegAddr = RegAddr;
	NoOfBytesToWrite = NoOfBytes;
	NoOfBytesWrote = 0;
	
    EnableTWIInterrupt();
	SendStrtByte(SEND_STRT);
	
	Delay_ms(10);
	
	if(TWIErrorOccurd == ERROR_OCCURED)
	{
		//PORTD |= 0x80;
	}
	
	TWIStatus = TWI_IDLE;
}

uint8_t GetMPUOrientation(void)
{
	if(ReadMPUData(AccStAddr, 14) == ERROR_OCCURED) return ERROR_OCCURED;

	int16_t Temp = 0;
	
	Temp = (ByteBuf[0] << 8) | ByteBuf[1];
	AccX = Temp / 16384.0;

	Temp = (ByteBuf[2] << 8) | ByteBuf[3];
	AccY = Temp / 16384.0;
	
	Temp = (ByteBuf[4] << 8) | ByteBuf[5];
	AccZ = Temp / 16384.0;

	AccAngX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccAngXOffset;
	AccAngY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccAngYOffset;

	Temp = (ByteBuf[8] << 8) | ByteBuf[9];
	GyroX = Temp / 131.0;

	Temp = (ByteBuf[10] << 8) | ByteBuf[11];
	GyroY = Temp / 131.0;

	Temp = (ByteBuf[12] << 8) | ByteBuf[13];
	GyroZ = Temp / 131.0;

	GyroX -= GyroXOffset;
	GyroY -= GyroYOffset;
	GyroZ -= GyroZOffset;
	
	TimeElapsed = ((0.002048 * TC0_OVF_Cnt) + (TCNT0 * 0.000008F));
	
	GyroAngX = GyroX * TimeElapsed;
	GyroAngY = GyroY * TimeElapsed;
	
	TC0_OVF_Cnt = 0;
	TCNT0 = 0;

	TempFloat = AccAngX - ClampedAccAngX;

	if(TempFloat > ANGLE_CLAMP_VAL)					ClampedAccAngX += ANGLE_CLAMP_VAL;
	else if(TempFloat < (-1 * ANGLE_CLAMP_VAL))		ClampedAccAngX -= ANGLE_CLAMP_VAL;
	else											ClampedAccAngX = AccAngX;

	TempFloat = GyroAngX - ClampedGyroAngX;

	if(TempFloat > ANGLE_CLAMP_VAL)					ClampedGyroAngX += ANGLE_CLAMP_VAL;
	else if(TempFloat < (-1 * ANGLE_CLAMP_VAL))		ClampedGyroAngX -= ANGLE_CLAMP_VAL;
	else											ClampedGyroAngX = GyroAngX;

	FinalAngX = ((FinalAngX + ClampedGyroAngX) * COMP_FILTER_GYRO_GAIN) + (ClampedAccAngX * COMP_FILTER_ACC_GAIN);
	
	TempFloat = FinalAngX - ClampedFinalAngX;

	if(TempFloat > ANGLE_CLAMP_VAL)					ClampedFinalAngX += ANGLE_CLAMP_VAL;
	else if(TempFloat < (-1 * ANGLE_CLAMP_VAL))		ClampedFinalAngX -= ANGLE_CLAMP_VAL;
	else											ClampedFinalAngX = FinalAngX;
	
	/////////////////////////////////////////////////////////////////////////////////////////////
	
	TempFloat = AccAngY - ClampedAccAngY;

	if(TempFloat > ANGLE_CLAMP_VAL)					ClampedAccAngY += ANGLE_CLAMP_VAL;
	else if(TempFloat < (-1 * ANGLE_CLAMP_VAL))		ClampedAccAngY -= ANGLE_CLAMP_VAL;
	else											ClampedAccAngY = AccAngY;

	TempFloat = GyroAngY - ClampedGyroAngY;

	if(TempFloat > ANGLE_CLAMP_VAL)					ClampedGyroAngY += ANGLE_CLAMP_VAL;
	else if(TempFloat < (-1 * ANGLE_CLAMP_VAL))		ClampedGyroAngY -= ANGLE_CLAMP_VAL;
	else											ClampedGyroAngY = GyroAngY;

	FinalAngY = ((FinalAngY + ClampedGyroAngY) * COMP_FILTER_GYRO_GAIN) + (ClampedAccAngY * COMP_FILTER_ACC_GAIN);
	
	TempFloat = FinalAngY - ClampedFinalAngY;

	if(TempFloat > ANGLE_CLAMP_VAL)					ClampedFinalAngY += ANGLE_CLAMP_VAL;
	else if(TempFloat < (-1 * ANGLE_CLAMP_VAL))		ClampedFinalAngY -= ANGLE_CLAMP_VAL;
	else											ClampedFinalAngY = FinalAngY;
	
	//Send_VUART_Data(FinalAngX * 100, FilteredFinalAng * 100);
	return NO_ERROR;
}

void AutoCalibrate(void)
{
	int16_t Temp = 0;
	float AccAngXTot = 0, AccAngYTot = 0, GyroXTot = 0, GyroYTot = 0, GyroZTot = 0;
	float FinalAngXTot = 0, FinalAngYTot = 0;

	for(uint8_t Sample = 0; Sample < CALIBRATION_SAMPLE_NOS; Sample++)
	{
		if(ReadMPUData(AccStAddr, 14) == ERROR_OCCURED) 
		{
			Sample--;
			continue;
		}
		
		Temp = (ByteBuf[0] << 8) | ByteBuf[1];
		AccX = Temp / 16384.0;
			
		Temp = (ByteBuf[2] << 8) | ByteBuf[3];
		AccY = Temp / 16384.0;
			
		Temp = (ByteBuf[4] << 8) | ByteBuf[5];
		AccZ = Temp / 16384.0;
		
		AccAngX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
		AccAngY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
		
		AccAngXTot += AccAngX;
		AccAngYTot += AccAngY;
		
		Temp = (ByteBuf[8] << 8) | ByteBuf[9];
		GyroX = Temp / 131.0;

		Temp = (ByteBuf[10] << 8) | ByteBuf[11];
		GyroY = Temp / 131.0;

		Temp = (ByteBuf[12] << 8) | ByteBuf[13];
		GyroZ = Temp / 131.0;
		
		GyroXTot += GyroX;
		GyroYTot += GyroY;
		GyroZTot += GyroZ;
	}
	
	AccAngXOffset = AccAngXTot / CALIBRATION_SAMPLE_NOS;
	AccAngYOffset = AccAngYTot / CALIBRATION_SAMPLE_NOS;
	
	GyroXOffset = GyroXTot / CALIBRATION_SAMPLE_NOS;
	GyroYOffset = GyroYTot / CALIBRATION_SAMPLE_NOS;
	GyroZOffset = GyroZTot / CALIBRATION_SAMPLE_NOS;
	
	for(uint8_t Sample = 0; Sample < CALIBRATION_SAMPLE_NOS; Sample++)
	{
		if(ReadMPUData(AccStAddr, 14) == ERROR_OCCURED) 
		{
			Sample--;
			continue;
		}
		
		Temp = (ByteBuf[0] << 8) | ByteBuf[1];
		AccX = Temp / 16384.0;
		
		Temp = (ByteBuf[2] << 8) | ByteBuf[3];
		AccY = Temp / 16384.0;
		
		Temp = (ByteBuf[4] << 8) | ByteBuf[5];
		AccZ = Temp / 16384.0;
		
		AccAngX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccAngXOffset;
		AccAngY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccAngYOffset;
		
		Temp = (ByteBuf[8] << 8) | ByteBuf[9];
		GyroX = Temp / 131.0;

		Temp = (ByteBuf[10] << 8) | ByteBuf[11];
		GyroY = Temp / 131.0;

		Temp = (ByteBuf[12] << 8) | ByteBuf[13];
		GyroZ = Temp / 131.0;
		
		GyroX -= GyroXOffset;
		GyroY -= GyroYOffset;
		GyroZ -= GyroZOffset;
		
		GyroAngX = GyroX * ((0.002048 * TC0_OVF_Cnt) + (TCNT0 * 0.000008F));
		GyroAngY = GyroY * ((0.002048 * TC0_OVF_Cnt) + (TCNT0 * 0.000008F));
		
		TC0_OVF_Cnt = 0;
		TCNT0 = 0;
		
		FinalAngX = ((FinalAngX + GyroAngX) * COMP_FILTER_ACC_GAIN) + (AccAngX * COMP_FILTER_GYRO_GAIN);
		FinalAngY = ((FinalAngY + GyroAngY) * COMP_FILTER_ACC_GAIN) + (AccAngY * COMP_FILTER_GYRO_GAIN);
		
		FinalAngXTot += FinalAngX;
		FinalAngYTot += FinalAngY;
	}
	
	FinalAngXOffset = FinalAngXTot / CALIBRATION_SAMPLE_NOS;
	FinalAngYOffset = FinalAngYTot / CALIBRATION_SAMPLE_NOS;
}

void Delay_ms(uint16_t NoOfms)
{
	//Max Delay is 60000 mS.
	
	DelayVal = NoOfms;
	
	while(DelayVal--)
	for(Delay = 0; Delay < 468; Delay++);
}

void Send_VUART_Data(int16_t AngX, int16_t AngY)
{	
	VUART_Data_Bytes[0] = AngX;
	VUART_Data_Bytes[1] = AngX >>8 ;
	
	VUART_Data_Bytes[2] = AngY;
	VUART_Data_Bytes[3] = AngY >> 8;
	
	VUART_ByteIndx = 0;
	VUART_Byte = 0x0200 | (VUART_Data_Bytes[0] << 1);
	VUART_BitIndx = 0;
	Max_No_Of_Bytes = 4;
	StartTimer1();
	VUART_TXD_Status = VUART_TXD_ACTIVE;
}

void Send_VUART_Byte(uint8_t Byte)
{
	//VUART = Virtual UART
	VUART_Byte = 0x0200 | (Byte << 1);
	VUART_BitIndx = 0;
	Max_No_Of_Bytes = 1;
	StartTimer1();
}

ISR(TIMER1_COMPA_vect)
{
	if(VUART_TXD_Status == VUART_TXD_ACTIVE)
	{
		if((VUART_Byte >> VUART_BitIndx) & 0x01)
			PORTD |= 0b00100000;
		else
			PORTD &= 0b11011111;
	
		VUART_BitIndx++;
	
		if(VUART_BitIndx == 10)
		{
			VUART_ByteIndx++;
		
			if(VUART_ByteIndx == Max_No_Of_Bytes) 
			{
				StopTimer1();
				VUART_TXD_Status = VUART_TXD_INACTIVE;
			}
			else
			{
				VUART_Byte = 0x0200 | (VUART_Data_Bytes[VUART_ByteIndx] << 1);
				VUART_BitIndx = 0;
			}
		}
	}
	/*
	if(((PORTD >> 6) & 0x01) == 0)
	{
		ConsecutiveHighSampleCnt = 0;
		
		if(VUART_RXD_Status == RXD_IDLE) 
		{	
			VUART_RXD_Status = RXD_CHECK_STRT_BIT;
			
			ConsecutiveEqualSampleCnt = 1;
		}
		else if(VUART_RXD_Status == RXD_CHECK_STRT_BIT)
		{
			ConsecutiveEqualSampleCnt++;
			if(ConsecutiveEqualSampleCnt == CONSEC_SAMPLE_FOR_STRT_BIT) 
			{
				VUART_RXD_Status = RXD_STRT_BIT_RECEIVED;
				ConsecutiveEqualSampleCnt = 0;
				ConsecutiveHighSampleCnt = 0;
			}
		}
		else if(VUART_RXD_Status == RXD_STRT_BIT_RECEIVED)
		{
			if(ConsecutiveHighSampleCnt > 0)
			{
				if(ConsecutiveHighSampleCnt > CONSEC_SAMPLE_1HIGHBIT)
					DeltaStatONFreq = STAT_ON_FREQ_INC_DEC;
				else if(ConsecutiveHighSampleCnt > CONSEC_SAMPLE_2HIGHBIT)
					DeltaStatONFreq = -STAT_ON_FREQ_INC_DEC;
				else if(ConsecutiveHighSampleCnt > CONSEC_SAMPLE_3HIGHBIT)
					DeltaStatONFreq = 0;
			}
		}
	}
	else if(VUART_RXD_Status == RXD_STRT_BIT_RECEIVED)
	{
		ConsecutiveHighSampleCnt++;
	}
	*/
}

ISR(TIMER0_OVF_vect)
{
	TC0_OVF_Cnt++;
}

ISR(USART_RX_vect)
{
	AccelarateMotorsAndRoll();
}

void AccelarateMotors(void)
{
	uint8_t JoyStickData = UDR0;
	int8_t SpeedIncrFact = 0;
	
	if((JoyStickData & 0x03) == 0) SpeedIncrFact = -1;
	else if((JoyStickData & 0x03) == 2) SpeedIncrFact = 1;
	
	if((JoyStickData & 0x30) == 0) SpeedIncrFact = -1;
	else if((JoyStickData & 0x30) == 2) SpeedIncrFact = 1;
	
	for(uint8_t ESC_ID = 0; ESC_ID < 4; ESC_ID++)
	{
		StatOnFreq_ToSend[ESC_ID] += STAT_ON_FREQ_INC_DEC * SpeedIncrFact;
		
		if(StatOnFreq_ToSend[ESC_ID] < LOWEST_STAT_ON_FREQ)
		StatOnFreq_ToSend[ESC_ID] = LOWEST_STAT_ON_FREQ;
		else if(StatOnFreq_ToSend[ESC_ID] > HIGHEST_STAT_ON_FREQ)
		StatOnFreq_ToSend[ESC_ID] = HIGHEST_STAT_ON_FREQ;
		
		ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)(ESC_MCKFreq / (StatOnFreq_ToSend[ESC_ID] / 10) / 6);
		
		DutyCycle = (((StatOnFreq_ToSend[ESC_ID] / 10) - 20) * 0.1) + 20;

		if(DutyCycle < 20)			DutyCycle = 20;
		else if(DutyCycle > 90)		DutyCycle = 90;

		ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
	}
	
	SendESCData(&ESC_Data[0]);
	Delay_ms(5);
}

void AccelarateMotorsAndRollFixed(void)
{
	uint8_t JoyStickData = UDR0;
	int8_t SpeedIncrFact = 0;
	int8_t RollRightOrLeft = 0;
	
	if((JoyStickData & 0x03) == 0) SpeedIncrFact = -1;
	else if((JoyStickData & 0x03) == 2) SpeedIncrFact = 1;
	
	if((JoyStickData & 0x30) == 0) RollRightOrLeft = 1;
	else if((JoyStickData & 0x30) == 0b100000) RollRightOrLeft = 2;
	
	if(RollRightOrLeft == 1)
	{
		StatOnFreq_ToSend[0] += 2;
		StatOnFreq_ToSend[1] -= 2;
		StatOnFreq_ToSend[2] -= 2;
		StatOnFreq_ToSend[3] += 2;
	}
	else if(RollRightOrLeft == 2)
	{
		StatOnFreq_ToSend[0] -= 2;
		StatOnFreq_ToSend[1] += 2;
		StatOnFreq_ToSend[2] += 2;
		StatOnFreq_ToSend[3] -= 2;
	}
	
	for(uint8_t ESC_ID = 0; ESC_ID < 4; ESC_ID++)
	{
		StatOnFreq_ToSend[ESC_ID] += STAT_ON_FREQ_INC_DEC * SpeedIncrFact;
		
		if(StatOnFreq_ToSend[ESC_ID] < LOWEST_STAT_ON_FREQ)
		StatOnFreq_ToSend[ESC_ID] = LOWEST_STAT_ON_FREQ;
		else if(StatOnFreq_ToSend[ESC_ID] > HIGHEST_STAT_ON_FREQ)
		StatOnFreq_ToSend[ESC_ID] = HIGHEST_STAT_ON_FREQ;
		
		ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)(ESC_MCKFreq / (StatOnFreq_ToSend[ESC_ID] / 10) / 6);
		
		DutyCycle = (((StatOnFreq_ToSend[ESC_ID] / 10) - 20) * 0.1) + 20;

		if(DutyCycle < 20)			DutyCycle = 20;
		else if(DutyCycle > 90)		DutyCycle = 90;

		ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
	}
	
	SendESCData(&ESC_Data[0]);
	Delay_ms(5);
}

void AccelarateMotorsAndRoll(void)
{
	uint8_t JoyStickData = UDR0;
	int8_t SpeedIncrFact = 0;
	int8_t RollRightOrLeft = 0;
	
	if((JoyStickData & 0x03) == 0) SpeedIncrFact = -1;
	else if((JoyStickData & 0x03) == 2) SpeedIncrFact = 1;
	
	if((JoyStickData & 0x30) == 0) RollRightOrLeft = 1;
	else if((JoyStickData & 0x30) == 0b100000) RollRightOrLeft = 2;
	
	if(RollRightOrLeft == 1) 
	{
		StatOnFreq_ToSend[0] += 2;
		StatOnFreq_ToSend[1] -= 2;
		StatOnFreq_ToSend[2] -= 2;
		StatOnFreq_ToSend[3] += 2;
	}
	else if(RollRightOrLeft == 2)
	{
		StatOnFreq_ToSend[0] -= 2;
		StatOnFreq_ToSend[1] += 2;
		StatOnFreq_ToSend[2] += 2;
		StatOnFreq_ToSend[3] -= 2;
	}
	
	for(uint8_t ESC_ID = 0; ESC_ID < 4; ESC_ID++)
	{
		StatOnFreq_ToSend[ESC_ID] += STAT_ON_FREQ_INC_DEC * SpeedIncrFact;
		
		if(StatOnFreq_ToSend[ESC_ID] < LOWEST_STAT_ON_FREQ)
		StatOnFreq_ToSend[ESC_ID] = LOWEST_STAT_ON_FREQ;
		else if(StatOnFreq_ToSend[ESC_ID] > HIGHEST_STAT_ON_FREQ)
		StatOnFreq_ToSend[ESC_ID] = HIGHEST_STAT_ON_FREQ;
		
		ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)(ESC_MCKFreq / (StatOnFreq_ToSend[ESC_ID] / 10) / 6);
		
		DutyCycle = (((StatOnFreq_ToSend[ESC_ID] / 10) - 20) * 0.1) + 20;

		if(DutyCycle < 20)			DutyCycle = 20;
		else if(DutyCycle > 90)		DutyCycle = 90;

		ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
	}
	
	SendESCData(&ESC_Data[0]);
	Delay_ms(5);
}

void VirtualUARTInit(void)
{
	//Use PD5 as Virtual UART TXD
	DDRD |= 0b00100000;
	DDRD &= 0b10111111;
	
	PORTD |= 0b00100000;
	
	Timer1Init_for_VUART();
	
	StartTimer1();
}

void PID(void)
{
	TWIErrorOccurd = NO_ERROR;
	
	if(GetMPUOrientation() == ERROR_OCCURED) return;
	
	PrevErrorAngX = ErrorAngX;
	ErrorAngX = DesiredAngX - FinalAngX;
	
	PrevErrorAngY = ErrorAngY;
	ErrorAngY = DesiredAngY - FinalAngY;
	
	//=======================================================================================================================
	
	P_Val_X = ErrorAngX * Kp_X;
	
	if((-4 < ErrorAngX) && (ErrorAngX < 4)) I_Val_X += (ErrorAngX * Ki_X);
	
	D_Val_X = Kd_X * (ErrorAngX - PrevErrorAngX);
	
	PrevPID_Val_X = PID_Val_X;
	PID_Val_X = P_Val_X + I_Val_X + D_Val_X;
	
	if(PID_Val_X > MAX_PID_VAL) PID_Val_X = MAX_PID_VAL;
	else if(PID_Val_X < (-MAX_PID_VAL)) PID_Val_X = (-MAX_PID_VAL);
	
	//=======================================================================================================================
	
	P_Val_Y = ErrorAngY * Kp_Y;
	
	if((-4 < ErrorAngY) && (ErrorAngY < 4)) I_Val_Y += (ErrorAngY * Ki_Y);
	
	D_Val_Y = Kd_Y * (ErrorAngY - PrevErrorAngY);
	
	PrevPID_Val_Y = PID_Val_Y;
	PID_Val_Y = P_Val_Y + I_Val_Y + D_Val_Y;
	
	if(PID_Val_Y > MAX_PID_VAL) PID_Val_Y = MAX_PID_VAL;
	else if(PID_Val_Y < (-MAX_PID_VAL)) PID_Val_Y = (-MAX_PID_VAL);
	
	//Send_VUART_Data(PID_Val_X, PID_Val_Y);
	//Delay_ms(10);
	
	//Send_VUART_Data(StatOnFreq_ToSend[1], FinalAngX * 100);
	//Delay_ms(10);
	//Send_VUART_Data(D_Val_X, PID_Val_X);
	//Delay_ms(10);
	//Send_VUART_Data(D_Val_X, ErrorAngX * 100);
	//if((PID_Val_X == PrevPID_Val_X) && ((PID_Val_Y == PrevPID_Val_Y))) Delay_ms(5);
	
	//Send_VUART_Data(StatOnFreq_ToSend[1], StatOnFreq_ToSend[3]);
	//Delay_ms(5);
	
	//if((PID_Val_X != PrevPID_Val_X) || (PID_Val_Y != PrevPID_Val_Y)) ExecuteStepwisePID();
	
	ExecutePID();
}

void ExecutePID(void)
{
	//X-Axis ================================================================================================
	int16_t Diff = PID_Val_X + NORMAL_STAT_ON_FREQ - StatOnFreq_ToSend[3];
	
	if(Diff > MAX_PID_STEP_VAL)
	{
		StatOnFreq_ToSend[1] -= MAX_PID_STEP_VAL;
		StatOnFreq_ToSend[3] += MAX_PID_STEP_VAL;
	}
	else if(Diff < -MAX_PID_STEP_VAL)
	{
		StatOnFreq_ToSend[1] += MAX_PID_STEP_VAL;
		StatOnFreq_ToSend[3] -= MAX_PID_STEP_VAL;
	}
	else
	{
		StatOnFreq_ToSend[1] -= Diff;
		StatOnFreq_ToSend[3] += Diff;
	}
	
	//Y-Axis ================================================================================================
	Diff = PID_Val_Y + NORMAL_STAT_ON_FREQ - StatOnFreq_ToSend[2];
	
	if(Diff > MAX_PID_STEP_VAL)
	{
		StatOnFreq_ToSend[0] -= MAX_PID_STEP_VAL;
		StatOnFreq_ToSend[2] += MAX_PID_STEP_VAL;
	}
	else if(Diff < -MAX_PID_STEP_VAL)
	{
		StatOnFreq_ToSend[0] += MAX_PID_STEP_VAL;
		StatOnFreq_ToSend[2] -= MAX_PID_STEP_VAL;
	}
	else
	{
		StatOnFreq_ToSend[0] -= Diff;
		StatOnFreq_ToSend[2] += Diff;
	}
	
	for(uint8_t Indx = 0; Indx < 4; Indx++)
	{
		if(StatOnFreq_ToSend[Indx] < (NORMAL_STAT_ON_FREQ - MAX_PID_VAL)) StatOnFreq_ToSend[Indx] = NORMAL_STAT_ON_FREQ - MAX_PID_VAL;
		else if(StatOnFreq_ToSend[Indx] > HIGHEST_STAT_ON_FREQ) StatOnFreq_ToSend[Indx] = HIGHEST_STAT_ON_FREQ;
	}
	
	for(uint8_t ESC_ID = 0; ESC_ID < 4; ESC_ID++)
	{
		ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)((float)ESC_MCKFreq / (StatOnFreq_ToSend[ESC_ID] / 10.0) / 6);
		
		//if(ESC_ID == 1) DutyCycle = (((StatOnFreq_ToSend[ESC_ID] / 10) - 20) * 0.1) + 30;
		//else 
		DutyCycle = (((StatOnFreq_ToSend[ESC_ID] / 10) - 20) * 0.1) + 25;

		if(DutyCycle < 20)			DutyCycle = 20;
		else if(DutyCycle > 90)		DutyCycle = 90;

		ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
	}

	Send_VUART_Data(StatOnFreq_ToSend[2], StatOnFreq_ToSend[0]);
	Delay_ms(10);
	//Send_VUART_Data(StatOnFreq_ToSend[3], SendValCnt);
	//Delay_ms(10);
	//Send_VUART_Data(FinalAngX * 100, FinalAngY * 100);
	//Delay_ms(10);
	
	SendValCnt++;
	
	SendESCData(&ESC_Data[0]);
	Delay_ms(5);
}

void TwoMotorSpeedControl(void)
{
	uint8_t JoyStickData = UDR0;
	int8_t RM_SpeedIncrFact = 0;
	int8_t LM_SpeedIncrFact = 0;
	
	if((JoyStickData & 0x03) == 0)
	LM_SpeedIncrFact = -1;
	else if((JoyStickData & 0x03) == 2)
	LM_SpeedIncrFact = 1;
	
	if((JoyStickData & 0x30) == 0)
	RM_SpeedIncrFact = -1;
	else if((JoyStickData & 0x30) == 0b100000)
	RM_SpeedIncrFact = 1;
	
	for(uint8_t ESC_ID = 0; ESC_ID < 4; ESC_ID++)
	{
		if(ESC_ID == 3) StatOnFreq_ToSend[3] += STAT_ON_FREQ_INC_DEC * RM_SpeedIncrFact;
		if(ESC_ID == 1) StatOnFreq_ToSend[1] += STAT_ON_FREQ_INC_DEC * LM_SpeedIncrFact;
		
		if(StatOnFreq_ToSend[ESC_ID] < LOWEST_STAT_ON_FREQ)
		StatOnFreq_ToSend[ESC_ID] = LOWEST_STAT_ON_FREQ;
		else if(StatOnFreq_ToSend[ESC_ID] > HIGHEST_STAT_ON_FREQ)
		{
			StatOnFreq_ToSend[ESC_ID] = HIGHEST_STAT_ON_FREQ;
			if(StatOnFreq_ToSend[1] < 3500) StatOnFreq_ToSend[1] = 3000;
			if(StatOnFreq_ToSend[3] < 3500) StatOnFreq_ToSend[3] = 3000;
		}
		
		ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)(ESC_MCKFreq / (StatOnFreq_ToSend[ESC_ID] / 10) / 6);
		
		DutyCycle = (((StatOnFreq_ToSend[ESC_ID] / 10) - 20) * 0.1) + 20;

		if(DutyCycle < 20)			DutyCycle = 20;
		else if(DutyCycle > 90)		DutyCycle = 90;

		ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
	}
	
	//Send_VUART_Data(SpeedIncrFact, 0);
	//Delay_ms(5);
	
	SendESCData(&ESC_Data[0]);
	Delay_ms(5);
}

void ExecuteStepwisePID(void)
{
	PID_Diff_X = PID_Val_X - PrevPID_Val_X;
	
	int8_t SignCoefficient_X = -1;
	if(PID_Diff_X > 0) SignCoefficient_X = 1;
	
	float NoOfStepsFloat_X = PID_Diff_X / MAX_PID_STEP_VAL;
	if(NoOfStepsFloat_X < 0) NoOfStepsFloat_X = -NoOfStepsFloat_X;

	uint8_t NoOfSteps_X = NoOfStepsFloat_X;
	NoOfSteps_X++;
	
	////////////////////////////////////////////////////////////////////////////////////////////
	
	PID_Diff_Y = PID_Val_Y - PrevPID_Val_Y;
	
	int8_t SignCoefficient_Y = -1;
	if(PID_Diff_Y > 0) SignCoefficient_Y = 1;
	
	float NoOfStepsFloat_Y = PID_Diff_Y / MAX_PID_STEP_VAL;
	if(NoOfStepsFloat_Y < 0) NoOfStepsFloat_Y = -NoOfStepsFloat_Y;

	uint8_t NoOfSteps_Y = NoOfStepsFloat_Y;
	NoOfSteps_Y++;
	
	uint8_t MaxNoOfSteps = NoOfSteps_X;
	if(NoOfSteps_Y > NoOfSteps_X) MaxNoOfSteps = NoOfSteps_Y;
	
	for(uint8_t Step = 0; Step < MaxNoOfSteps; Step++)
	{
		if(Step < NoOfSteps_X)
		{
			if((PID_Diff_X >= MAX_PID_STEP_VAL) || (PID_Diff_X <= -MAX_PID_STEP_VAL))
			{
				StatOnFreq_ToSend[1] -= MAX_PID_STEP_VAL * SignCoefficient_X;
				StatOnFreq_ToSend[3] += MAX_PID_STEP_VAL * SignCoefficient_X;
			}
			else
			{
				StatOnFreq_ToSend[1] -= PID_Diff_X;
				StatOnFreq_ToSend[3] += PID_Diff_X;
			}
			
			if(PID_Diff_X > 0) PID_Diff_X -= MAX_PID_STEP_VAL;
			else PID_Diff_X += MAX_PID_STEP_VAL;
		}
		
		if(Step < NoOfSteps_Y)
		{
			if((PID_Diff_Y >= MAX_PID_STEP_VAL) || (PID_Diff_Y <= -MAX_PID_STEP_VAL))
			{
				StatOnFreq_ToSend[0] -= MAX_PID_STEP_VAL * SignCoefficient_Y;
				StatOnFreq_ToSend[2] += MAX_PID_STEP_VAL * SignCoefficient_Y;
			}
			else
			{
				StatOnFreq_ToSend[0] -= PID_Diff_Y;
				StatOnFreq_ToSend[2] += PID_Diff_Y;
			}
			
			if(PID_Diff_Y > 0) PID_Diff_Y -= MAX_PID_STEP_VAL;
			else PID_Diff_Y += MAX_PID_STEP_VAL;
		}
		
		for(uint8_t Indx = 0; Indx < 4; Indx++)
		{
			if(StatOnFreq_ToSend[Indx] < LOWEST_STAT_ON_FREQ) StatOnFreq_ToSend[Indx] = LOWEST_STAT_ON_FREQ;
			else if(StatOnFreq_ToSend[Indx] > HIGHEST_STAT_ON_FREQ) StatOnFreq_ToSend[Indx] = HIGHEST_STAT_ON_FREQ;
		}
		
		for(uint8_t ESC_ID = 0; ESC_ID < 4; ESC_ID++)
		{
			ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)((float)ESC_MCKFreq / (StatOnFreq_ToSend[ESC_ID] / 10.0) / 6);
			
			DutyCycle = (((StatOnFreq_ToSend[ESC_ID] / 10) - 20) * 0.1) + 20;

			if(DutyCycle < 20)			DutyCycle = 20;
			else if(DutyCycle > 90)		DutyCycle = 90;

			ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
		}

		Send_VUART_Data(StatOnFreq_ToSend[0], StatOnFreq_ToSend[2]);
		Delay_ms(10);
		
		Send_VUART_Data(StatOnFreq_ToSend[0], StatOnFreq_ToSend[2]);
		Delay_ms(10);
		
		//Send_VUART_Data(1234, 1234);
		//Delay_ms(10);
		
		SendValCnt++;
		//Delay_ms(5);
		
		SendESCData(&ESC_Data[0]);
		Delay_ms(5);
	}
}

void RC_ReceiverInit(void)
{	
	StatOnFreq_ToSend[0] = LOWEST_STAT_ON_FREQ;
	StatOnFreq_ToSend[1] = LOWEST_STAT_ON_FREQ;
	StatOnFreq_ToSend[2] = LOWEST_STAT_ON_FREQ;
	StatOnFreq_ToSend[3] = LOWEST_STAT_ON_FREQ;
}

void AccelerateFromRemote(void)
{
	uint8_t Data = 0;
	uint8_t JoyStickData = 0;
	uint8_t JoyStickCheckData = 0;
	int8_t SpeedIncrFact = 1;	
	
	if(Data == Receiver_STRT_BYTE)
	{	
		if(JoyStickData == JoyStickCheckData)
		{
			if((JoyStickData & 0x03) == JoyStick_REVERSE)
				SpeedIncrFact = -1;
			
			for(uint8_t ESC_ID = 0; ESC_ID < 4; ESC_ID++)
			{
				StatOnFreq_ToSend[ESC_ID] += STAT_ON_FREQ_INC_DEC * SpeedIncrFact;
				
				ESC_Data[ESC_ID].OCR1A_VAl_forStatONFreq = (uint16_t)(ESC_MCKFreq / (StatOnFreq_ToSend[ESC_ID] / 10) / 6);
				
				DutyCycle = (((StatOnFreq_ToSend[ESC_ID] / 10) - 20) * 0.1) + 20;

				if(DutyCycle < 20)			DutyCycle = 20;
				else if(DutyCycle > 90)		DutyCycle = 90;

				ESC_Data[ESC_ID].OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
			}
			
			Send_VUART_Data(StatOnFreq_ToSend[1], 0);
			Delay_ms(10);
			
			SendESCData(&ESC_Data[0]);
			Delay_ms(5);
		}
	}
}

int main(void)
{
	//Enable Global Interrupts
	sei();
	
	//Delay for propeller startup
	Delay_ms(5000);
	
	//Set PD(7,6) as output
	DDRD |= 0b11000000;
	
	MPU_Init();
	USARTInit();
	VirtualUARTInit();
	StatOnFreq = 21;
	DeltaStatONFreq = 0;
	
    TWIErrorOccurd = NO_ERROR;

	DisableTWIInterrupt();
	
	//Delay for Calibration, Keep the drone stable during this time.
	Delay_ms(5000);
	
	AutoCalibrate();
	
	StatOnFreq_ToSend[0] = LOWEST_STAT_ON_FREQ;
	StatOnFreq_ToSend[1] = LOWEST_STAT_ON_FREQ;
	StatOnFreq_ToSend[2] = LOWEST_STAT_ON_FREQ;
	StatOnFreq_ToSend[3] = LOWEST_STAT_ON_FREQ;
	
	//AcclRetarESC(LOWEST_STAT_ON_FREQ, NORMAL_STAT_ON_FREQ + 250);
	//AcclRetarESC(LOWEST_STAT_ON_FREQ, NORMAL_STAT_ON_FREQ);
	
	//StatOnFreq_ToSend[0] = NORMAL_STAT_ON_FREQ;
	//StatOnFreq_ToSend[1] = NORMAL_STAT_ON_FREQ;
	//StatOnFreq_ToSend[2] = NORMAL_STAT_ON_FREQ;
	//StatOnFreq_ToSend[3] = NORMAL_STAT_ON_FREQ;
	
	EnableRXCompleteInterrupt();
	
	while (1)
    {
		PID();
		
		//Send_VUART_Data(FinalAngX * 100, FinalAngY * 100);
		//Delay_ms(10);
    }
}