/*
	* Sensorless_ESC.c
	*
	* Author : Rhugved Chaudhari
	*/ 

#include<avr/io.h>
#include<avr/interrupt.h>

// PWM for MOSFET High Sides is configured on the Pins PD3, PD5, PD6
// Signals for Low Sides are configured on the Pins PD2, PD4, PD7

#define MCKFreq 8000000L // Master Clock Frequency
#define USARTBAUD 9600 // Baud rate of USART Communication
#define UBRR0Val (MCKFreq / 16 / USARTBAUD) - 1 // Baud Rate registers value
#define NO_OF_RECEIVE_BYTES 6
#define LAST_RECEIVE_BYTE_INDX (NO_OF_RECEIVE_BYTES - 1)
#define ESC_ID 1

#define USART_RECEIVER_START_BYTE (uint8_t) 253

#define NOT_RECEIVING 0
#define RECEIVING_STARTED 1
#define RECEIVING_DONE 2

#define OCR02AB_BYTE_INDX ESC_ID + 1

#define MAX_DTY_CYCLE 80
#define OCR02AB_DTY_REG_MULTIPLIER 2.50F
#define MAX_OCR02B_BYTE_VAL MAX_DTY_CYCLE * OCR02AB_DTY_REG_MULTIPLIER

// Recommend Max Freq = 160kHz
// Recommend Min Freq = 65Hz
//#define PWMFreq 15625 //In Hz

#define UHS 0 // UHS - PD3
#define VHS 1 // VHS - PD5
#define WHS 2 // WHS - PD6

#define ULS (uint8_t)0x04 // ULS - PD2
#define VLS (uint8_t)0x10 // VLS - PD4
#define WLS (uint8_t)0x80 // WLS - PD7

// U - 1 Red-CCW	1 Red-CW	1 Black-CCW	1 Black-CW	2 Red		2 Red-CW
// V - 2 Red		1 Black		1 Red		2 Red		1 Black		1 Red
// W - 1 Black		2 Red		2 Red		1 Red		1 Red		1 Black

//  PWM Prescalers = {1, 8, 64, 256, 1024}
// Corresponding N = {1, 2, 3,  4,   5}
#define PWMPrescalerIndxTC0 1 // PWM Freq = MCKFreq / (N * 256)

//  PWM Prescalers = {1, 8, 32, 64, 128, 256, 1024}
// Corresponding N = {1, 2, 3,  4,  5,   6,   7}
#define PWMPrescalerIndxTC2 1 // PWM Freq = MCKFreq / (N * 256)

//      Prescalers = {1, 8, 64, 256, 1024}
// Corresponding N = {1, 2, 3,  4,   5}
#define TC1PrescalerIndx 1 

#define PrescaledFreq MCKFreq / PWMPrescalerIndxTC0

#define CHECKSUM_OFFSET 0x50

#define MAX_PID_STEP_VAL 40

#define STALL_PCINT_FREQ 15000
#define TC1_MinCntThres (uint16_t)(MCKFreq / STALL_PCINT_FREQ)

#define NO_STALL 1
#define MOTOR_STALLED 2
#define STALL_RECOVERY_DTY_CYCLE 60
#define STALL_RECOVERY_OCR02AB_VAL (uint8_t)(2.55F * STALL_RECOVERY_DTY_CYCLE)
#define STARTUP_FREQ 21
#define STAT_STEP_SWITCH_TIME_ms (uint16_t)((1.0 / STARTUP_FREQ) * 1000.0 / 6.0)

#define SET 1
#define NOT_SET 2

#define YES 1
#define NO 2

#define START 1
#define STOP 2

#define HALL_SENS_SWITCH_START_FREQ 300
#define HALL_SENS_SWITCH_TCNT1_THRES (uint16_t)(MCKFreq / (HALL_SENS_SWITCH_START_FREQ * 6))

//PC5 -> U Phase
//PC4 -> V Phase
//PC0 -> W Phase

#define VH_UL 0b00010000
#define WH_UL 0b00010001
#define WH_VL 0b00000001
#define UH_VL 0b00100001
#define UH_WL 0b00100000
#define VH_WL 0b00110000
			  
void PWMInit(void);
void EnablePWMClock(void);
void DisablePWMClock(void);
void ONHighSide(uint8_t HighSide);
void OFFHighSide(uint8_t HighSide);
void ONLowSide(uint8_t LowSide);
void OFFLowSide(uint8_t LowSide); 
void ActivateStator(uint8_t HighSide, uint8_t LowSide);
void USARTInit(void);
void HighSideCapacitorCharge(void);
void Delay_ms(uint16_t NoOfms);
void EnableDataRegEmptyInterrupt(void);
void DisableDataRegEmptyInterrupt(void);
void PhaseSwitchInterruptInit(void);
void StartTimer1(void);
void StopTimer1(void);
void MotorStartUp(void);

uint8_t StallRecovery_OCR02AB_VAL = (uint8_t)(2.55F * STALL_RECOVERY_DTY_CYCLE);

uint8_t TC0_OV_Flag = 0;
uint8_t IntrptCnt = 0;

//DutyCycle should be such that
// 1 <= DutyCycle <= 90
uint8_t DutyCycle = 30;
float StatorONFreq = 21; // Minimum value should be 21Hz

uint16_t CycleCnts = 2;
uint8_t Flag = 1;
uint8_t ConfigNo = 0;
uint16_t StatCycleCnt = 0;
uint16_t OCR1A_VAl_forStatONFreq;
uint8_t OCR02AB_Val_forDutyCycle;

volatile uint8_t USARTReceivedData = 0;
uint8_t ReceivedBytes[20];
uint8_t ReceivedBytesIndx = 0;
volatile uint8_t USART_Receiver_Status = NOT_RECEIVING;
uint8_t SendByteIndx = 0;

uint16_t TC1_OVF_Cnt = 0;
uint8_t IsStallDetected = MOTOR_STALLED;
uint8_t StallFreqDetectCnt = 0;

uint8_t Prev_TCNT1_SET_FLAG = NOT_SET;
uint16_t Prev_TCNT1 = 0, Dup_TCNT1 = 0;
int16_t TCNT1_Diff = 0;
uint8_t IsFalseSpike = 0;
uint8_t HallSensor;

uint8_t IsFakeSpike = 0;

uint8_t HighFreqCnt = 0;

uint8_t StartHallSensSwitch = STOP;
uint8_t HallSensStartFreqCnt = 0;

float New_StatOnFreq, Old_StatOnFreq;

void PWMInit(void)
{
	// Configure PWM on the Pins PD3, PD5, PD6
	// TC0 Configured for 2 PWM Channel on PD6(Channel A) and PD5 (Channel B)
	// TC2 Configured for 1 PWM Channel on PD3(Channel B)
	
	// Disable Power Reduction of TC(0,1,2) (Enable TC(0,1,2))
	PRR &= 0b11010111;
	
	/*
		* PWM Initialization Explanation TC0 & TC2
		
		* In this mode the Counter starts with the BOTTOM value(0x00) counting up steadily and
		overflows and resets to BOTTOM(0x00) when it reaches the "MAX" i.e. 0xFF
		
		* Each PWM channel waveform (Channel A & B) starts with LOGIC_LOW (0V) and switches to
		LOGIC_HIGH (5V) when the counter reaches the values in the respective Output Compare
		registers OCR1A and OCR1B
	*/

	// PWM Initialization TC0 - PD5(VHS) & PD6(WHS)
	{
		
		// Set TC0 in Fast PWM Mode (Mode 3) - Inverting Configuration(Active Low).
		// Clock Source not set yet - TC0/PWM not started yet.
		// Disable Channels A & B, the channels will be enabled when "ONHighSide(HighSide)" will be called
		//TCCR0A = 0b00000011;
		//TCCR0B = 0b00000000;

		TCCR0A = 0b00000001;
		TCCR0B = 0b00000000;
	
		if((DutyCycle >= 1) && (DutyCycle <= 90))
		{
			// Set Output Compare Register A with the Duty Cycle Value. - PWM Channel on PD6
			OCR0A = (uint8_t)(DutyCycle * 2.55); // OCR0A = (DutyCycle / 100) * 255
			// Set Output Compare Register B with the Duty Cycle Value. - PWM Channel on PD5
			OCR0B = (uint8_t)(DutyCycle * 2.55); // OCR0B = (DutyCycle / 100) * 255	
		}
		else
		{
			OCR0A = 10;
			OCR0B = 10;
		}
	}
	
	// Enable TC0 Overflow Interrupt
	//TIMSK0 |= 0b00000001;
	
	// PWM Initialization TC2 - PD3(UHS)
	{
		// Set TC2 in Fast PWM Mode (Mode 3) - Inverting Configuration(Active Low).
		// Clock Source not set yet - TC2/PWM not started yet.
		// Disable Channels A & B, the channel B will be enabled when "ONHighSide(HighSide)" will be called
		//TCCR2A = 0b00000011;
		//TCCR2B = 0b00000000;
	
		TCCR2A = 0b00000001;
		TCCR2B = 0b00000000;
	
		if((DutyCycle >= 1) && (DutyCycle <= 90))
		{
			// Set Output Compare Register B with the Duty Cycle Value. - PWM Channel on PD3
			OCR2B = (uint8_t)(DutyCycle * 2.55); // OCR0B = (DutyCycle / 100) * 255
		}
		else
		{
			OCR2B = 10;
		}
	}
}

void EnableClocks(void)
{
	//Set Clock Source with the Prescaler to start the PWM.
	TCCR0B |= PWMPrescalerIndxTC0;
	TCCR2B |= PWMPrescalerIndxTC2;
	
	//Set Clock Source with the Prescaler to start TC1
	TCCR1B |= TC1PrescalerIndx;
}

void DisablePWMClock(void)
{
	// Clock disconnected - TC0 & TC2 stopped - PWM stopped
	TCCR0B &= 0b11111000;
	TCCR2B &= 0b11111000;
	
	// Clock disconnected - TC1 stopped
	TCCR1B &= 0b11111000;
}

void ONHighSide(uint8_t HighSide)
{
	switch(HighSide)
	{
		case UHS: { TCCR2A |= 0b00110000; break; }
		case VHS: { TCCR0A |= 0b00110000; break; }
		case WHS: { TCCR0A |= 0b11000000; break; }
	}
}

void OFFHighSide(uint8_t HighSide)
{
	switch(HighSide)
	{
		case UHS: { TCCR2A &= 0b11001111; PORTD |= 0b00001000; break; }
		case VHS: { TCCR0A &= 0b11001111; PORTD |= 0b00100000; break; }
		case WHS: { TCCR0A &= 0b00111111; PORTD |= 0b01000000; break; }
	}
}

void ONLowSide(uint8_t LowSide)
{
	PORTD |= LowSide;
}

void OFFLowSide(uint8_t LowSide)
{
	PORTD &= ~LowSide;
}

void ActivateStator(uint8_t HighSide, uint8_t LowSide)
{
	if(
	((HighSide == UHS) && ((LowSide == VLS) || (LowSide == WLS))) ||
	((HighSide == VHS) && ((LowSide == WLS) || (LowSide == ULS))) ||
	((HighSide == WHS) && ((LowSide == ULS) || (LowSide == VLS)))
	)
	{
		ONLowSide(LowSide);
		ONHighSide(HighSide);
	}
}

ISR(TIMER1_OVF_vect, ISR_NOBLOCK)
{
	TC1_OVF_Cnt++;
}

ISR(USART_RX_vect)
{	
	USARTReceivedData = UDR0;
	sei();

	if(USART_Receiver_Status == RECEIVING_DONE)
		return;
	
	if(USARTReceivedData == USART_RECEIVER_START_BYTE)
	{
		
		ReceivedBytesIndx = 0;
		ReceivedBytes[ReceivedBytesIndx] = USARTReceivedData;
		USART_Receiver_Status = RECEIVING_STARTED;
	}
	else if(USART_Receiver_Status == RECEIVING_STARTED)
	{
		ReceivedBytesIndx++;
		if(ReceivedBytesIndx < NO_OF_RECEIVE_BYTES)
			ReceivedBytes[ReceivedBytesIndx] = USARTReceivedData;	
			
		if(ReceivedBytesIndx == LAST_RECEIVE_BYTE_INDX)
			USART_Receiver_Status = RECEIVING_DONE;
	}
}

void USARTInit(void)
{
	// Set the Baud rate by writing into the registers the baud rate Prescaler value
	UBRR0 = UBRR0Val;

	/* 
		Enable the "RX Complete Interrupt" 
		Enable the Receiver
		Set the bit UCSZ02 to 0  
		(Setting the bits UCSZ0(2:0) in the registers (UCSR0B & UCSR0C) to 0b011 sets the character size to 8-bit
	*/ 
	UCSR0B = 0b10010000;
	//UCSR0B = 0;
	
	/*
		<<Set Frame Format>>
		
		Set the Mode to Asynchronous USART (UMSEL0(1:0) = 0b00)
		Disable the Parity mode (UPM0(1:0) = 0b00)
		One Stop bit
		Character Size = 8-bit (UCSZ0(2:0) = 0b011)
	*/
	UCSR0C = 0b00000110;
}

void EnableDataRegEmptyInterrupt(void)
{
	UCSR0B |= 0b00100000;
}

void DisableDataRegEmptyInterrupt(void)
{
	UCSR0B &= 0b11011111;
}

void Delay_ms(uint16_t NoOfms)
{
	//Max Delay is 60000 mS.
	
	uint16_t Delay = 0;
	
	while(NoOfms--)
		for(Delay = 0; Delay < 468; Delay++);
}

void Timer1Init(void)
{
	// Configure Timer in Normal Mode.
	
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	
	//Enable Timer/Counter1 Overflow Interrupt 
	TIMSK1 = 1;
}

void StartTimer1(void)
{
	TCCR1B = 1;
}

void StopTimer1(void)
{
	TCCR1B = 0;
}

void New_ISR(void)
{
	
	if((TCNT1 < TC1_MinCntThres) && (TC1_OVF_Cnt == 0))
	{
		StallFreqDetectCnt++;
		
		if(StallFreqDetectCnt > 10) IsStallDetected = MOTOR_STALLED;
		
		Prev_TCNT1_SET_FLAG = NOT_SET;
		
		TCNT1 = 0;
		TC1_OVF_Cnt = 0;
	}
	else if((TCNT1 > TC1_MinCntThres) || (TC1_OVF_Cnt > 0))
	{
		StallFreqDetectCnt = 0;
		IsStallDetected = NO_STALL;
		
		IsFalseSpike = NO;
		
		if(TCNT1 < 12000)
		{
			HighFreqCnt++;
			
			if(HighFreqCnt > 20)
			{
				if(Prev_TCNT1_SET_FLAG == NOT_SET)
				{
					Prev_TCNT1_SET_FLAG = SET;
					Prev_TCNT1 = TCNT1;
					
					TCNT1 = 0;
					TC1_OVF_Cnt = 0;
				}
				else if(Prev_TCNT1_SET_FLAG == SET)
				{
					Dup_TCNT1 = TCNT1;
					TCNT1_Diff = Dup_TCNT1 - Prev_TCNT1;
					if(TCNT1_Diff < 0) TCNT1_Diff = -TCNT1_Diff;
					
					if(TCNT1_Diff < (Prev_TCNT1 / 20.0))
					{
						Prev_TCNT1 = Dup_TCNT1;
						
						TCNT1 = 0;
						TC1_OVF_Cnt = 0;
					}
					else IsFalseSpike = YES;
				}
			}
		}
		else
		{
			HighFreqCnt = 0;
			
			TCNT1 = 0;
			TC1_OVF_Cnt = 0;
		}
	}
	
	if(
	((Prev_TCNT1_SET_FLAG == SET) && (IsFalseSpike == NO) && (IsStallDetected == NO_STALL))
	||
	(IsStallDetected == MOTOR_STALLED)
	||
	(StallFreqDetectCnt > 0)
	)
	{
		switch(ConfigNo)
		{
			case 0: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); break; }
			case 1: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); break; }
			case 2: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); break; }
			case 3: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); break; }
			case 4: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); break; }
			case 5: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); break; }
		}
		
		OCR1A = OCR1A_VAl_forStatONFreq; // Set the new Stator ON Frequency
		
		if(IsStallDetected == NO_STALL)
		{
			OCR0A = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
			OCR0B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
			OCR2B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
		}
		else if(IsStallDetected == MOTOR_STALLED)
		{
			OCR0A = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
			OCR0B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
			OCR2B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
		}
		
		ConfigNo++;
		
		if(ConfigNo > 5) ConfigNo = 0;
	}
}

void Working_ISR(void)
{
	uint8_t TimePass = PORTC;
	HallSensor = TimePass & 0b00110001;
	
	if((TCNT1 < TC1_MinCntThres) && (TC1_OVF_Cnt == 0))
	{
		StallFreqDetectCnt++;
		
		if(StallFreqDetectCnt > 40) IsStallDetected = MOTOR_STALLED;
	}
	else if((TCNT1 > TC1_MinCntThres) || (TC1_OVF_Cnt > 0))
	{
		StallFreqDetectCnt = 0;
		IsStallDetected = NO_STALL;
	}
	
	switch(HallSensor)
	{
		case UH_VL: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); ConfigNo = 0; break; }
		case UH_WL: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); ConfigNo = 1; break; }
		case VH_WL: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); ConfigNo = 2; break; }
		case VH_UL: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); ConfigNo = 3; break; }
		case WH_UL: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); ConfigNo = 4; break; }
		case WH_VL: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); ConfigNo = 5; break; }
		default:
		{
			switch(ConfigNo)
			{
				case 0: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); break; }
				case 1: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); break; }
				case 2: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); break; }
				case 3: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); break; }
				case 4: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); break; }
				case 5: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); break; }
			}
			break;
		}
	}
	
	OCR1A = OCR1A_VAl_forStatONFreq; // Set the new Stator ON Frequency
	
	if(IsStallDetected == NO_STALL)
	{
		OCR0A = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
		OCR0B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
		OCR2B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
	}
	else if(IsStallDetected == MOTOR_STALLED)
	{
		OCR0A = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
		OCR0B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
		OCR2B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
	}
	
	ConfigNo++;
	
	if(ConfigNo > 5) ConfigNo = 0;
	
	TCNT1 = 0;
	TC1_OVF_Cnt = 0;
}

void False_Spike_Ignorance_ISR(void)
{
	uint8_t TimePass = PORTC;
	HallSensor = TimePass & 0b00110001;
	
	IsFakeSpike = NO;
	
	if((TCNT1 < TC1_MinCntThres) && (TC1_OVF_Cnt == 0))
	{
		StallFreqDetectCnt++;
		
		if(StallFreqDetectCnt > 40) IsStallDetected = MOTOR_STALLED;
		else IsFakeSpike = YES;
	}
	else if((TCNT1 > TC1_MinCntThres) || (TC1_OVF_Cnt > 0))
	{
		StallFreqDetectCnt = 0;
		IsStallDetected = NO_STALL;
	}
	
	if(IsFakeSpike == NO)
	{
		switch(HallSensor)
		{
			case UH_VL: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); ConfigNo = 0; break; }
			case UH_WL: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); ConfigNo = 1; break; }
			case VH_WL: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); ConfigNo = 2; break; }
			case VH_UL: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); ConfigNo = 3; break; }
			case WH_UL: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); ConfigNo = 4; break; }
			case WH_VL: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); ConfigNo = 5; break; }
			default:
			{
				switch(ConfigNo)
				{
					case 0: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); break; }
					case 1: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); break; }
					case 2: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); break; }
					case 3: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); break; }
					case 4: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); break; }
					case 5: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); break; }
				}
				break;
			}
		}
		
		OCR1A = OCR1A_VAl_forStatONFreq; // Set the new Stator ON Frequency
		
		if(IsStallDetected == NO_STALL)
		{
			OCR0A = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
			OCR0B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
			OCR2B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
		}
		else if(IsStallDetected == MOTOR_STALLED)
		{
			OCR0A = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
			OCR0B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
			OCR2B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
		}
		
		ConfigNo++;
		
		if(ConfigNo > 5) ConfigNo = 0;
		
		TCNT1 = 0;
		TC1_OVF_Cnt = 0;
	}
}

ISR(PCINT1_vect)
{
	IsFakeSpike = NO;
	
	if(TCNT1 < HALL_SENS_SWITCH_TCNT1_THRES)
	{
		HallSensStartFreqCnt++;
		
		//if(HallSensStartFreqCnt > 40) StartHallSensSwitch = START;
	}
	else 
	{
		HallSensStartFreqCnt = 0;
		StartHallSensSwitch = STOP;
	}
	
	if((TCNT1 < 850) && (TC1_OVF_Cnt == 0)) 
	{
		IsFakeSpike = YES;
	}
	
	if((TCNT1 < TC1_MinCntThres) && (TC1_OVF_Cnt == 0))
	{
		StallFreqDetectCnt++;
		
		if(StallFreqDetectCnt > 40) IsStallDetected = MOTOR_STALLED;
	}
	else if((TCNT1 > TC1_MinCntThres) || (TC1_OVF_Cnt > 0))
	{
		StallFreqDetectCnt = 0;
		IsStallDetected = NO_STALL;
	}
	
	if((IsFakeSpike == NO) || (IsStallDetected == MOTOR_STALLED))
	{
		uint8_t TimePass = PINC;
		HallSensor = TimePass & 0b00110001;
		
		if(StartHallSensSwitch == START)
		{	
			switch(HallSensor)
			{	
				case UH_VL: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); ConfigNo = 0; break; }
				case UH_WL: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); ConfigNo = 1; break; }
				case VH_WL: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); ConfigNo = 2; break; }
				case VH_UL: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); ConfigNo = 3; break; }
				case WH_UL: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); ConfigNo = 4; break; }
				case WH_VL: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); ConfigNo = 5; break; }
			}	
		}
		else
		{				
			switch(ConfigNo)
			{
				case 0: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS);
					PORTB |= 0x04;break; }
				case 1: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); break; }
				case 2: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); break; }
				case 3: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); break; }
				case 4: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); break; }
				case 5: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); break; }
			}				
		}
		
		OCR1A = OCR1A_VAl_forStatONFreq; // Set the new Stator ON Frequency
		
		if(IsStallDetected == NO_STALL)
		{
			OCR0A = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
			OCR0B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
			OCR2B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
		}
		else if(IsStallDetected == MOTOR_STALLED)
		{
			OCR0A = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
			OCR0B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
			OCR2B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
		}
		
		ConfigNo++;
		
		if(ConfigNo > 5) ConfigNo = 0;
		
		TCNT1 = 0;
		TC1_OVF_Cnt = 0;
	}
	
	PORTB &= 0xFB;
}

void MotorStartUpNew(void)
{
	ConfigNo++;
	if(ConfigNo > 5) ConfigNo = 0;
	
	switch(ConfigNo)
	{
		case 0: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); break; }
		case 1: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); break; }
		case 2: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); break; }
		case 3: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); break; }
		case 4: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); break; }
		case 5: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); break; }
	}
	
	/*
	switch(HallSensor)
	{
		case WH_VL: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); HallSensor = UH_VL; break; }
		case UH_VL: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); HallSensor = UH_WL; break; }
		case UH_WL: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); HallSensor = VH_WL; break; }
		case VH_WL: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); HallSensor = VH_UL; break; }
		case VH_UL: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); HallSensor = WH_UL; break; }
		case WH_UL: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); HallSensor = WH_VL; break; }
	}
	*/
	
	OCR1A = OCR1A_VAl_forStatONFreq; // Set the new Stator ON Frequency
	OCR0A = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
	OCR0B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
	OCR2B = STALL_RECOVERY_OCR02AB_VAL; // Set Duty Cycle
	
	TCNT1 = 0;
	TC1_OVF_Cnt = 0;
	
	Prev_TCNT1_SET_FLAG = NOT_SET;
}

void MotorStartUp(void)
{
	//Disable Pin Change Interrupt 1
	PCICR &= 0b11111101;
	uint16_t Cycle = 0;
	uint8_t StatStepSwitchTime_ms;
	
	for(StatStepSwitchTime_ms = 8; StatStepSwitchTime_ms > 0; StatStepSwitchTime_ms--)
	{
		for(Cycle = 0; Cycle < 50; Cycle++)
		{
			switch(ConfigNo)
			{
				case 0: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); break; }
				case 1: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); break; }
				case 2: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); break; }
				case 3: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); break; }
				case 4: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); break; }
				case 5: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); break; }
			}
	
			OCR1A = OCR1A_VAl_forStatONFreq; // Set the new Stator ON Frequency
			OCR0A = StallRecovery_OCR02AB_VAL; // Set Duty Cycle
			OCR0B = StallRecovery_OCR02AB_VAL; // Set Duty Cycle
			OCR2B = StallRecovery_OCR02AB_VAL; // Set Duty Cycle
	
			ConfigNo++;
	
			if(ConfigNo > 5) ConfigNo = 0;
	
			Delay_ms(StatStepSwitchTime_ms);
		}
	}
	
	/*
	StatStepSwitchTime_ms = 1;
	StallRecovery_OCR02AB_VAL = 45;
	
	for(Cycle = 0; Cycle < 15000; Cycle++)
	{
		switch(ConfigNo)
		{
			case 0: { OFFHighSide(WHS);					 ActivateStator(UHS, VLS); break; }
			case 1: {					OFFLowSide(VLS); ActivateStator(UHS, WLS); break; }
			case 2: { OFFHighSide(UHS);					 ActivateStator(VHS, WLS); break; }
			case 3: {					OFFLowSide(WLS); ActivateStator(VHS, ULS); break; }
			case 4: { OFFHighSide(VHS);					 ActivateStator(WHS, ULS); break; }
			case 5: {					OFFLowSide(ULS); ActivateStator(WHS, VLS); break; }
		}
		
		OCR1A = OCR1A_VAl_forStatONFreq; // Set the new Stator ON Frequency
		OCR0A = StallRecovery_OCR02AB_VAL; // Set Duty Cycle
		OCR0B = StallRecovery_OCR02AB_VAL; // Set Duty Cycle
		OCR2B = StallRecovery_OCR02AB_VAL; // Set Duty Cycle
		
		ConfigNo++;
		
		if(ConfigNo > 5) ConfigNo = 0;
		
		Delay_ms(StatStepSwitchTime_ms);
	}
	*/
	
	Delay_ms(20);
	
	// Enable Pin Change Interrupt 1
	PCICR |= 0b00000010;
	
}

void PhaseSwitchInterruptInit(void)
{	
	//Enable PCINT13 (PC5), PCINT12 (PC4), PCINT8 (PC0)
	PCMSK1 |= 0b00110001;
	
	// Enable Pin Change Interrupt 1
	PCICR |= 0b00000010;
}

void HighSideCapacitorCharge(void)
{
	Delay_ms(100);
	
	ONLowSide(ULS);
	ONLowSide(VLS);
	ONLowSide(WLS);
	
	Delay_ms(100);
	
	OFFLowSide(ULS);
	OFFLowSide(VLS);
	OFFLowSide(WLS);
}

/*
ISR(USART_UDRE_vect)
{
	
	//if(SendStatONFreqFlag == 1)
	{
		UDR0 = ReceivedBytes[19];
		DisableDataRegEmptyInterrupt();
		return;
		
		SendByteIndx++;
		if(SendByteIndx > 0)
		{
			SendByteIndx = 0;
			//SendStatONFreqFlag = 0;
			DisableDataRegEmptyInterrupt();
		}
	}
	
}
*/

int main(void)
{
	// Set PD(WHS), PD(VHS), PD(UHS) as 1, i.e. Switch OFF Them
	PORTD |= 0b01101000;
	
	// Set PD7(WLS), PD4(VLS), PD2(ULS) as 0, i.e. Switch OFF them
	PORTD &= 0b01101011;
	
	// Set PD(2,3,4,5,6,7)as outputs
	DDRD |= 0b11111100;
	
	// Set PB2 as output
	DDRB |= 0x04;
	
	// Set PC(0, 4, 5) as inputs
	DDRC &= 0b11001110;
	
	PORTB &= 0xFB;
	
	HighSideCapacitorCharge();
	
	OCR1A_VAl_forStatONFreq = PrescaledFreq / StatorONFreq / 6;
	OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);

	PWMInit();
	USARTInit();
	EnableClocks();
	PhaseSwitchInterruptInit();
	Timer1Init();
	sei();
	
	StartTimer1();
	
	while(1)
	{
		if(TC1_OVF_Cnt == 10) 
		{
			IsStallDetected = MOTOR_STALLED;
			MotorStartUpNew();
		}
		
		if(USART_Receiver_Status == RECEIVING_DONE)
		{	
			if(ReceivedBytes[0] == USART_RECEIVER_START_BYTE)
			{
				uint8_t CheckSum = 0;
				
				for(uint8_t Indx = 0; Indx < LAST_RECEIVE_BYTE_INDX; Indx++)
					CheckSum += ReceivedBytes[Indx];
				
				CheckSum += CHECKSUM_OFFSET;
				
				if(CheckSum == USART_RECEIVER_START_BYTE) CheckSum--;
				
				if((ReceivedBytes[LAST_RECEIVE_BYTE_INDX] == CheckSum) && (ReceivedBytes[OCR02AB_BYTE_INDX] < MAX_OCR02B_BYTE_VAL))
				{
					OCR02AB_Val_forDutyCycle = ReceivedBytes[OCR02AB_BYTE_INDX];
				}
			}
			
			USART_Receiver_Status = NOT_RECEIVING;
		}
	}
}
