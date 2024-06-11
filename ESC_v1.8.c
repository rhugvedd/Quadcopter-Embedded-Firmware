/*
	* ESC.c
	*
	* Author : Rhugved Chaudhari
	*/ 

#include<avr/io.h>
#include<avr/interrupt.h>

// PWM for MOSFET High Sides is configured on the Pins PD3, PD5, PD6
// Signals for Low Sides are configured on the Pins PD2, PD4, PD7

#define MCKFreq 8000000L // Master Clock Frequency
#define USART	BAUD 62500 // Baud rate of USART Communication
#define UBRR0Val (MCKFreq / 16 / USARTBAUD) - 1 // Baud Rate registers value
#define ESC_ID 1

#define USART_RECEIVER_START_BYTE (uint8_t) ('S' | 0x80)
#define USART_RECEIVER_STOP_BYTE (uint8_t) ('E' | 0x80)

#define NOT_RECEIVING 0
#define RECEIVING_STARTED 1
#define RECEIVING_DONE 2

#define OCR1A_HIGH_BYTE_INDX (ESC_ID * 4) + 1
#define OCR1A_LOW_BYTE_INDX (ESC_ID * 4) + 2
#define OCR02AB_BYTE_INDX (ESC_ID * 4) + 3
#define MSB_BITS_BYTE_INDX (ESC_ID * 4) + 4

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

// PWM Prescalers = {1, 8, 64, 256, 1024}
// Corresponding N = {1, 2, 3,  4,   5}
#define PWMPrescalerIndxTC0 1 // PWM Freq = MCKFreq / (N * 256)

// PWM Prescalers = {1, 8, 32, 64, 128, 256, 1024}
// Corresponding N = {1, 2, 3,  4,  5,   6,   7}
#define PWMPrescalerIndxTC2 1 // PWM Freq = MCKFreq / (N * 256)

// Prescalers = {1, 8, 64, 256, 1024}
// Corresponding N = {1, 2, 3,  4,   5}
#define TC1PrescalerIndx 1 

#define PrescaledFreq MCKFreq / PWMPrescalerIndxTC0

#define CHECKSUM_OFFSET 0x50

#define MAX_PID_STEP_VAL 40

void PWMInit(void);
void EnablePWMClock(void);
void DisablePWMClock(void);
void StatorTimerInit(void);
void ONHighSide(uint8_t HighSide);
void OFFHighSide(uint8_t HighSide);
void ONLowSide(uint8_t LowSide);
void OFFLowSide(uint8_t LowSide); 
void StatorFreqUpdate(float StatorONFreqDiff);
void ActivateStator(uint8_t HighSide, uint8_t LowSide);
void USARTInit(void);
void HighSideCapacitorCharge(void);
void Delay_ms(uint16_t NoOfms);
void SetDutyCycle(float StatorONFreq);
void EnableDataRegEmptyInterrupt(void);
void DisableDataRegEmptyInterrupt(void);

uint8_t TC0_OV_Flag = 0;
uint8_t IntrptCnt = 0;

//DutyCycle should be such that
// 1 <= DutyCycle <= 90
uint8_t DutyCycle = 30;
float StatorONFreq = 21; // Minimum value can be 21Hz

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
	
	// Enable global interrupts
	//sei();
	
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

void StatorTimerInit(void)
{
	// TC1 Configuration for Timer Interrupt for Stator ON Time
	{	
		/*
		* TC1 Initialization Explanation 
		* TC1 is set in Fast PWM Mode (Mode 15)
		* In this mode the Counter starts with the BOTTOM value(0x00) counting up steadily and
			overflows and resets to BOTTOM(0x00) when it reaches the "TOP" i.e. the value mentioned
			in the OCR1A register.
		* Enable the Output Compare A Interrupt so that the software gets an interrupt whenever the 
			counter TC1 reaches the OCR1A value and resets to bottom. In this interrupt we will set the 
			next stator configuration.
		*/
		
		// Set TC1 in Fast PWM Mode (Mode 15 -> WGM1(3:0) = 0b1111) - Inverting Configuration(Active Low).
		// Clock Source not set yet - TC1 not started yet.
		TCCR1A = 0b11110011;
		TCCR1B = 0b00011000;
		
		// Set Output Compare Register A & B with the Duty Cycle Value.
		OCR1A = PrescaledFreq / StatorONFreq / 6;
		//OCR1B = (PrescaledFreq / StatorONFreq) * DutyCycle / 100;
		
		//OCR1AH = (uint8_t)((PrescaledFreq / StatorONFreq) >> 16);
		//OCR1AL = (uint8_t)((PrescaledFreq / StatorONFreq) & 0xFF);
		//OCR1BH = ((PrescaledFreq / StatorONFreq) * DutyCycle / 100) >> 16;
		//OCR1BL = ((PrescaledFreq / StatorONFreq) * DutyCycle / 100) * 0xFF;
		
		// Enable TC1 Output Compare Match A Interrupt
		TIMSK1 |= 0b00000010;
		
		// Enable global interrupts
		//sei();
	}
}

void StatorFreqUpdate(float StatorONFreqDiff)
{
	StatorONFreq += StatorONFreqDiff;

	SetDutyCycle(StatorONFreq);
	
	//TCNT1 = 0;
	
	OCR1A = PrescaledFreq / StatorONFreq / 6;
	
	//OCR1AH = (uint8_t)((PrescaledFreq / StatorONFreq) >> 16);
	//OCR1AL = (uint8_t)((PrescaledFreq / StatorONFreq) & 0xFF);
}

void SetDutyCycle(float StatorONFreq)
{
	DutyCycle = ((StatorONFreq - 20) * 0.12) + 20;
	
	if((DutyCycle >= 1) && (DutyCycle <= 90))
	{
		// Set Output Compare Register A with the Duty Cycle Value. - PWM Channel on PD6
		OCR0A = (uint8_t)(DutyCycle * 2.55); // OCR0A = (DutyCycle / 100) * 255
		// Set Output Compare Register B with the Duty Cycle Value. - PWM Channel on PD5
		OCR0B = (uint8_t)(DutyCycle * 2.55); // OCR0B = (DutyCycle / 100) * 255
		
		// Set Output Compare Register B with the Duty Cycle Value. - PWM Channel on PD3
		OCR2B = (uint8_t)(DutyCycle * 2.55); // OCR0B = (DutyCycle / 100) * 255
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

ISR(TIMER1_COMPA_vect)
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
	OCR0A = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
	OCR0B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
	OCR2B = OCR02AB_Val_forDutyCycle; // Set Duty Cycle
	
	ConfigNo++;
	
	if(ConfigNo > 5)
	{
		ConfigNo = 0;
	}
}

ISR(USART_RX_vect)
{
	USARTReceivedData = UDR0;
	
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
		if(ReceivedBytesIndx < 19)
			ReceivedBytes[ReceivedBytesIndx] = USARTReceivedData;	
			
		if(ReceivedBytesIndx == 18)
			USART_Receiver_Status = RECEIVING_DONE;
	}
}

void USARTInit(void)
{
	//Set the Baud rate by writing into the registers the baud rate Prescaler value
	//UBRR0H = UBRR0Val >> 8;
	//UBRR0L = UBRR0Val * 0xFF;
	UBRR0 = UBRR0Val;

	/*  
		Enable the "RX Complete Interrupt" 
		Enable the Transmitter 
		Enable the Receiver
		Set the bit UCSZ02 to 0  
		(Setting the bits UCSZ0(2:0) in the registers (UCSR0B & UCSR0C) to 0b011 sets the character size to 8-bit
	*/ 
	UCSR0B = 0b10011000;
	
	/*
		<<Set Frame Format>>
		
		Set the Mode to Asynchronous USART (UMSEL0(1:0) = 0b00)
		Disable the Parity mode (UPM0(1:0) = 0b00)
		One Stop bit
		Character Size = 8-bit (UCSZ0(2:0) = 0b011)
	*/
	UCSR0C = 0b00000110;
	
	//Enable global interrupts
	//sei();
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

int main(void)
{
	// Set PD(WHS), PD(VHS), PD(UHS) as 1, i.e. Switch OFF Them
	PORTD |= 0b01101000;
	
	// Set PD7(WLS), PD4(VLS), PD2(ULS) as 0, i.e. Switch OFF them
	PORTD &= 0b01101011;
	
	// Set PD(2,3,4,5,6,7)as outputs
	DDRD |= 0b11111100;
	
	HighSideCapacitorCharge();
	
	OCR1A_VAl_forStatONFreq = PrescaledFreq / StatorONFreq / 6;
	OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);

	PWMInit();
	USARTInit();
	StatorTimerInit();
	EnableClocks();
	
	sei();
	
	uint16_t NewOCR1A_VAl_forStatONFreq;
	uint8_t DutyCycle;
	float New_StatOnFreq, Old_StatOnFreq;
	int16_t StatONFreqDiff = 0;
	
	while(1)
	{
		if(USART_Receiver_Status == RECEIVING_DONE)
		{
			//USART_Receiver_Status = NOT_RECEIVING;
			
			if((ReceivedBytes[0] == USART_RECEIVER_START_BYTE) && (ReceivedBytes[18] == USART_RECEIVER_STOP_BYTE))
			{
				uint8_t CheckSum = 0;
				
				for(uint8_t Indx = 0; Indx < 17; Indx++)
					CheckSum += ReceivedBytes[Indx];
				
				CheckSum = (CheckSum + CHECKSUM_OFFSET) & 0x7F;
				
				if(ReceivedBytes[17] == CheckSum)
				{
					NewOCR1A_VAl_forStatONFreq = (ReceivedBytes[MSB_BITS_BYTE_INDX] << 14) | (ReceivedBytes[OCR1A_HIGH_BYTE_INDX] << 7) | ReceivedBytes[OCR1A_LOW_BYTE_INDX];
					New_StatOnFreq = ((float)MCKFreq * 10) / (6 * (float)NewOCR1A_VAl_forStatONFreq);
					Old_StatOnFreq = ((float)MCKFreq * 10) / (6 * (float)OCR1A_VAl_forStatONFreq);
					StatONFreqDiff = New_StatOnFreq - Old_StatOnFreq;
					
					if((-MAX_PID_STEP_VAL <= StatONFreqDiff) && (StatONFreqDiff <= MAX_PID_STEP_VAL))
					{
						OCR1A_VAl_forStatONFreq = NewOCR1A_VAl_forStatONFreq;
						OCR02AB_Val_forDutyCycle = ReceivedBytes[OCR02AB_BYTE_INDX] | (ReceivedBytes[MSB_BITS_BYTE_INDX] << 1);
					}
					else
					{
							 if(StatONFreqDiff > MAX_PID_STEP_VAL)  Old_StatOnFreq += MAX_PID_STEP_VAL;
						else if(StatONFreqDiff < -MAX_PID_STEP_VAL) Old_StatOnFreq -= MAX_PID_STEP_VAL;
						
						DutyCycle = ((((float)Old_StatOnFreq / 10) - 20) * 0.1) + 20;

						if(DutyCycle < 20)			DutyCycle = 20;
						else if(DutyCycle > 90)		DutyCycle = 90;
						
						OCR1A_VAl_forStatONFreq = (uint16_t)((float)MCKFreq / (Old_StatOnFreq / 10.0) / 6);
						OCR02AB_Val_forDutyCycle = (uint8_t)(DutyCycle * 2.55);
					}
				}
			}
			
			USART_Receiver_Status = NOT_RECEIVING;
		}
	}
}
