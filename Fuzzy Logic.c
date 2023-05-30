// Program: Fuzzy Logic

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include "AVRX_Clocks.h"
#include "AVRXSerial.h"
#include <string.h>

// Serial Calculations
// nBSel = (32000000 / (2^0*16*9600) -1 = 207, nBScale = 0
// BAUD  = ( (32000000)/(2^0*16(207+1)) = 9615 ~9600
#define TRUE 1
#define FALSE 0

#define _RX_BUF_SZ 80 // RX buffer size
#define _TX_BUF_SZ 40 // TX buffer size


// Light Sensor Defines
#define loLvlDark 0 // The low level value range of dark for the light sensor 
#define loLvlDim 1300 // The low level value range of dim for the light sensor
#define loLvlBright 1600 //2800 the low level value range of bright for the light sensor
#define hiLvlDark 1500 // The high level value range of dark for the light sensor
#define hiLvlDim 1700 // 3000 the high level value range of dim for the light sensor
#define hiLvlBright 4300 // The high level value range of bright for the light sensor

// Accelerometer Sensor Defines
#define loleftT 2700
#define lorightT 2000
#define loflatT  2200
#define hileftT 3200
#define hirightT 2300
#define hiflatT  2800



// Accelerometer Defines
typedef enum{
	idle, dark, dim, bright 
}lightSensor;

typedef enum{
	null, leftTilt, flat, rightTilt
}accelerometerSensor;


// Globals - push clock parameters to top level so they're visible to functions
unsigned long sClk, pClk;
volatile XUSARTst stU;
volatile int adcaBufX = 0, adcaBufY = 0, adcaBufZ = 0, adcbBuf = 0;
lightSensor ligState = idle;
accelerometerSensor accState = null;

// Make the variable "true" depending on values from sensors
void fuzzification(){ 
	// Light sensor 
	// If the sensor reads is greater than the low lvl dark & less than low lvl dim
	if(adcbBuf >= loLvlDark && adcbBuf < hiLvlDark)
	{ 
		if(adcbBuf < loLvlDim)
		{
			// then it's dark
			ligState =dark; 
		}
		else
		{
			// not sure if it's dark or dim
			*(lightSensor**)ligState=(lightSensor*)idle; 
		}
	}
	if(adcbBuf >= loLvlDim && adcbBuf < hiLvlDim)
	{
		if(adcbBuf >= loLvlDark && adcbBuf <= hiLvlBright)
		{
			// then its dim
			ligState=dim; 
		}
		else
		{
			// not sure if it's dark, dim, or bright
			*(lightSensor**)ligState=(lightSensor*)idle; 
		}
	}
	if(adcbBuf >= loLvlDark && adcbBuf < hiLvlBright)
	{
		if(adcbBuf >= hiLvlDim)
		{
			// then it's bright
			ligState=bright; 
		}
		else
		{
			// not sure if it's dim or bright
			*(lightSensor**)ligState=(lightSensor*)idle; 
		} 

	/*** Acceleration ***/ 
	if((adcaBufX >= lorightT) && (adcaBufX < loflatT))
	{
		// then it's right tilt
		accState =rightTilt;
	}
		else
		{
			//  not sure if it's dark or dim
			*(accelerometerSensor**)accState=(accelerometerSensor*)null; 
		}
	}
	if((adcaBufX >= loflatT) && (adcaBufX < hiflatT))
	{
		if(adcaBufX >= hirightT && adcaBufX <= loleftT)
		{
		//then it's dim
		accState=flat;
		}
		else
		{
			// not sure if it's dark, dim, or bright
			*(accelerometerSensor**)accState=(accelerometerSensor*)null;
		}
	}
	if((adcaBufX >= loleftT) && (adcaBufX < hileftT))
	{
		if(adcaBufX >= hiflatT)
		{
			//then it's bright
			accState=leftTilt;
		}
		else
		{
			//then not sure if it's dim or bright
			*(accelerometerSensor**)accState=(accelerometerSensor*)null;
		}
	}
}
 
// Method that toggles the LEDs if fuzzy logical works
void ledToggle(){
	// If Dark & Left Tilt
	if(ligState == dark && accState == leftTilt)
	{ 
		// LED 0 turns on
		PORTB_OUT ^= 0x10;
	}
	// If Dim & Flat
	if(ligState == dim && accState == flat)
	{ 
		PORTB_OUT ^= 0x20; // LED 1 turns on
	}
	// If Bright & Right Tilt
	if(ligState == bright && accState == rightTilt)
	{ 
		// LED 2 turns on
		PORTB_OUT ^= 0x40;
	}
}

// Serial ISRs
// Executes when rx complete
ISR(USARTC0_RXC_vect)
{
	Rx_Handler(&stU);
}

// Executes when tx complete
ISR(USARTC0_TXC_vect) 
{
	Tx_Handler(&stU);
}

int main(void)
{
	//local variables
	char rxbuffer[20], txbuffer[40];
	
	cli();
	
	// Clock Setup - shift to 32 MHz internal RC oscillator
	SetSystemClock(CLK_SCLKSEL_RC32M_gc, CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);
	GetSystemClocks(&sClk, &pClk);
	
	PORTB_DIR = 0x70; // Make the first 3 LEDs on the board to input
	PORTC_DIR = 0x0A;
	
	PORTB_OUT = 0xF0; // Turn off the lights
	
	// Set up the USART on C0 for 57600 baud 8N1 with low priority k
	// Enable low priority interrupts
	USART_init(&stU, 0xC0, pClk, (_USART_TXCIL_LO | _USART_RXCIL_LO), 576, -4,	_USART_CHSZ_8BIT, _USART_PM_DISABLED, _USART_SM_1BIT);
	
	// Initialize the driver ring buffers for incoming and outgoing serial transmissions
	USART_buffer_init(&stU, _RX_BUF_SZ, _TX_BUF_SZ);
	
	// Set the input and output modes for the specified serial port
	stU.fInMode =  _INPUT_ECHO | _INPUT_CR | _INPUT_TTY;
	stU.fOutMode = _OUTPUT_CRLF;
	
	// Enable specified serial port
	USART_enable(&stU, (USART_TXEN_bm | USART_RXEN_bm));
	
	// ADC SetUp	
	// ADCA for accelerometer
	// Set the configuration for a sample on channel A
	ADCA_CTRLA = ADC_ENABLE_bm; // 0x05; // Bits: 7:3(reserved) 2(CH0START) 1(FLUSH) 0(ENABLE)
	// Resolution control
	ADCA_CTRLB = ADC_RESOLUTION_12BIT_gc; // 0x08; // Bits: 7(reserved) 6:5(CURRLIMIT[1:0]) 4(CONVMODE) 3(FREERUN) 2:1(RESOLUTION[1:0]) 0(reserved)
	// Reference control register either
	ADCA_REFCTRL = ADC_REFSEL_AREFA_gc; // 0x40; // Bits: 7(reserved) 6:4(REFSEL[2:0]table 26-3) 3:2(reserved) 1(BANDGAP) 0(TEMPREF)
	// Set the clock input to the ADC,and integration time between samples
	ADCA_PRESCALER = ADC_PRESCALER_DIV512_gc; // 0x03; // Bit 2:0 – PRESCALER[2:0]: Prescaler Configuration Table 26-6(DIV32) the quicker the better
	// Setting the input and the output of the channel
	ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN5_gc; // 0x28; // Bits: 7(reserved = 0) 6:3(MuxPos = 0101)[lets use pin 5] 2:0(MuxNeg = 000)
	//InputMode[1:0] is 01 (single-ended) therefore using table 26-11 for MuxPos
	// Since we are in single-ended measurements, MuxNeg is unused
	ADCA_CH0_CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; //0x08; //input mode(ADC_CH_INPUTMODE_SINGLEENDED_gc = 01) and gain(ADC_CH_GAIN_1X_gc = 000)
	// Therefore MUXCTRL Bits: 7(start) 6:5 (reserved) 4:2(Gain) 1:0 (input)
	
	// ADCB for light sensor
	ADCB_CTRLA = ADC_ENABLE_bm | ADC_CH0START_bm;
	ADCB_CTRLB = ADC_RESOLUTION_12BIT_gc;
	ADCB_REFCTRL = ADC_REFSEL_INT1V_gc;
	// ADCB_REFCTRL = ADC_REFSEL_AREFA_gc; // need to connect 3.3V to PB0
	ADCB_PRESCALER = ADC_PRESCALER_DIV512_gc;
	ADCB_CH0_CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_1X_gc;
	ADCB_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;

	
	// Unmask desired interrupt priority levels
	PMIC_CTRL = PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm; // | PMIC_HILVLEN_bm;
	// Enable interrupts
	sei(); 

    USART_send(&stU, "c:\n");
	while(!(stU.serStatus & _USART_TX_EMPTY));
	
	// Prompt the user
	USART_send(&stU, "Press enter to collect data\n"); 
	stU.serStatus &= ~(_USART_RX_DONE);
	
	
	while(1)
	{
		
		if (stU.serStatus & _USART_RX_DONE)  // If true, the specified EOL character has been detected
		{
			USART_read(&stU, rxbuffer); // Read data out of ring buffer into local variable
		
			if(strlen(rxbuffer) > 0){ // Valid character
				// First axis
				ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN5_gc; // Change the accelerometers muxctrl to measure first axis
				ADCA_CH0_CTRL |= ADC_CH_START_bm;
				
				while(!(ADCA_INTFLAGS & 0x01)){;}
				
				adcaBufX = ADCA_CH0_RES; // Saves the results from ADC into adcaBuf
				sprintf(txbuffer, "X axis: %d", adcaBufX); // Output the the axis and its value
				
				USART_send(&stU, txbuffer);
				while(!(stU.serStatus & _USART_TX_EMPTY));

				// Second axis
				ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN6_gc; // Change the accelerometers muxctrl to measure first axis
				ADCA_CH0_CTRL |= ADC_CH_START_bm;
				
				while(!(ADCA_INTFLAGS & 0x01));
				
				adcaBufY = ADCA_CH0_RES; // Saves the results from ADC into adcaBuf
				sprintf(txbuffer, "Y axis: %d", adcaBufY); // Output the the axis and its value
			
				USART_send(&stU, txbuffer);
				while(!(stU.serStatus & _USART_TX_EMPTY));
				
				// Third axis
				ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN5_gc; // Change the accelerometers muxctrl to measure first axis
				ADCA_CH0_CTRL |= ADC_CH_START_bm;
				
				while(!(ADCA_INTFLAGS & 0x01));
				
				adcaBufZ = ADCA_CH0_RES; // Saves the results from ADC into adcaBuf
				sprintf(txbuffer, "Z axis: %d", adcaBufZ); // Output the the axis and its value
				
				USART_send(&stU, txbuffer);
				while(!(stU.serStatus & _USART_TX_EMPTY));
			
				// Light sensor
				ADCB_CH0_CTRL |= ADC_CH_START_bm;
				while(!(ADCA_INTFLAGS & 0x01));
				
				adcbBuf = ADCB_CH0_RES; // Saves the results from ADC into adcaBuf
				sprintf(txbuffer, "Light sensor: %d\n", adcbBuf); // Output the the axis and its value
				
				USART_send(&stU, txbuffer);
				while(!(stU.serStatus & _USART_TX_EMPTY));
				
				// Fuzzify our data into fuzzy variables
				fuzzification();
				
				// Display fuzzyness to USART
				// Reset the txbuffer
				memset(txbuffer,0,strlen(txbuffer)); // Set 0 for each index in txbuffer
				
				// Display the correct fuzziness based off the logic
				switch((lightSensor)ligState)
				{ 
					case dark:
					strcat(txbuffer,"Dark\t");
					break;
					case dim:
					strcat(txbuffer,"Dim\t");
					break;
					case bright:
					strcat(txbuffer,"Bright\t");
					break;
					case idle:
					strcat(txbuffer,"Idle\t");
					break;
					
				}
				
				// Display the correct fuzziness based off the logic
				switch(accState)
				{
					case leftTilt:
					strcat(txbuffer, "Left Tilt\n");
					break;
					case rightTilt:
					strcat(txbuffer, "Right Tilt\n");
					break;
					case flat:
					strcat(txbuffer, "Flat\n");
					break;
					case null:
					strcat(txbuffer, "Null\n");
					break;
				}
				// Send to USART
				USART_send(&stU, txbuffer);
				while(!(stU.serStatus & _USART_TX_EMPTY));
				
				// ledToggle according to fuzzy variables
				ledToggle();
			}
		}
		
	}
	return(0);	
}
