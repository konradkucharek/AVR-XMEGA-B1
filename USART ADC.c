//Program: USART, ADC

#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "AVRX_Clocks.h"
#include "AVRXSerial.h"
#include "addr_macro.h"
#include "iox128b1.h"

*Defines for setting up serial link*/
#define BSCALE_FACTOR 0
#define FBAUD 96
#define nBScale 0
#define RX_BUFSIZE 80
#define TX_BUFSIZE 40
#define ADCA_CH1_vect_num  72
#define ADCA_CH1_vect      _VECTOR(72) // Interrupt 1
#define ADCA_CH2_vect_num  73
#define ADCA_CH2_vect      _VECTOR(73) // Interrupt 2
#define ADCA_CH1_RES  _SFR_MEM16(0x0224)
#define ADCA_CH2_RES  _SFR_MEM16(0x0224)

volatile XUSARTst stU;
volatile int adcBuf_CH0 = 0;
volatile int adcBuf_CH1 = 0;
volatile int adcBuf_CH2 = 0;
uint16_t adcVal;
char bADC = 0;

ISR(ADCA_CH0_vect)
{
	adcBuf = ADCA_CH0_RES;
}

ISR(ADCA_CH1_vect)
{
	adcBuf_CH1 = ADCA_CH1_RES;
}

ISR(ADCA_CH2_vect)
{
	adcBuf_CH2 = ADCA_CH2_RES;
}

/***** SET UP RECIEVER ******/
ISR(USARTC0_RXC_vect)
{
	Rx_Handler(&stU);
}
/******** SET UP TRANSMITTER ***********/
ISR(USARTC0_TXC_vect)
{
	Tx_Handler(&stU);
}

int main(int argc, char const *argv[])
{
	unsigned long sClk, pClk;
	char x[10], y[10], z[10];
	cli(); /*disable interrupts*/
	
	/************ SET UP SYSTEM CLOCK *************/
	SetSystemClock(CLK_SCLKSEL_RC32M_gc, CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);
	GetSystemClocks(&sClk, &pClk);
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	
	/************ SET UP ADC *************/
	ADCA_CTRLA |= ADC_ENABLE_bm; // Set the overall configuration for a sample on channel A
	ADCA_CTRLB = ADC_RESOLUTION_12BIT_gc; // Set the ADC conversion resolution
	ADCA_REFCTRL = ADC_REFSEL_AREFA_gc; // Select the voltage reference(external ref on port A)
	ADCA_PRESCALER |= ADC_PRESCALER_DIV32_gc; // Pre-scale ADC clock input
	ADCA_CH0_CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_1X_gc; // Set channel input mode and gain
	ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN6_gc; // Set pin to get value from
	
	/************ SET UP SERIAL PORT *************/
	// Initialize serial port to desired values
	USART_init(&stU, 0xC0, pClk, (_USART_TXCIL_LO | _USART_RXCIL_LO), 576, 0,	_USART_CHSZ_8BIT, _USART_PM_DISABLED, _USART_SM_1BIT);
	// Initialize a buffer for incoming and outgoing serial transmissions
	USART_buffer_init(&stU,_RX_BUF_SZ,TX_BUFSIZE);
	// Set the input and output modes for the specified serial port.
	stU.fInMode = _INPUT_CR | _INPUT_ECHO | _INPUT_TTY;
	stU.fOutMode = _OUTPUT_CRLF;
	// Enable specified serial port
	USART_enable(&stU, (USART_TXEN_bm | USART_RXEN_bm));
	sei(); // Enable interrupts
	
	/************ PROGRAM LOOP *************/
	while (!(stU.serStatus & _USART_TX_EMPTY) ) { ; }
	while(1)
	{
		if (stU.serStatus & _USART_RX_DONE)
		{
			USART_read(&stU, x);
			if (strlen(x) > 0)
			{
				ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN5_gc;
				ADCA.CH0.CTRL |= ADC_CH_START_bm;
				while (!(ADCA_INTFLAGS & 0x01));
				adcBuf_CH0 = ADCA_CH0_RES ;
				sprintf(x,"\nx axis %d", adcBuf_CH0);
				USART_send(&stU, x);
				while (!(stU.serStatus & _USART_TX_EMPTY) )
				{
					;
				}
			
				ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;
				ADCA.CH0.CTRL |= ADC_CH_START_bm;
				while (!(ADCA_INTFLAGS & 0x01));
				adcBuf_CH1 = ADCA_CH1_RES ;
				sprintf(y,"y axis %d", adcBuf_CH1);
				USART_send(&stU, y);
				while (!(stU.serStatus & _USART_TX_EMPTY) )
				{
					;
				}
			
				ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN7_gc;
				ADCA.CH0.CTRL |= ADC_CH_START_bm;
				while (!(ADCA_INTFLAGS & 0x01));
				adcBuf_CH2 = ADCA_CH2_RES ;
				sprintf(z,"z axis %d", adcBuf_CH2);
				USART_send(&stU, z);
				while (!(stU.serStatus & _USART_TX_EMPTY) ) 
				{
					;
				}
			}
		}
	}
}
