// Program: MCU I/O via Timer (Interrupt)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <AVRXlib/AVRX_Clocks.h>

ISR(TCC0_OVF_vect) {
	PORTB_OUT ^= 0x01;
}

int main(void)
{
	// Part I & II - Timer C0 and its Interrupt
    cli();
	// Set all pins on port B to be output.
	PORTB_DIR = 0xFF; 
	PORTB_OUT = 0xA3;

	TCC0_PER = 0x3E8;
	//TCC0_PER = 0x3DE; 
	TCC0_CTRLA = 0x06;
	TCC0_INTCTRLA = PMIC_MEDLVLEX_bm;
	
	PMIC_CTRL |= PMIC_MEDLVLEN_bm;
	sei();
	
	// Part III - Real Time Clock and Multiplexing
	// Set & enable clock to 32.768 kHz
	CLK_RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm; 
	
	//RTC_PER =
	RTC_CTRL = 0x01;
	RTC_INTCTRL = PMIC_LOLVLEX_bm; // Set to low priority
		
	// Part IV - Capturing Input
	PORTCFG_MPCMASK = 0x01 | 0x02;
	// Set all pins on port E to be output
	PORTE_PIN1CTRL = 0x01; 
		
    while (1) 
    {
	}	
}