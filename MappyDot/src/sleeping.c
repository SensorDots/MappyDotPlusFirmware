/*
 * sleeping.c
 *
 * Created: 29/10/2017 1:41:35 AM
 *  Author: Blair Wyatt
 */ 

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include "sleeping.h"

void disable_analog()
{
	// Disable the ADC by setting the ADEN bit (bit 7)  of the
	// ADCSRA register to zero.
	ADCSRA = ADCSRA & 0x7f;

	// Disable the analog comparator by setting the ACD bit
	// (bit 7) of the ACSR register to one.
	ACSR = 0x80;

	// Disable digital input buffers on all analog input pins
	// by setting bits 0-5 of the DIDR0 register to one.
	DIDR0 = DIDR0 | 0x3f;
}

void sleep_avr()
{  
    // Choose sleep mode:
    set_sleep_mode(SLEEP_MODE_IDLE);
    
    // Set sleep enable (SE) bit:
    sleep_enable();
    
    // Put the device to sleep:
    sleep_mode();
    
    // Upon waking up, continue from this point.
    sleep_disable();
}