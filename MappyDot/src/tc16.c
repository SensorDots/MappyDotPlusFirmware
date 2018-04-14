/**
 * \file
 *
 * \brief TC16 related functionality implementation.
 *
 * Copyright (C) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <tc16.h>
#include <utils.h>

#define PWM_TOP			99 //100 steps (start from 0)

/**
 * \brief Initialize TIMER_0 interface
 */
int8_t TIMER_0_init()
{
	/* Runs at approximately 1.2kHz */

	/* Enable TC1 */
	PRR0 &= ~(1 << PRTIM1);

	// TCCR1A = (0 << COM1A1) | (0 << COM1A0) /* Normal port operation, OCA disconnected */
	//		 | (0 << COM1B1) | (0 << COM1B0) /* Normal port operation, OCB disconnected */
	//		 | (0 << WGM11) | (0 << WGM10); /* TC16 Mode 4 CTC (reset on OCR1A (top))*/

	TCCR1B = (0 << WGM13) | (1 << WGM12)                /* TC16 Mode 4 CTC */
	         | 0 << ICNC1                               /* Input Capture Noise Canceler: disabled */
	         | 0 << ICES1                               /* Input Capture Edge Select: disabled */
	         | (0 << CS12) | (1 << CS11) | (1 << CS10); /* IO clock divided by 64 */

	/* Input capture value, used as top counter value in some modes: 0 */
	// ICR1 = 0; 

	//OCR1A needs to be higher

	OCR1A = PWM_TOP; /* Output compare A: 100 */

	OCR1B = 64; /* Output compare B: 64 */

	TIMSK1 = 1 << OCIE1B   /* Output Compare B Match Interrupt Enable: enabled */
	         | 1 << OCIE1A /* Output Compare A Match Interrupt Enable: enabled */
	         | 0 << ICIE1  /* Input Capture Interrupt Enable: disabled */
	         | 0 << TOIE1; /* Overflow Interrupt Enable: disabled */

	return 0;
}

void TIMER_0_set_duty(uint8_t duty)
{
	if (duty >= PWM_TOP) duty = PWM_TOP - 1;
	OCR1B = duty;
}

void TIMER_0_stop()
{
	 TCCR1B = (0 << WGM13) | (1 << WGM12) /* TC16 Mode 4 CTC */
			 | 0 << ICNC1 /* Input Capture Noise Canceler: disabled */
			 | 0 << ICES1 /* Input Capture Edge Select: disabled */
			 | (0 << CS12) | (0 << CS11) | (0 << CS10); /* No clock source (Timer/Counter stopped) */
}

/**
 * \brief Initialize TIMER_1 interface
 */
int8_t TIMER_1_init()
{

	/* Enable TC3 */
	PRR1 &= ~(1 << PRTIM3);

	// TCCR3A = (0 << COM3A1) | (0 << COM3A0) /* Normal port operation, OCA disconnected */
	//		 | (0 << COM3B1) | (0 << COM3B0) /* Normal port operation, OCB disconnected */
	//		 | (0 << WGM31) | (0 << WGM30); /* TC16 Mode 4 CTC */

	TCCR3B = (0 << WGM33) | (1 << WGM32)                /* TC16 Mode 4 CTC */
	         | 0 << ICNC3                               /* Input Capture Noise Canceler: disabled */
	         | 0 << ICES3                               /* Input Capture Edge Select: disabled */
	         | (0 << CS32) | (1 << CS31) | (1 << CS30); /* IO clock divided by 64 */

    /* Input capture value, used as top counter value in some modes: 0 */
	// ICR3 = 0; 

	OCR3A = PWM_TOP; /* Output compare A: 100 */

	OCR3B = 64; /* Output compare B: 64 */

	TIMSK3 = 1 << OCIE3B   /* Output Compare B Match Interrupt Enable: enabled */
	         | 1 << OCIE3A /* Output Compare A Match Interrupt Enable: enabled */
	         | 0 << ICIE3  /* Input Capture Interrupt Enable: disabled */
	         | 0 << TOIE3; /* Overflow Interrupt Enable: disabled */

	return 0;
}

void TIMER_1_set_duty(uint8_t duty)
{
	if (duty >= PWM_TOP) duty = PWM_TOP - 1;
	OCR3B = duty;
}

void TIMER_1_stop()
{

	TCCR3B = (0 << WGM33) | (1 << WGM32) /* TC16 Mode 4 CTC */
			 | 0 << ICNC3 /* Input Capture Noise Canceler: disabled */
			 | 0 << ICES3 /* Input Capture Edge Select: disabled */
			 | (0 << CS32) | (0 << CS31) | (0 << CS30); /* No clock source (Timer/Counter stopped) */
}

/**
 * \brief Initialize TIMER_2 interface
 */
int8_t TIMER_2_init()
{
	/* Enable TC4 */
	PRR1 &= ~(1 << PRTIM4);

	// TCCR4A = (0 << COM4A1) | (0 << COM4A0) /* Normal port operation, OCA disconnected */
	//		 | (0 << COM4B1) | (0 << COM4B0) /* Normal port operation, OCB disconnected */
	//		 | (0 << WGM41) | (0 << WGM40); /* TC16 Mode 0 Normal */

	TCCR4B = (0 << WGM43) | (0 << WGM42)                /* TC16 Mode 0 Normal */
	         | 0 << ICNC4                               /* Input Capture Noise Canceler: disabled */
	         | 0 << ICES4                               /* Input Capture Edge Select: disabled */
	         //| (0 << CS42) | (1 << CS41) | (1 << CS40); /* IO clock divided by 64 */
			 | (1 << CS42) | (0 << CS41) | (0 << CS40); /* IO clock divided by 256 */

	// ICR4 = 0; /* Input capture value, used as top counter value in some modes: 0 */

	// OCR4A = 0; /* Output compare A: 0 */

	// OCR4B = 0; /* Output compare B: 0 */

	TIMSK4 = 0 << OCIE4B   /* Output Compare B Match Interrupt Enable: disabled */
	         | 0 << OCIE4A /* Output Compare A Match Interrupt Enable: disabled */
	         | 0 << ICIE4  /* Input Capture Interrupt Enable: disabled */
	         | 1 << TOIE4; /* Overflow Interrupt Enable: enabled */
	return 0;
}

void TIMER_2_reset()
{
    //reset the counter, clear the timer's count register (TCNTx).

    TCNT4 = 0;	
}

void TIMER_2_stop()
{

	TCCR4B = (0 << WGM43) | (1 << WGM42) /* TC16 Mode 0 Normal */
	| 0 << ICNC4 /* Input Capture Noise Canceler: disabled */
	| 0 << ICES4 /* Input Capture Edge Select: disabled */
	| (0 << CS42) | (0 << CS41) | (0 << CS40); /* No clock source (Timer/Counter stopped) */
}