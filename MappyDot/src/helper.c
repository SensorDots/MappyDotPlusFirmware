/**
   MappyDot Firmware - helper.c

   Copyright (C) 2017 SensorDots.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <atmel_start.h>
#include <avr/wdt.h>
#include <math.h>
#include <string.h>
#include <util/delay.h>
#include "helper.h"
#include "mappydot_reg.h"
#include "vl53l1_types.h"
#include "nvmctrl.h"
#include "crc8.h"

/**
 * \brief _delay_ms helper function
 * 
 * \param ms
 * 
 * \return void
 */
void delay_ms(uint16_t ms)
{
	while (0 < ms)
	{
		_delay_ms(1);
		--ms;
	}
}

/**
 * \brief Stores the current address to EEPROM
 *
 * \param slave_address
 *
 * \return void
 */
void store_current_address_eeprom(uint8_t eeprom_address, uint8_t slave_address)
{
    /* Only store if stored address is different to current address */
    if (FLASH_0_read_eeprom_byte(eeprom_address) != slave_address)
	{
		/* Store current slave address in eeprom */
		FLASH_0_write_eeprom_byte(eeprom_address, slave_address);
	}
}

/**
 * \brief Average function
 * 
 * \param array
 * \param count
 * 
 * \return uint16_t
 */
uint16_t avg(uint16_t * array, uint8_t count)
{
	uint16_t sum = 0;

	for(uint8_t i = 0; i < count; i++)
	{
		sum += array[i];
	}

	return sum / count;
}


/**
 * \brief Flash LED
 * 
 * \param flash_ms between state chances, double this value for the period
 * \param num_of_flashes (-1 for indefinite)
 * 
 * \return void
 */
void flash_led(uint32_t timeout_ms, int8_t num_of_flashes, bool pwm)
{
    /* Turn off LED */
    LED_set_level(true);

	delay_ms(timeout_ms);

	//Can only breath in indefinite mode
    if (pwm && num_of_flashes == -1)
	{
		uint8_t i = 0;
		TIMER_1_init();

		while (1)
		{	
			if (i >= 99) i = 0;
			TIMER_1_set_duty(i);
			delay_ms(timeout_ms);
			i++;
		}
		
	}
	while (abs(num_of_flashes) > 0)
    {
	    delay_ms(timeout_ms);

		/* Turn on LED */
		LED_set_level(false);

		delay_ms(timeout_ms);

		/* Turn off LED */
		LED_set_level(true);

		/* Never decrement if -1 */
		if (num_of_flashes > 0) num_of_flashes--;
    }
}

/**
 * \brief Translates the ranging mode for API mode
 * 
 * \param ranging_mode
 * 
 * \return uint8_t
 */
uint8_t translate_ranging_mode(uint8_t ranging_mode)
{
    if (ranging_mode == SET_CONTINUOUS_RANGING_MODE) return 0;

	//if (ranging_mode == SET_SINGLE_RANGING_MODE) //Set by default
	return 1;
}

/* Duty Cycle point */
ISR(TIMER1_COMPB_vect)
{
	SYNC_set_level(false);
}

/* LED PWM Timer (TIMER_1) */
ISR(TIMER3_COMPA_vect)
{
	/* Turn on LED */
	LED_set_level(false);

}

/* Duty Cycle point */
ISR(TIMER3_COMPB_vect)
{
	/* Turn off LED */
	LED_set_level(true);

}

/* GPIO PWM Timer (TIMER_0) */
/* "Start" of the PWM signal. While COMPA resets after interrupt,
 * we can never get full off with this because there is always time.
 * until the next interrupt fire */
ISR(TIMER1_COMPA_vect)

{
    SYNC_set_level(true);
}

/* Debugging purposes */
ISR(BADISR_vect)
{
	/* Turn on LED */
	//LED_set_level(true);
	asm("nop;");
}