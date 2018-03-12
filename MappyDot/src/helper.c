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
#include "vl53l0x_types.h"
#include "vl53l0x_profiles.h"
#include "nvmctrl.h"

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
    if (ranging_mode == SET_CONTINUOUS_RANGING_MODE) return 1;

	//if (ranging_mode == SET_SINGLE_RANGING_MODE) //Set by default
	return 0;
}

/**
 * \brief Translates the human readable measurement mode to API mode.
 * 
 * \param measurement_mode
 * 
 * \return uint8_t
 */
void translate_measurement_mode(uint8_t measurement_mode, VL53L0X_Measurement_Mode *mode_settings, uint8_t * custom_profile_settings)
{
    if (measurement_mode == CUSTOM) {
	    //Check if mode settings are set else measurement_mode = DEFAULT

		uint8_t n = 9;
		while(--n>0 && custom_profile_settings[n] == 0);

		if (n == 0) 
		{ 
		    measurement_mode = VL53L0X_DEFAULT;
		}
		else
		{
		    /* For custom profile storage:
		    RANGE_IGNORE_THRESHOLD       uint8_t  (store on/off)
		    RANGE_IGNORE_THRESHOLD_VALUE uint16_t (store final calculated value of 1.5*0.0230*65536) Value NOT multiplied by 65536 in software
		    SIGNAL_RATE_FINAL_RANGE      uint8_t  (store Decimal values 2dp) Value divided by 100 and multiplied by 65536 in software
		    SIGMA_FINAL_RANGE            uint8_t  (store value) Value multiplied by 65536 in software
		    TIMING_BUDGET                uint16_t (store time in ms) Value multiplied by 1000 to get microseconds in software
		    VCSEL_PERIOD_PRE_RANGE       uint8_t  (store value)
		    VCSEL_PERIOD_FINAL_RANGE     uint8_t  (store value)
		    */

		    /* This is sent as one long profile string as bytes
		    | byte 0 | byte 1 | byte 2 | byte 3 | byte 4 | byte 5 | byte 6 | byte 7 | byte 8 |
		    | THRESH | THRESHOLD_VALUE | SR_F_R | SM_F_R |  TIMING_BUDGET  | VP_P_R | VP_F_R |

		    */
			mode_settings->range_ignore_threshold =                 (custom_profile_settings[0] & 0x01);
			mode_settings->range_ignore_threshold_value = (uint32_t)((uint32_t)custom_profile_settings[1] << 8 | custom_profile_settings[2]);
			mode_settings->signal_rate_final_range =      (uint32_t)(custom_profile_settings[3]) * 65536 / 100;
			mode_settings->sigma_final_range =            (uint32_t)(custom_profile_settings[4]) * 65536;
			mode_settings->timing_budget_ms =             (uint32_t)((uint32_t)custom_profile_settings[5] << 8 | custom_profile_settings[6]) * 1000;
			mode_settings->vcsel_period_pre_range =                  custom_profile_settings[7];
			mode_settings->vscel_period_final_range =                custom_profile_settings[8];
			return;
		}

	    
    }
    
    if (measurement_mode == HIGHLY_ACCURATE) {
	    mode_settings->range_ignore_threshold = HIGH_ACCURACY_RANGE_IGNORE_THRESHOLD;
		mode_settings->range_ignore_threshold_value = 0;
		mode_settings->signal_rate_final_range = HIGH_ACCURACY_SIGNAL_RATE_FINAL_RANGE;
		mode_settings->sigma_final_range = HIGH_ACCURACY_SIGMA_FINAL_RANGE;
		mode_settings->timing_budget_ms = HIGH_ACCURACY_TIMING_BUDGET;
		mode_settings->vcsel_period_pre_range = HIGH_ACCURACY_VCSEL_PERIOD_PRE_RANGE;
		mode_settings->vscel_period_final_range = HIGH_ACCURACY_VCSEL_PERIOD_FINAL_RANGE;
		return;
	}

    if (measurement_mode == LONG_RANGE) {
	    mode_settings->range_ignore_threshold = LONG_RANGE_RANGE_IGNORE_THRESHOLD;
	    mode_settings->range_ignore_threshold_value = 0;
		mode_settings->signal_rate_final_range = LONG_RANGE_SIGNAL_RATE_FINAL_RANGE;
	    mode_settings->sigma_final_range = LONG_RANGE_SIGMA_FINAL_RANGE;
	    mode_settings->timing_budget_ms = LONG_RANGE_TIMING_BUDGET;
	    mode_settings->vcsel_period_pre_range = LONG_RANGE_VCSEL_PERIOD_PRE_RANGE;
	    mode_settings->vscel_period_final_range = LONG_RANGE_VCSEL_PERIOD_FINAL_RANGE;
		return;
	}

    if (measurement_mode == HIGH_SPEED) 
	{
	    mode_settings->range_ignore_threshold = HIGH_SPEED_RANGE_IGNORE_THRESHOLD;
	    mode_settings->range_ignore_threshold_value = 0;
		mode_settings->signal_rate_final_range = HIGH_SPEED_SIGNAL_RATE_FINAL_RANGE;
	    mode_settings->sigma_final_range = HIGH_SPEED_SIGMA_FINAL_RANGE;
	    mode_settings->timing_budget_ms = HIGH_SPEED_TIMING_BUDGET;
	    mode_settings->vcsel_period_pre_range = HIGH_SPEED_VCSEL_PERIOD_PRE_RANGE;
	    mode_settings->vscel_period_final_range = HIGH_SPEED_VCSEL_PERIOD_FINAL_RANGE;
	    return;
	}

	mode_settings->range_ignore_threshold = DEFAULT_RANGE_IGNORE_THRESHOLD;
	mode_settings->range_ignore_threshold_value = DEFAULT_CHECKENABLE_RANGE_IGNORE_THRESHOLD_VALUE;
	mode_settings->signal_rate_final_range = DEFAULT_SIGNAL_RATE_FINAL_RANGE;
	mode_settings->sigma_final_range = DEFAULT_SIGMA_FINAL_RANGE;
	mode_settings->timing_budget_ms = DEFAULT_TIMING_BUDGET;
	mode_settings->vcsel_period_pre_range = DEFAULT_VCSEL_PERIOD_PRE_RANGE;
	mode_settings->vscel_period_final_range = DEFAULT_VCSEL_PERIOD_FINAL_RANGE;

	return;
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