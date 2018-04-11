/**
   MappyDot Firmware - main_helper.h

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


#ifndef HELPER_H_
#define HELPER_H_

#include "vl53l1_types.h"

void flash_led(uint32_t timeout_ms, int8_t num_of_flashes, bool pwm);
void delay_ms(uint16_t ms);
#ifndef DEV_DISABLE
uint16_t avg(uint16_t * array, uint8_t count);
#endif
//void translate_measurement_mode(uint8_t measurement_mode, VL53L1_Measurement_Mode *mode_settings, uint8_t * custom_profile_settings);
uint8_t translate_ranging_mode(uint8_t ranging_mode);
void store_current_address_eeprom(uint8_t eeprom_address, uint8_t slave_address);

#endif /* HELPER_H_ */