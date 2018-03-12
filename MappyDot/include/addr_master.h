/**
   MappyDot Firmware - addr_master.h

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


#ifndef ADDR_MASTER_H_
#define ADDR_MASTER_H_

#include <stdint.h>

/* Init function. Sets/resets the initial pin states. */
void addr_master_init(void);

/* Reset function (Detect Presence) */
uint8_t addr_master_reset(void);

/* Write bit */
void addr_master_write_bit(uint8_t bit);

/* Read bit */
uint8_t addr_master_read_bit(void);

/* Write byte */
void addr_master_write_byte(uint8_t byte_to_write);

/* Read byte */
uint8_t addr_master_read_byte();

#endif /* ADDR_MASTER_H_ */