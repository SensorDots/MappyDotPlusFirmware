/**
   MappyDot Firmware - addr.h

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


#ifndef ADDR_H_
#define ADDR_H_

#include <stdbool.h>

//Valid i2c addresses
#define START_ADDRESS 0x08
#define END_ADDRESS 0x78

int16_t addr_init(bool is_master);



#endif /* ADDR_H_ */