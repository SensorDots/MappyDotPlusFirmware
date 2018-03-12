/**
   MappyDot Firmware i2c_slave.c

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

#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H


#define I2C_BUFFER_SIZE 32

uint8_t register_address;
uint8_t register_address_reset;

uint8_t txbuffer[I2C_BUFFER_SIZE];
uint8_t rxbuffer[I2C_BUFFER_SIZE];
uint8_t txbuffer_count;
uint8_t rxbuffer_count;

void i2c_slave_init(uint8_t address);
void i2c_slave_stop(void);
ISR(TWI_vect);

#endif // I2C_SLAVE_H
