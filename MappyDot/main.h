/**
   MappyDot Firmware - main.h

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


#ifndef MAIN_H_
#define MAIN_H_


/* Command Callbacks */

//Process receive command, the command length is assumed
void main_process_rx_command(uint8_t command, uint8_t * arg, uint8_t arg_length);

//Returns number of bytes to send
uint8_t main_process_tx_command(uint8_t command, uint8_t * arg);




#endif /* MAIN_H_ */