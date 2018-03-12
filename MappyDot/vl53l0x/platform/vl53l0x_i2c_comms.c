/**
   MappyDot Firmware - vl53l0x_i2c_comms.c

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

#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"
#include "i2c_master.h"

int VL53L0X_i2c_init(void) {
  i2c_init();
  return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, uint32_t count) {
  
   if (i2c_start(address | 0x00))  { return VL53L0X_ERROR_UNDEFINED;}

    i2c_write(index);

    for (uint16_t i = 0; i < count; i++)
    {
	    if (i2c_write(pdata[i])) {return VL53L0X_ERROR_UNDEFINED;}
    }

    i2c_stop();

  return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, uint32_t count) {
	if (i2c_start(address)) {return VL53L0X_ERROR_UNDEFINED;}

	i2c_write(index);

	if (i2c_start(address | 0x01)) {return VL53L0X_ERROR_UNDEFINED;};

	for (uint16_t i = 0; i < (count-1); i++)
	{
		pdata[i] = i2c_read_ack();
	}
	pdata[(count-1)] = i2c_read_nack();

	i2c_stop();

  return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data) {
  uint8_t data_t[1] = {0};
  data_t[0] = data;
  
  if (i2c_writeReg(address,index,data_t,1)) return VL53L0X_ERROR_CONTROL_INTERFACE;

  return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data) {

  uint8_t data_t[2] = {0};
  data_t[0] = (data >> 8) & 0xFF;
  data_t[1] =  data       & 0xFF;
  	if (i2c_writeReg(address,index,data_t,2)) return VL53L0X_ERROR_CONTROL_INTERFACE;

  return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data) {

  	uint8_t data_t[4] = {0};
  	data_t[0] = (data >> 24) & 0xFF; // value highest byte
  	data_t[1] = (data >> 16) & 0xFF;
  	data_t[2] = (data >>  8) & 0xFF;
  	data_t[3] =  data        & 0xFF;

	if (i2c_writeReg(address,index,data_t,4)) return VL53L0X_ERROR_CONTROL_INTERFACE;

  return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *data) {

	uint8_t data_t[1] = {0};
	if (i2c_readReg(address,index,data_t,1)) return VL53L0X_ERROR_CONTROL_INTERFACE;
	uint8_t tmp = data_t[0];
	*data = tmp;

  return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *data) {

    uint8_t data_t[2] = {0};
    if (i2c_readReg(address,index,data_t,2)) return VL53L0X_ERROR_CONTROL_INTERFACE;
      
    uint16_t value;

    value  = (uint16_t)data_t[0] << 8; // value high byte
    value |=           data_t[1];      // value low byte

    *data = value;

	return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *data) {

  	uint8_t data_t[4] = {0};
	if (i2c_readReg(address,index,data_t,4)) return VL53L0X_ERROR_CONTROL_INTERFACE;
  	
  	uint32_t value;

  	value  = (uint32_t)data_t[0] << 24; // value highest byte
  	value |= (uint32_t)data_t[1] << 16;
  	value |= (uint32_t)data_t[2] <<  8;
  	value |=           data_t[3];       // value lowest byte

	*data = value;

  	return VL53L0X_ERROR_NONE;
}
