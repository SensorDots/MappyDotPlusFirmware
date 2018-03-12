/**
   MappyDot Firmware - addr.c

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

#include "addr_master.h"
#include "addr_slave.h"
#include "addr.h"
#include <stdbool.h>
#include "port.h"
#include "atmel_start_pins.h"
#include <util/delay.h>

//Returns the address of the device, if negative then invalid/timeout

int16_t own_address = -1;

/**
 * \brief Start addressing for next device in chain
 * 
 * 
 * \return bool
 */
bool address_next()
{
    /* Initialise address procedure with incremented address */
    addr_master_init();

    /* Perform detect presence procedure */
    if (addr_master_reset())
    {
        /* Send incremented address */
        addr_master_write_byte(own_address + 1);

        if (addr_master_read_byte() == own_address + 1)
        {
            addr_master_write_byte(0x01);
            return 1;
        }

        else
        {
            /* Restart procedure once more if address fail */
            addr_master_write_byte(0xfe);
            return 0;
        }
    }

    return 0;
}

/**
 * \brief Starts the address initilisation procedure
 * 
 * \param is_master - if current device is the master
 * 
 * \return int16_t
 */
int16_t addr_init(bool is_master)
{
    /* Check if is first in chain */
    if (is_master)
    {
        own_address = START_ADDRESS;

        /* If slave fail, retry (especially for ICP debugger as it holds chip idle for a bit) */
        if(!address_next())
        {
            //Delay for 400ms
            _delay_us(400000);
            address_next();
        }
    }

    else
    {
        /* Slave addressing init */
        addr_slave_init();

        /* Wait for response from master device. */
        /* If no response just fall through. */
        if (addr_slave_response())
        {
            /* Get address from previous device in chain */
            own_address = addr_slave_read_byte();
            //SYNC_set_dir(PORT_DIR_OUT);
            //SYNC_set_level(true);

            /* Send verification address */
            addr_slave_write_byte(own_address);

            //SYNC_set_dir(PORT_DIR_OUT);
            //SYNC_set_level(false);

            /* If get address */
            if (addr_slave_read_byte() == 0x01)
            {
                /* Initialise address procedure with incremented address */
                if(!address_next())
                {
                    //Delay for 400ms
                    _delay_us(400000);
                    address_next();
                }
            }

            else
            {
                own_address = -1;
            }

            //SYNC_set_dir(PORT_DIR_OUT);
            //SYNC_set_level(true);
        }
    }

    /* Return own address or -1 if no address timeout */
    return own_address;
}
