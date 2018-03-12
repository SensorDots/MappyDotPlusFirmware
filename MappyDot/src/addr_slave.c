/**
   MappyDot Firmware - addr_slave.c

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

   Loosely based on the Dallas 1wire protocol:
   http://www.atmel.com/images/Atmel-2579-Dallas-1Wire-Master-on-tinyAVR-and-megaAVR_ApplicationNote_AVR318.pdf
   Timings are from the recommended values.
   No CRC checking is performed as we perform address verification.
   ADDR_IN pin should not have pullups set during board initialisation.
   ADDR_IN line is initially set no pullup input with init function. In the
   This is because the master will wait for pullup. Once master detects pullup from the slave,
   it initialises the reset procedure and the slave then waits for the low from the master.

*/

#include "addr_slave.h"
#include "port.h"
#include "atmel_start_pins.h"
#include <util/delay.h>


/**
 * \brief Init function. Sets/resets the initial pin states.
 * 
 * \param 
 * 
 * \return void
 */
void addr_slave_init(void)
{
    /* Set pin direction to input */
    ADDR_IN_set_dir(PORT_DIR_IN);
    /* Slave has no pullup (yet) */
    ADDR_IN_set_pull_mode(
        PORT_PULL_OFF);
    //Debug set pin output to check when init starts.
    //SYNC_set_dir(PORT_DIR_OUT);
    //SYNC_set_level(false);
}

/**
 * \brief Wait for bus rising edge (not interrupt driven)
 * 
 * \param retries
 * 
 * \return bool
 */
bool wait_for_rising_edge(uint32_t retries)
{
    bool transition_up = 0;
    ADDR_IN_set_dir(PORT_DIR_IN);
    bool now = ADDR_IN_get_level();
    bool last = now;

    while (!transition_up)
    {
        now = ADDR_IN_get_level();

        /* Check if transition */
        if (now != last)
        {
            /* Check if rising edge */
            if (now > 0)
            {
                transition_up = 1;
            }

            last = now;
        }

        if (retries == 0)
        {
            sei();
            return 0;
        }

        retries--;
        //_delay_us(2);
    }

    return 1;
}

/**
 * \brief Wait for bus falling edge (not interrupt driven)
 * 
 * \param retries
 * 
 * \return bool
 */
bool wait_for_falling_edge(uint32_t retries)
{
    bool transition_down = 0;
    ADDR_IN_set_dir(PORT_DIR_IN);
    bool now = ADDR_IN_get_level();
    bool last = now;

    while (!transition_down)
    {
        now = ADDR_IN_get_level();

        /* Check if transition */
        if (now != last)
        {
            /* Check if falling edge */
            if (now < 1)
            {
                transition_down = 1;
            }

            last = now;
        }

        if (retries == 0)
        {
            sei();
            return 0;
        }

        retries--;
        //_delay_us(2);
    }

    return 1;
}

/**
 * \brief Response function (Assert Presence)
 * 
 * \param 
 * 
 * \return uint8_t
 */
uint8_t addr_slave_response(void)
{
    /* wait for 1 second as addressing cycles through all devices */
    //TODO: change to constant wait with LED breath.
    uint32_t retries = 500000;
    /* Disable interrupts */
    cli();
    /* Slave has pullup - slave is ready for master */
    ADDR_IN_set_dir(PORT_DIR_IN);
    ADDR_IN_set_pull_mode(
        PORT_PULL_UP);

    //Debug set pin output to check when init starts.
    //SYNC_set_dir(PORT_DIR_OUT);
    //SYNC_set_level(true);

    /* Wait for line to go from low to high */
    /* Manual loop, as we disable interrupts at this stage. */
    if (wait_for_rising_edge(retries))
    {
        /* Presence Delay */
        _delay_us(32);
        /* Drive bus low */
        ADDR_IN_set_dir(PORT_DIR_OUT);
        ADDR_IN_set_level(false);
        /* Presence */
        _delay_us(140);
        /* Release bus */
        ADDR_IN_set_dir(PORT_DIR_IN);
        ADDR_IN_set_pull_mode(
            PORT_PULL_UP);
    }

    /* Restore interrupts */
    sei();
    return 1;
}

/**
 * \brief Slave Write bit
 * 
 * \param bit
 * 
 * \return int8_t
 */
int8_t addr_slave_write_bit(uint8_t bit)
{
    uint32_t retries = 250000;
    /* Disable interrupts */
    cli();
    /* Release bus */
    ADDR_IN_set_dir(PORT_DIR_IN);
    ADDR_IN_set_pull_mode(
        PORT_PULL_UP);

    /* Wait for line to go low */
    if (wait_for_falling_edge(retries))
    {
        if (bit & 1)
        {
            _delay_us(45);
        }

        else
        {
            /* Drive bus low */
            ADDR_IN_set_dir(PORT_DIR_OUT);
            ADDR_IN_set_level(false);
            _delay_us(45);
            /* Release bus */
            ADDR_IN_set_dir(PORT_DIR_IN);
            ADDR_IN_set_pull_mode(
                PORT_PULL_UP);
        }
    }

    /* Restore interrupts */
    sei();
    return 1;
}

/**
 * \brief Slave read bit
 * 
 * \param 
 * 
 * \return uint8_t
 */
uint8_t addr_slave_read_bit(void)
{
    uint8_t response = 0;
    uint32_t retries = 250000;
    /* Disable interrupts */
    cli();
    /* Release bus */
    ADDR_IN_set_dir(PORT_DIR_IN);
    ADDR_IN_set_pull_mode(
        PORT_PULL_UP);

    /* Wait for line to go low */
    if (wait_for_falling_edge(retries))
    {
        /* Delay */
        _delay_us(45);
        /* Read bus state */
        response = ADDR_IN_get_level();
    }

    /* Restore interrupts */
    sei();
    return response;
}

/**
 * \brief Write byte
 * 
 * \param byte_to_write
 * 
 * \return void
 */
void addr_slave_write_byte(uint8_t byte_to_write)
{
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        addr_slave_write_bit( (bitMask & byte_to_write)?1:0);
    }
}

/**
 * \brief Slave read byte
 * 
 * 
 * \return uint8_t
 */
uint8_t addr_slave_read_byte()
{
    uint8_t bitMask;
    uint8_t response = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        if ( addr_slave_read_bit()) response |= bitMask;
    }

    return response;
}