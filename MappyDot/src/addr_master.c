/**
   MappyDot Firmware - addr_master.c

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
   ADDR_OUT pin should not have pullups set during board initialisation.
   ADDR_OUT line is initially set to input with no pullup with init function.

*/

#include "addr_master.h"
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
void addr_master_init(void)
{
    /* Set pin direction to output */
    ADDR_OUT_set_dir(PORT_DIR_IN);
    /* Master has no pull up on init*/
    ADDR_OUT_set_pull_mode(
        PORT_PULL_OFF);
}


/**
 * \brief Reset function (Detect Presence)
 * 
 * \param 
 * 
 * \return uint8_t - Return 1 if response, 0 if no device connected.
 * Waits up to 1s for response due to bus
 * address prorogation (112 devices @ ~1ms each). If there's no response then there's no
 * more devices in chain.
 */
uint8_t addr_master_reset(void)
{
    uint8_t response;
    uint32_t retries = 500000;
    /* Disable interrupts */
    cli();
    /* Wait for line to go high */
    ADDR_OUT_set_dir(PORT_DIR_IN);

    do
    {
        if (retries == 0) return 0;

        retries--;
        _delay_us(2);
    }
    while ( !ADDR_OUT_get_level());

    /* Drive bus low */
    ADDR_OUT_set_dir(PORT_DIR_OUT);
    ADDR_OUT_set_level(false);
    /* Delay H */
    _delay_us(480);
    /* Release bus */
    ADDR_OUT_set_dir(PORT_DIR_IN);
    ADDR_OUT_set_pull_mode(
        PORT_PULL_UP);
    /* Delay I */
    _delay_us(70);
    /* Read bus state */
    response = !ADDR_OUT_get_level();
    /* Delay J */
    _delay_us(410);
    /* Restore interrupts */
    sei();
    return response;
}


/**
 * \brief Write 1Wire protocol bit
 * 
 * \param bit - value of state
 * 
 * \return void
 */
void addr_master_write_bit(uint8_t bit)
{
    /* Disable interrupts */
    cli();
    /* Drive bus low */
    ADDR_OUT_set_dir(PORT_DIR_OUT);
    ADDR_OUT_set_level(false);

    if (bit & 1)
    {
        /* Delay A */
        _delay_us(6);
        /* Release bus */
        ADDR_OUT_set_dir(PORT_DIR_IN);
        /* Delay B */
        _delay_us(64);
    }

    else
    {
        /* Delay C */
        _delay_us(60);
        /* Release bus */
        ADDR_OUT_set_dir(PORT_DIR_IN);
        ADDR_OUT_set_pull_mode(
            PORT_PULL_UP);
        /* Delay D */
        _delay_us(10);
    }

    /* Restore interrupts */
    sei();
}


/**
 * \brief Read 1Wire Protocol bit
 * 
 * \param 
 * 
 * \return uint8_t
 */
uint8_t addr_master_read_bit(void)
{
    uint8_t response;
    /* Disable interrupts */
    cli();
    /* Drive bus low */
    ADDR_OUT_set_dir(PORT_DIR_OUT);
    ADDR_OUT_set_level(false);
    /* Delay A */
    _delay_us(6);
    /* Release bus */
    ADDR_OUT_set_dir(PORT_DIR_IN);
    ADDR_OUT_set_pull_mode(
        PORT_PULL_UP);
    /* Delay E */
    _delay_us(9);
    /* Read bus state */
    response = ADDR_OUT_get_level();
    /* Delay F */
    _delay_us(55);
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
void addr_master_write_byte(uint8_t byte_to_write)
{
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        addr_master_write_bit( (bitMask & byte_to_write)?1:0);
    }
}

/**
 * \brief Read byte
 * 
 * 
 * \return uint8_t
 */
uint8_t addr_master_read_byte()
{
    uint8_t bitMask;
    uint8_t response = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        if ( addr_master_read_bit()) response |= bitMask;
    }

    return response;
}