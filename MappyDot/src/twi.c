/**
 * \file
 *
 * \brief TWI related functionality implementation.
 *
 *
 * Copyright (C) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *
 */

#include <twi.h>



/**
 * \brief Initialize twi interface
 */

void I2C_0_init()
{
    ///* Enable TWI0 */
    PRR0 &= ~(1 << PRTWI0);
    TWCR0 = (1 << TWEN)   /* TWI0: enabled */
            | (0 << TWIE) /* TWI0 Interrupt: disabled */
            | (0 << TWEA) /* TWI0 Acknowledge: disabled */;
    /* SCL bitrate = F_CPU / (16 + 2 * TWBR0 * TWPS value) */
    /* Configured bit rate is 400.000kHz, based on CPU frequency 8.000MHz */
    TWBR0 = 0x02;          /* SCL bit rate: 400.000kHZ before prescaling */
    TWSR0 = 0x00 << TWPS0; /* SCL precaler: 1, effective bitrate = 400.000kHz */

	/* Enable TWI0 */
	//PRR0 &= ~(1 << PRTWI0);
	//TWCR0 = (1 << TWEN)   /* TWI0: enabled */
	//| (0 << TWIE) /* TWI0 Interrupt: disabled */
	//| (0 << TWEA) /* TWI0 Acknowledge: disabled */;
	///* SCL bitrate = F_CPU / (16 + 2 * TWBR0 * TWPS value) */
	///* Configured bit rate is 100.000kHz, based on CPU frequency 8.000MHz */
	//TWBR0 = 0x20;          /* SCL bit rate: 100.000kHZ before prescaling */
	//TWSR0 = 0x00 << TWPS0; /* SCL precaler: 1, effective bitrate = 100.000kHz */
}

int8_t I2C_1_init()
{
    /* Enable TWI1 */
    PRR1 &= ~(1 << PRTWI1);
    TWCR1 = (1 << TWEN)   /* TWI1: enabled */
            | (1 << TWIE) /* TWI1 Interrupt: enabled */
            | (1 << TWEA) /* TWI1 Acknowledge: enabled */;
    TWAR1 = (0x07 << TWA0) /* TWI1 (Slave) Address: 7 */
            | (1 << TWGCE) /* TWI1 General Call Recognition: enabled */;
    TWAMR1 = (0x00 << TWAM0) /* TWI1 (Slave) Address Mask: 0 */;
    return 0;
}


