/*
 * nvmctrl.c
 *
 * Created: 27/08/2017 12:17:27 AM
 *  Author: Blair Wyatt
 */
/**
* \file
*
* \brief NVMCTRL Basic driver implementation.
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

#include <nvmctrl.h>

/* clang-format off */
#if defined(__ICCAVR__)

#define _GET_LOCK_BITS() __AddrToZByteToSPMCR_LPM( (void __flash *) 0x0001, 0x09 )
#define _GET_LOW_FUSES() __AddrToZByteToSPMCR_LPM( (void __flash *) 0x0000, 0x09 )
#define _GET_HIGH_FUSES() __AddrToZByteToSPMCR_LPM( (void __flash *) 0x0003, 0x09 )
#define _GET_EXTENDED_FUSES() __AddrToZByteToSPMCR_LPM( (void __flash *) 0x0002, 0x09 )
#define _SET_LOCK_BITS(data) __DataToR0ByteToSPMCR_SPM( data, 0x09 )
#define _ENABLE_RWW_SECTION() __DataToR0ByteToSPMCR_SPM( 0x00, 0x11 )

#define _WAIT_FOR_SPM() while( SPMCSR & (1<<SPMEN) );

#ifndef CONFIG_MEMORY_MODEL_LARGE
#define _LOAD_PROGRAM_MEMORY(addr) __load_program_memory( (const unsigned char __flash *) (addr) )
#define _FILL_TEMP_WORD(addr,data) __AddrToZWordToR1R0ByteToSPMCR_SPM( (void __flash *) (addr), data, 0x01 )
#define _PAGE_ERASE(addr) __AddrToZByteToSPMCR_SPM( (void __flash *) (addr), 0x03 )
#define _PAGE_WRITE(addr) __AddrToZByteToSPMCR_SPM( (void __flash *) (addr), 0x05 )
#else
#define _LOAD_PROGRAM_MEMORY(addr) __extended_load_program_memory( (const unsigned char __farflash *) (addr) )
#define _FILL_TEMP_WORD(addr,data) __AddrToZ24WordToR1R0ByteToSPMCR_SPM( (void __farflash *) (addr), data, 0x01 )
#define _PAGE_ERASE(addr) __AddrToZ24ByteToSPMCR_SPM( (void __farflash *) (addr), 0x03 )
#define _PAGE_WRITE(addr) __AddrToZ24ByteToSPMCR_SPM( (void __farflash *) (addr), 0x05 )
#endif

#endif

#if defined(__GNUC__)

#define	_FLASH_WRITE_PAGE          boot_page_write
#define	_FLASH_ERASE_PAGE          boot_page_erase
#define _FLASH_ENABLE_RWW_SECTION  boot_rww_enable
#define _FLASH_WRITE_PAGEBUF       boot_page_fill
#define _FLASH_READ                pgm_read_byte_near
#define _FLASH_WAIT_SPM()          boot_spm_busy_wait()

#elif defined(__ICCAVR__)

#define	_FLASH_WRITE_PAGE          _PAGE_WRITE
#define	_FLASH_ERASE_PAGE          _PAGE_ERASE
#define _FLASH_ENABLE_RWW_SECTION  _ENABLE_RWW_SECTION
#define _FLASH_WRITE_PAGEBUF       _FILL_TEMP_WORD
#define _FLASH_READ                _LOAD_PROGRAM_MEMORY
#define _FLASH_WAIT_SPM()          _WAIT_FOR_SPM()

#else
#error Unsupported compiler.
#endif
/* clang-format on */

int8_t FLASH_0_init()
{
    EECR = 0 << EERE /* EEPROM Read Enable: disabled */
           | 0 << EEPE /* EEPROM Write Enable: disabled */
           | 0 << EEMPE /* EEPROM Master Write Enable: disabled */
           | 0 << EERIE /* EEPROM Ready Interrupt Enable: disabled */
           | 0 << EEPM0 /* EEPROM Programming mode 1: disabled */ //erase/write
           | 0 << EEPM1; /* EEPROM Programming mode 0: disabled */ //erase
    //SPMCSR = 0 << SPMEN    /* SPM Enable: disabled */
    //         | 0 << PGERS  /* Page Erase: disabled */
    //         | 0 << PGWRT  /* Page Write: disabled */
    //         | 1 << BLBSET /* Boot Lock Bit Set: enabled */
    //         | 0 << RWWSRE /* Read-While-Write Section Enable: disabled */
    //         | 0 << SIGRD  /* Signature Row Read: disabled */
    //         | 0 << RWWSB  /* Read-While-Write Busy: disabled */
    //         | 0 << SPMIE; /* SPM Interrupt Enable: disabled */
    return 0;
}

uint8_t FLASH_0_read_eeprom_byte(eeprom_adr_t eeprom_adr)
{
    // Wait until any EEPROM write has completed
    while (EECR & (1 << EEPE))
        ;

    /* Set up address register */
    EEAR = eeprom_adr;
    /* Start eeprom read by writing EERE */
    EECR |= (1 << EERE);
    /* Return data from Data Register */
    return EEDR;
}

nvmctrl_status_t FLASH_0_write_eeprom_byte(eeprom_adr_t eeprom_adr, uint8_t data)
{
    /* Wait for completion of previous write */
    while (EECR & (1 << EEPE))
        ;

    /* Set up address and Data Registers */
    EEAR = eeprom_adr;
    EEDR = data;
    ENTER_CRITICAL(WRITE_BYTE);
    /* Write logical one to EEMPE */
    EECR |= (1 << EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1 << EEPE);
    EXIT_CRITICAL(WRITE_BYTE);
    return NVM_OK;
}

void FLASH_0_read_eeprom_block(eeprom_adr_t eeprom_adr, uint8_t *data, size_t size)
{
    uint8_t i;

    for (i = 0; i < size; i++)
    {
        *data++ = FLASH_0_read_eeprom_byte(eeprom_adr++);
    }
}

nvmctrl_status_t FLASH_0_write_eeprom_block(eeprom_adr_t eeprom_adr, uint8_t *data, size_t size)
{
    uint8_t i;

    for (i = 0; i < size; i++)
    {
        FLASH_0_write_eeprom_byte(eeprom_adr++, *data++);
    }

    return NVM_OK;
}

bool FLASH_0_is_eeprom_ready()
{
    return (EECR & (1 << EEPE));
}

/*uint8_t FLASH_0_read_flash_byte(flash_adr_t flash_adr)
{

	return _FLASH_READ(flash_adr);
}

nvmctrl_status_t FLASH_0_write_flash_byte(flash_adr_t flash_adr, uint8_t *ram_buffer, uint8_t data)
{

	uint8_t  i;
	uint16_t write_adr = (flash_adr & ~(SPM_PAGESIZE - 1));

	for (i = 0; i < SPM_PAGESIZE; i++) {
		ram_buffer[i] = FLASH_0_read_flash_byte(write_adr + i);
	}

	// Write new byte into correct location in ram_buffer
	ram_buffer[flash_adr % SPM_PAGESIZE] = data;
	FLASH_0_erase_flash_page(write_adr);
	FLASH_0_write_flash_page(write_adr, ram_buffer);
	return NVM_OK;
}

nvmctrl_status_t FLASH_0_erase_flash_page(flash_adr_t flash_adr)
{

	_FLASH_ERASE_PAGE(flash_adr);
	_FLASH_WAIT_SPM();
	_FLASH_ENABLE_RWW_SECTION();
	return NVM_OK;
}

nvmctrl_status_t FLASH_0_write_flash_page(flash_adr_t flash_adr, uint8_t *data)
{

	uint8_t i;

	for (i = 0; i < SPM_PAGESIZE; i += 2) {
		// Set up little-endian word.
		uint16_t w = *data++;
		w += (*data++) << 8;

		_FLASH_WRITE_PAGEBUF(flash_adr + i, w);
	}
	_FLASH_WRITE_PAGE(flash_adr);
	_FLASH_WAIT_SPM();
	_FLASH_ENABLE_RWW_SECTION();

	return NVM_OK;
}

nvmctrl_status_t FLASH_0_write_flash_block(flash_adr_t flash_adr, uint8_t *data, size_t size, uint8_t *ram_buffer)
{

	// Get address of the start of first page to modify
	uint16_t write_adr    = flash_adr & ~(SPM_PAGESIZE - 1);
	uint8_t  start_offset = flash_adr % SPM_PAGESIZE;
	uint8_t  i;

	// Step 1:
	// Fill page buffer with contents of first flash page to be written up
	// to the first flash address to be replaced by the new contents
	for (i = 0; i < start_offset; i++) {
		ram_buffer[i] = FLASH_0_read_flash_byte(write_adr++);
	}

	// Step 2:
	// Write all of the new flash contents to the page buffer, writing the
	// page buffer to flash every time the buffer contains a complete flash
	// page.
	while (size > 0) {
		ram_buffer[i++] = *data++;
		write_adr++;
		size--;
		if ((write_adr % SPM_PAGESIZE) == 0) {
			// Erase and write the flash page
			FLASH_0_erase_flash_page(write_adr - SPM_PAGESIZE);
			FLASH_0_write_flash_page(write_adr - SPM_PAGESIZE, ram_buffer);
			i = 0;
		}
	}

	// Step 3:
	// After step 2, the page buffer may be partially full with the last
	// part of the new data to write to flash. The remainder of the flash page
	// shall be unaltered. Fill up the remainder
	// of the page buffer with the original contents of the flash page, and do a
	// final flash page write.
	while (1) {
		ram_buffer[i++] = FLASH_0_read_flash_byte(write_adr++);
		if ((write_adr % SPM_PAGESIZE) == 0) {
			// Erase and write the last flash page
			FLASH_0_erase_flash_page(write_adr - SPM_PAGESIZE);
			FLASH_0_write_flash_page(write_adr - SPM_PAGESIZE, ram_buffer);
			return NVM_OK;
		}
	}
}

nvmctrl_status_t FLASH_0_write_flash_stream(flash_adr_t flash_adr, uint8_t data, bool finalize)
{

	bool            final_adr_in_page = ((flash_adr & (SPM_PAGESIZE - 1)) == (SPM_PAGESIZE - 1));
	static uint16_t word_to_write;

	// Write the new byte value to the correct address in the page buffer
	if (flash_adr & 0x1) {
		word_to_write += data << 8;
		_FLASH_WRITE_PAGEBUF(flash_adr, word_to_write);
	} else {
		word_to_write = data;
	}

	if (final_adr_in_page || finalize) {
		// Erase page
		_FLASH_ERASE_PAGE(flash_adr);
		_FLASH_WAIT_SPM();

		// Write page
		_FLASH_WRITE_PAGE(flash_adr);
		_FLASH_WAIT_SPM();
		_FLASH_ENABLE_RWW_SECTION();
	}

	return NVM_OK;
}*/