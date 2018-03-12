/**
 * \file
 *
 * \brief NVMCTRL Basic driver declaration.
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

/**
 * \defgroup doc_driver_nvmctrl_basic NVMCTRL Basic Driver
 * \ingroup doc_driver_nvmctrl
 *
 * \section doc_driver_nvmctrl_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */

#ifndef _NVMCTRL_H_INCLUDED
#define _NVMCTRL_H_INCLUDED

#include <compiler.h>

#if defined(__GNUC__)
#include <avr/pgmspace.h>
#include <avr/boot.h>
#endif
#include <atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Datatype for flash address */
//typedef uint16_t flash_adr_t;

/** Datatype for EEPROM address */
typedef uint16_t eeprom_adr_t;

/** Datatype for return status of NVMCTRL operations */
typedef enum
{
    NVM_OK    = 0, ///< NVMCTRL free, no write ongoing.
    NVM_ERROR = 1, ///< NVMCTRL operation retsulted in error
    NVM_BUSY  = 2, ///< NVMCTRL busy, write ongoing.
} nvmctrl_status_t;

/**
 * \brief Initialize nvmctrl interface
 * \return Return value 0 if success
 */
int8_t FLASH_0_init();

/**
 * \brief Read a byte from eeprom
 *
 * \param[in] eeprom_adr The byte-address in eeprom to read from
 *
 * \return The read byte
 */
uint8_t FLASH_0_read_eeprom_byte(eeprom_adr_t eeprom_adr);

/**
 * \brief Write a byte to eeprom
 *
 * \param[in] eeprom_adr The byte-address in eeprom to write to
 * \param[in] data The byte to write
 *
 * \return Status of write operation
 */
nvmctrl_status_t FLASH_0_write_eeprom_byte(eeprom_adr_t eeprom_adr, uint8_t data);

/**
 * \brief Read a block from eeprom
 *
 * \param[in] eeprom_adr The byte-address in eeprom to read from
 * \param[in] data Buffer to place read data into
 *
 * \return Nothing
 */
void FLASH_0_read_eeprom_block(eeprom_adr_t eeprom_adr, uint8_t *data, size_t size);

/**
 * \brief Write a block to eeprom
 *
 * \param[in] eeprom_adr The byte-address in eeprom to write to
 * \param[in] data The buffer to write
 *
 * \return Status of write operation
 */
nvmctrl_status_t FLASH_0_write_eeprom_block(eeprom_adr_t eeprom_adr, uint8_t *data, size_t size);

/**
 * \brief Check if the EEPROM can accept data to be read or written
 *
 * \return The status of EEPROM busy check
 * \retval false The EEPROM can not receive data to be read or written
 * \retval true The EEPROM can receive data to be read or written
 */
bool FLASH_0_is_eeprom_ready();

/**
 * \brief Read a byte from flash
 *
 * \param[in] flash_adr The byte-address in flash to read from
 *
 * \return The read byte
 */

//uint8_t FLASH_0_read_flash_byte(flash_adr_t flash_adr);

/**
 * \brief Write a byte to flash
 *
 * \param[in] flash_adr The byte-address in flash to write to
 * \param[in] page_buffer A buffer in memory the size of a flash page, used as a scratchpad
 * \param[in] data The byte to write
 *
 * \return Status of the operation
 */

//nvmctrl_status_t FLASH_0_write_flash_byte(flash_adr_t flash_adr, uint8_t *ram_buffer, uint8_t data);

/**
 * \brief Erase a page in flash
 *
 * \param[in] flash_adr The byte-address in flash to erase. Must point to start-of-page.
 *
 * \return Status of the operation
 */

//nvmctrl_status_t FLASH_0_erase_flash_page(flash_adr_t flash_adr);

/**
 * \brief Write a page in flash. No page erase is performed by this function.
 *
 * \param[in] flash_adr The byte-address of the flash page to write to. Must point to start-of-page.
 * \param[in] data The data to write to the flash page
 *
 * \return Status of the operation
 */

//nvmctrl_status_t FLASH_0_write_flash_page(flash_adr_t flash_adr, uint8_t *data);

/**
 * \brief Writes a buffer to flash.
 * The flash does not need to be erased beforehand.
 * The flash address to write to does not need to be aligned to any specific boundary.
 *
 * \param[in] flash_adr The byte-address of the flash to write to
 * \param[in] data The data to write to the flash
 * \param[in] size The size of the data (in bytes) to write to the flash
 * \param[in] page_buffer A buffer in memory the size of a flash page, used as a scratchpad
 *
 * \return Status of the operation
 */

//nvmctrl_status_t FLASH_0_write_flash_block(flash_adr_t flash_adr, uint8_t *data, size_t size, uint8_t *ram_buffer);

/**
 * \brief Writes a byte stream to flash.
 * The erase granularity of the flash (i.e. one page) will cause this operation
 * to erase an entire page at a time. To avoid corrupting other flash contents,
 * make sure that the memory range in flash being streamed to is starting on a page
 * boundary, and that enough flash pages are available to hold all data being written.
 *
 * The function will perform flash page operations such as erase and write
 * as appropriate, typically when the last byte in a page is written. If
 * the last byte written is not at the last address of a page, the "finalize"
 * parameter can be set to force a page write after this byte.
 *
 * This function is intended used in devices where RAM resources are too limited
 * to afford a buffer needed by the write and erase page functions, and where
 * performance needs and code size concerns leaves the byte write and block
 * write functions too expensive.
 *
 * \param[in] flash_adr The byte-address of the flash to write to
 * \param[in] data The data byte to write to the flash
 * \param[in] finalize Set to true for the final write to the buffer
 *
 * \return Status of the operation
 */

//nvmctrl_status_t FLASH_0_write_flash_stream(flash_adr_t flash_adr, uint8_t data, bool finalize);

#ifdef __cplusplus
}
#endif

#endif /* _NVMCTRL_H_INCLUDED */
