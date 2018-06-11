/*
 * mappydot_reg.c
 *
 * Created: 26/08/2017 3:45:12 AM
 *  Author: Blair Wyatt
 */

#include <stdint.h>
#include "mappydot_reg.h"

/**
 * \brief Check command size for read commands
 * 
 * \param command
 * 
 * \return uint8_t
 */
uint8_t check_command_size(uint8_t command)
{
    if (command == AVERAGING_SAMPLES)                       return 1;

	if (command == RANGING_MEASUREMENT_MODE)                return 1;

    if (command == SET_LED_MODE)                            return 1;

	if (command == SET_GPIO_MODE)                           return 1;

	if (command == INTERSENSOR_CROSSTALK_TIMEOUT)           return 1;

	if (command == INTERSENSOR_CROSSTALK_MEASUREMENT_DELAY) return 1;

    if (command == REBOOT_TO_BOOTLOADER)                    return 1;

	if (command == SIGNAL_LIMIT_CHECK_VALUE)                return 1;

	if (command == SIGMA_LIMIT_CHECK_VALUE)                 return 2;

    if (command == SET_LED_THRESHOLD_DISTANCE_IN_MM)        return 2;

	if (command == MEASUREMENT_BUDGET)						return 2;

    if (command == SET_GPIO_THRESHOLD_DISTANCE_IN_MM)       return 2;

    if (command == CALIBRATE_DISTANCE_OFFSET)               return 2;

    if (command == CALIBRATE_CROSSTALK)                     return 2;

	if (command == RANGING_MEASUREMENT_MODE)                return 2;

	if (command == REGION_OF_INTEREST)                      return 4;

    if (command == ENTER_FACTORY_MODE)                      return 6;
	
	if (command == WIPE_ALL_SETTINGS)                       return 6;

    if (command == NAME_DEVICE)                             return 16;

    return 0;
}

/**
 * \brief Check if is read command
 * 
 * \param command
 * 
 * \return uint8_t
 */
uint8_t is_read_command(uint8_t command)
{
	if (command == READ_DISTANCE || command == READ_ACCURACY || 
	    command == READ_ERROR_CODE || command == DEVICE_NAME || 
		command == READ_CURRENT_SETTINGS || command == CHECK_INTERRUPT ||
		command == READ_NONFILTERED_VALUE || command == FIRMWARE_VERSION ||
		command == AMBIENT_RATE_RETURN || command == SIGNAL_RATE_RETURN) return 1;

    return 0;
}

/*uint16_t bytes_to_mm(uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] << 8 | bytes[1]);
}*/

void mm_to_bytes(uint8_t *bytes, uint16_t mm)
{
    bytes[0] = (mm >> 8 & 0xFF);
    bytes[1] = (mm & 0xFF);
}