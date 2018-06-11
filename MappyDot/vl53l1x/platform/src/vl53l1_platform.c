/*******************************************************************************
 Copyright (C) 2016, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/**
 * @file   vl53l1_platform.c
 * @brief  Code function definitions for EwokPlus25 Platform Layer
 *         RANGING SENSOR VERSION
 *
 */

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "vl53l1_platform.h"
#include "vl53l1x_i2c_platform.h"
#include <stdint.h>
#include <util/delay.h>
#include "atmel_start_pins.h"



#ifdef PAL_EXTENDED
	#include "vl53l1_register_strings.h"
#else
	#define VL53L1_get_register_name(a,b)
#endif


#define  VL53L1_COMMS_CHUNK_SIZE  56
#define  VL53L1_COMMS_BUFFER_SIZE 64

/*#define GPIO_INTERRUPT          RS_GPIO62
#define GPIO_POWER_ENABLE       RS_GPIO60
#define GPIO_XSHUTDOWN          RS_GPIO61
#define GPIO_SPI_CHIP_SELECT    RS_GPIO51*/

/*#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)*/

	#define trace_print(level, ...)
	#define trace_i2c(...)

VL53L1_Error VL53L1_CommsInitialise(
	VL53L1_Dev_t *pdev)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_i2c_init();
	return status;
}


VL53L1_Error VL53L1_CommsClose(
	VL53L1_Dev_t *pdev)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;

	
	return status;
}

/*
 * ----------------- COMMS FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_WriteMulti(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	uint32_t     position       = 0;
	uint32_t     data_size      = 0;
    VL53L1_Error Status = VL53L1_ERROR_NONE;

		for(position=0; position<count; position+=VL53L1_COMMS_CHUNK_SIZE)
		{
			if (count > VL53L1_COMMS_CHUNK_SIZE)
			{
				if((position + VL53L1_COMMS_CHUNK_SIZE) > count)
				{
					data_size = count - position;
				}
				else
				{
					data_size = VL53L1_COMMS_CHUNK_SIZE;
				}
			}
			else
			{
				data_size = count;
			}

			if (Status == VL53L1_ERROR_NONE)
			{
				if( VL53L1_write_multi(
				pdev->i2c_slave_address,
				index+position,
				pdata+position,
				data_size) != 0 )
				{
					Status = VL53L1_ERROR_CONTROL_INTERFACE;
				}
			}
		}


    return Status;
}


VL53L1_Error VL53L1_ReadMulti(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	uint32_t     position       = 0;
	uint32_t     data_size      = 0;
	for(position=0; position<count; position+=VL53L1_COMMS_CHUNK_SIZE)
	{
		if(count > VL53L1_COMMS_CHUNK_SIZE)
		{
			if((position + VL53L1_COMMS_CHUNK_SIZE) > count)
			{
				data_size = count - position;
			}
			else
			{
				data_size = VL53L1_COMMS_CHUNK_SIZE;
			}
		}
		else
		data_size = count;

		if(Status == VL53L1_ERROR_NONE)
		{
			if( VL53L1_read_multi(
			pdev->i2c_slave_address,
			index+position,
			pdata+position,
			data_size) != 0 )
			{
				Status = VL53L1_ERROR_CONTROL_INTERFACE;
			}
		}
    }

	return Status;
}


VL53L1_Error VL53L1_WrByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t       data)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    status_int = VL53L1_write_byte(pdev->i2c_slave_address, index, data);

    if (status_int != 0)
    Status = VL53L1_ERROR_CONTROL_INTERFACE;

    return Status;
}


VL53L1_Error VL53L1_WrWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t      data)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    status_int = VL53L1_write_word(pdev->i2c_slave_address, index, data);

    if (status_int != 0)
    Status = VL53L1_ERROR_CONTROL_INTERFACE;

    return Status;
}


VL53L1_Error VL53L1_WrDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t      data)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    status_int = VL53L1_write_dword(pdev->i2c_slave_address, index, data);

    if (status_int != 0)
    Status = VL53L1_ERROR_CONTROL_INTERFACE;

    return Status;
}


VL53L1_Error VL53L1_RdByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    status_int = VL53L1_read_byte(pdev->i2c_slave_address, index, pdata);

    if (status_int != 0)
    Status = VL53L1_ERROR_CONTROL_INTERFACE;

    return Status;
}


VL53L1_Error VL53L1_RdWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    status_int = VL53L1_read_word(pdev->i2c_slave_address, index, pdata);

    if (status_int != 0)
    Status = VL53L1_ERROR_CONTROL_INTERFACE;

    return Status;
}


VL53L1_Error VL53L1_RdDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    status_int = VL53L1_read_dword(pdev->i2c_slave_address, index, pdata);

    if (status_int != 0)
    Status = VL53L1_ERROR_CONTROL_INTERFACE;

    return Status;
}

/*
 * ----------------- HOST TIMING FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_WaitUs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_us)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	while (wait_us > 0)
	{
		_delay_us(1);
		wait_us--;
	}
	

	return status;
}


VL53L1_Error VL53L1_WaitMs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_ms)
{
		VL53L1_Error status         = VL53L1_ERROR_NONE;
		while (wait_ms > 0)
		{
			_delay_ms(1);
			wait_ms--;
		}

		return status;
}

/*
 * ----------------- HARDWARE STATE FUNCTIONS -----------------
 */

VL53L1_Error  VL53L1_GpioXshutdown(uint8_t value)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

	//XSHUT_set_level(value);

	return status;
}


VL53L1_Error  VL53L1_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

	//SUPPRESS_UNUSED_WARNING(function);
	//SUPPRESS_UNUSED_WARNING(edge_type);

	return status;
}


VL53L1_Error  VL53L1_GpioInterruptDisable(void)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

	return status;
}


VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{

	/* Returns current tick count in [ms] */

	VL53L1_Error status  = VL53L1_ERROR_NONE;

	/**ptick_count_ms = timeGetTime();

	trace_print(
	VL53L1_TRACE_LEVEL_DEBUG,
	"VL53L1_GetTickCount() = %5u ms;\n",
	*ptick_count_ms);*/

	return status;

}


VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	/*
	 * Platform implementation of WaitValueMaskEx V2WReg script command
	 *
	 * WaitValueMaskEx(
	 *          duration_ms,
	 *          index,
	 *          value,
	 *          mask,
	 *          poll_delay_ms);
	 */

	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53L1_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	//_LOG_STRING_BUFFER(register_name);

	//SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53L1_LOG_ENABLE
	/* look up register name */
	VL53L1_get_register_name(
			index,
			register_name);

	/* Output to I2C logger for FMT/DFT  */
	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif // VL53L1_LOG_ENABLE

	/* calculate time limit in absolute time */

	//VL53L1_GetTickCount(&start_time_ms);
	//pdev->new_data_ready_poll_duration_ms = 0;

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53L1_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
	//_LOG_SET_TRACE_FUNCTIONS(VL53L1_TRACE_FUNCTION_NONE);

	/* wait until value is found, timeout reached on error occurred */
	while ((status == VL53L1_ERROR_NONE) &&
		 		   (found == 0) && (timeout_ms > 0))
	{
	    timeout_ms--;
		status = VL53L1_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}

		if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(
							pdev,
							poll_delay_ms);
		

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		//VL53L1_GetTickCount(&current_time_ms);
		//pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}

	/* Restore function logging */
	//_LOG_SET_TRACE_FUNCTIONS(trace_functions);

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}


VL53L1_Error VL53L1_WaitUntilInterrupt(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint32_t      poll_delay_ms)
{

	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;

	/* wait until interrupt line is triggered or timeout reached */
	while ((status == VL53L1_ERROR_NONE) &&
		 		   (found == 0) && (timeout_ms > 0))
	{
	    timeout_ms--;


		if(GPIO1_get_level() == 0)
		{
			found = 1;
		}

		if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(
							pdev,
							poll_delay_ms);
		
	}

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}

