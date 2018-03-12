/**
   MappyDot Firmware vl53l0x.c

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

#include "vl53l0x.h"
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_api.h"
#include "tc16.h"
#include "vl53l0x_profiles.h"

void startContinuous(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint32_t period_ms);
uint16_t setSingleRangingMode(VL53L0X_Dev_t * device, VL53L0X_Error * status);

/**
 * \brief Initialise Ranging
 * 
 * \param device
 * \param status
 * \param ranging_mode
 * \param measurement_mode
 * \param refSpadCount
 * \param ApertureSpads
 * \param offsetMicroMeter
 * \param xTalkCompensationRateMegaCps
 * \param vhvSettings
 * \param phaseCal
 * 
 * \return bool
 */
bool init_ranging(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint8_t ranging_mode, VL53L0X_Measurement_Mode * measurement_mode,
                  uint32_t refSpadCount, uint8_t ApertureSpads, int32_t offsetMicroMeter, 
				  FixPoint1616_t xTalkCompensationRateMegaCps, uint8_t vhvSettings, uint8_t phaseCal)
{
    //int32_t   status_int;
    //int32_t   init_done         = 0;
    *status      = VL53L0X_ERROR_NONE;
	bool spad_has_calibration = false;

    // Initialize Comms
    device->I2cDevAddr     =  VL53L0X_I2C_ADDR;
	
	/* Pointer error (null), this should not happen but it will if you don't allocate the pointer */
	if (device->I2cDevAddr != VL53L0X_I2C_ADDR) return false;

    VL53L0X_i2c_init();

    // Data initialization
    /*VL53L0X_DataInit() function is called one time, and it
    performs the device initialization. To be called once and only once
    after device is brought out of reset.*/

    if( *status == VL53L0X_ERROR_NONE )
    {
        *status = VL53L0X_DataInit( device );
    }

	/* If we have spad calibration data, load here and perform static init
	 * without setting SPADs (alt_spad_init = true)*/

	/* Note: Ref Spad should happen before Refcalibration. */


	if (refSpadCount != 0 && ApertureSpads != 0)
	{
	    VL53L0X_SetReferenceSpads(device,refSpadCount,ApertureSpads);
		*status = VL53L0X_StaticInit( device, true );
		spad_has_calibration = true;
	}
	if( (*status == VL53L0X_ERROR_NONE && !spad_has_calibration) || *status == VL53L0X_ERROR_REF_SPAD_INIT )
	{
		/* Fallback SPAD Init */
		/* Only happens when no calibration has been done. Can happen
		if SPADs fail (very very rare; heat or shock damage can cause this) an a recalibration is required.
		If the calibration has not been performed (using
		VL53L0X_PerformRefSpadManagement()), or if the Host has not programmed the
		number and type of SPADs (using VL53L0X_SetReferenceSpads()),
		VL53L0X_GetReferenceSpads() will return the number and type of reference SPADs
		programmed into the device NVM. */
		*status = VL53L0X_StaticInit( device, false );

		*status = VL53L0X_PerformRefSpadManagement(device, &refSpadCount, &ApertureSpads);
		    
	}

	/* Temperature Cal on Startup*/
	*status = VL53L0X_PerformRefCalibration(device, &vhvSettings, &phaseCal);

	/* Set Calibration Values */

	if (spad_has_calibration && *status == VL53L0X_ERROR_NONE && offsetMicroMeter != 0)
	{
		/* Set Offset calibration */
		*status = VL53L0X_SetOffsetCalibrationDataMicroMeter(device,offsetMicroMeter);
	}

	if (spad_has_calibration && *status == VL53L0X_ERROR_NONE && offsetMicroMeter != 0)
	{
		/* Set Cross-talk correction */
		*status = VL53L0X_SetXTalkCompensationRateMegaCps(device,xTalkCompensationRateMegaCps);
		VL53L0X_SetXTalkCompensationEnable(device,1);
	}

	/* Used to check on each, but we can assume everything after init should be fine */
	if( *status == VL53L0X_ERROR_NONE )
	{

	    /* Sigma is the time difference (shift) between the reference and return SPAD arrays. */
        *status = VL53L0X_SetLimitCheckEnable( device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );

	    /* Return Signal Rate */
        *status = VL53L0X_SetLimitCheckEnable( device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );		
		
		setRangingMeasurementMode(device, status, measurement_mode);
	    setRangingMode(device, status, ranging_mode);
	}

    if( *status == VL53L0X_ERROR_NONE )
    {
        return true;
    }

    else
    {
        return false;
    }
}

/**
 * \brief Set Single Range Mode
 * 
 * \param device
 * \param status
 * 
 * \return uint16_t
 */
uint16_t setSingleRangingMode(VL53L0X_Dev_t * device, VL53L0X_Error * status)
{
    return VL53L0X_SetDeviceMode( device, VL53L0X_DEVICEMODE_SINGLE_RANGING );        // Setup in single ranging mode
}


/**
 * \brief Set the ranging mode
 * 
 * \param device
 * \param status
 * \param mode - 0 for single, 1 for continuous
 * 
 * \return void
 */
void setRangingMode(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint8_t mode)
{
    if (mode == 1)
    {
	    startContinuous(device, status, 0);
		TIMER_2_init();
    }

    else
    {
	    TIMER_2_stop();
	    stopContinuous(device, status);
		setSingleRangingMode(device, status);
    }
}

/**
 * \brief Sets the measurement mode parameters
 * 
 * \param device
 * \param status
 * \param rangingMode
 * 
 * \return uint16_t
 */

uint16_t setRangingMeasurementMode(VL53L0X_Dev_t * device, VL53L0X_Error * status, VL53L0X_Measurement_Mode * measurement_mode)
{
	//Defaults not in API - VL53L0X_VCSEL_PERIOD_PRE_RANGE 14 VL53L0X_VCSEL_PERIOD_FINAL_RANGE 10
	/* uint8_t test;
	VL53L0X_GetVcselPulsePeriod(device, VL53L0X_VCSEL_PERIOD_PRE_RANGE, &test);
	VL53L0X_GetVcselPulsePeriod(device, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, &test);*/
    
    /* Signal rate minimum threshold. Measurements with signal rates below this value are ignored. Disabled by default */
    VL53L0X_SetLimitCheckEnable(device, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, measurement_mode->range_ignore_threshold);
	if (measurement_mode->range_ignore_threshold != 0) VL53L0X_SetLimitCheckValue(device, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)measurement_mode->range_ignore_threshold_value);
	VL53L0X_SetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)measurement_mode->signal_rate_final_range);
	VL53L0X_SetLimitCheckValue(device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)measurement_mode->sigma_final_range);
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(device, measurement_mode->timing_budget_ms); //30Hz
	VL53L0X_SetVcselPulsePeriod(device, VL53L0X_VCSEL_PERIOD_PRE_RANGE, measurement_mode->vcsel_period_pre_range);
	VL53L0X_SetVcselPulsePeriod(device, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, measurement_mode->vscel_period_final_range);

	return 0;
}


/**
 * \brief Calibrate SPADs
 * 
 * \param device
 * \param status
 * \param refSpadCount
 * \param ApertureSpads
 * 
 * \return uint8_t
 */
uint8_t calibrateSPAD(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint32_t * refSpadCount, uint8_t * ApertureSpads)
{
    if (VL53L0X_PerformRefSpadManagement(device, refSpadCount, ApertureSpads) != VL53L0X_ERROR_NONE) return 1;
	
	return 0;
}


/**
 * \brief Calibrate Distance Offset (with white target at 100mm)
 * 
 * \param device
 * \param status
 * \param referenceDistanceMM
 * \param pOffsetMicroMeter
 * 
 * \return uint8_t
 */
uint8_t calibrateDistanceOffset(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint16_t referenceDistanceMM, int32_t * pOffsetMicroMeter)
{
    if (VL53L0X_PerformOffsetCalibration(device,referenceDistanceMM, pOffsetMicroMeter) != VL53L0X_ERROR_NONE) return 1;
	return 0;
}


/**
 * \brief Calibrate Crosstalk/Cover (with gray target at 400mm)
 * 
 * \param device
 * \param status
 * \param referenceDistanceMM
 * \param pXTalkCompensationRateMegaCps
 * 
 * \return uint8_t
 */
uint8_t calibrateCrosstalk(VL53L0X_Dev_t * device, VL53L0X_Error * status,uint16_t referenceDistanceMM, FixPoint1616_t *pXTalkCompensationRateMegaCps)
{
	if (VL53L0X_PerformXTalkCalibration(device,referenceDistanceMM, pXTalkCompensationRateMegaCps) != VL53L0X_ERROR_NONE) return 1;
	return 0;
}

/**
 * \brief Calibrate Temperature Offset
 * 
 * \param device
 * \param status
 * \param pVhvSettings
 * \param pPhaseCal
 * 
 * \return uint8_t
 */
uint8_t calibrateTemperature(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint8_t *pVhvSettings, uint8_t *pPhaseCal)
{
    if (VL53L0X_PerformRefCalibration(device, pVhvSettings, pPhaseCal) != VL53L0X_ERROR_NONE) return 1;	
	return 0;
}


/**
 * \brief Reset Measurement Interrupt
 * 
 * \param device
 * \param status
 * 
 * \return uint16_t
 */
uint16_t resetVl53l0xInterrupt(VL53L0X_Dev_t * device, VL53L0X_Error * status)
{
    //Reset interrupt
    //VL53L0X_write_byte(device->I2cDevAddr,VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    return VL53L0X_ClearInterruptMask(device, 1);
    return 0;
}

/**
 * \brief Perform a Single Ranging Measurement
 * 
 * \param device
 * \param status
 * \param RangingMeasurementData
 * 
 * \return VL53L0X_Error
 */
VL53L0X_Error startSingleRangingMeasurement(VL53L0X_Dev_t * device, VL53L0X_Error * status, VL53L0X_RangingMeasurementData_t *RangingMeasurementData )
{
    //VL53L0X_Error   Status = VL53L0X_ERROR_NONE;
	*status = VL53L0X_ERROR_NONE;
    //FixPoint1616_t  LimitCheckCurrent;
    *status = VL53L0X_PerformSingleRangingMeasurement(device, RangingMeasurementData, 0);
    //VL53L0X_GetLimitCheckCurrent( pDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent );
    resetVl53l0xInterrupt(device,status);
    return 0;
}

/**
 * \brief Start continuous ranging measurement
 * 
 * \param device
 * \param status
 * \param period_ms
 * 
 * \return void
 *
 *  If period_ms (optional) is 0 or not
 *  given, continuous back-to-back mode is used (the sensor takes measurements as
 *  often as possible); otherwise, continuous timed mode is used, with the given
 *  inter-measurement period in milliseconds determining how often the sensor
 *  takes a measurement.
 */
void startContinuous(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint32_t period_ms)
{
    if (period_ms != 0)
    {
        /* continuous timed mode */
        VL53L0X_SetInterMeasurementPeriodMilliSeconds(device, period_ms);
        *status = VL53L0X_SetDeviceMode( device, VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING );
    }

    else
    {
        /* continuous back-to-back mode */
        *status = VL53L0X_SetDeviceMode( device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING );
    }

    VL53L0X_StartMeasurement(device);
    resetVl53l0xInterrupt(device, status);
}

/**
 * \brief Stop continuous measurements
 * 
 * \param device
 * \param status
 * 
 * \return void
 */
void stopContinuous(VL53L0X_Dev_t * device, VL53L0X_Error * status)
{
    VL53L0X_StopMeasurement(device);
}


/**
 * \brief Read Range once Interrupt has fired.
 * 
 * \param device
 * \param status
 * \param RangingMeasurementData
 * 
 * \return uint16_t
 *  Returns a range reading in millimeters once a measurement interrupt has fired.
 *  This function DRASTICALLY!!!! reduces the amount of i2c cycles required. And also allows
 *  you to read distances greater than 254mm.
 */
uint16_t readRange(VL53L0X_Dev_t * device, VL53L0X_Error * status, VL53L0X_RangingMeasurementData_t *RangingMeasurementData )
{
    VL53L0X_GetRangingMeasurementData(device, RangingMeasurementData);

    resetVl53l0xInterrupt(device, status);
    return 0;
}
