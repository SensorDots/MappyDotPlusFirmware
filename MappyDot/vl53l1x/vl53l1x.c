/**
   MappyDot Firmware VL53L1.c

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

#include "vl53l1x.h"
#include "vl53l1_platform.h"
#include "vl53l1_platform_init.h"
#include "vl53l1_core.h"
#include "vl53l1x_i2c_platform.h"
#include "vl53l1_api.h"
#include "tc16.h"
#include "vl53l1_wait.h"
#include "mappydot_reg.h"

void startContinuous(VL53L1_Dev_t * device, VL53L1_Error * status, uint32_t period_ms);

/**
 * \brief Initialise Ranging
 * 
 * \param device
 * \param status
 * \param ranging_mode
 * \param measurement_mode
 * \param ROI
 * \param calibration
 * \param got_calibration_data
 * 
 * \return bool
 */
bool init_ranging(VL53L1_Dev_t * device, VL53L1_Error * status, uint8_t ranging_mode, uint8_t measurement_mode, uint16_t measurement_budget, VL53L1_UserRoi_t * ROI, 
                  VL53L1_CalibrationData_t * calibration, uint8_t got_calibration_data)
{
    *status      = VL53L1_ERROR_NONE;

    // Initialize Comms
    device->i2c_slave_address     =  VL53L1_I2C_ADDR;
	
	/* Pointer error (null), this should not happen but it will if you don't allocate the pointer */
	#ifdef DEBUG
	if (device->i2c_slave_address != VL53L1_I2C_ADDR) return false;
	#endif
	
    //*status = VL53L1_platform_init(device, device->i2c_slave_address);

	*status = VL53L1_CommsInitialise(device);

	if( *status == VL53L1_ERROR_NONE ) {
		waitDeviceReady(device, status);
	}

	if( *status == VL53L1_ERROR_NONE )
    {
        *status = VL53L1_DataInit( device );
    }

	if( *status == VL53L1_ERROR_NONE )
	{
		*status = VL53L1_StaticInit( device );
	}

	// Perform this after static init and data init
	if( *status == VL53L1_ERROR_NONE && got_calibration_data)
	{
	    *status = VL53L1_SetCalibrationData(device, calibration);
	}


	if (*status == VL53L1_ERROR_NONE && !got_calibration_data)
	{
		*status = VL53L1_PerformRefSpadManagement(device);
	}

	//Get calibration should be called first and foremost. This will always 
	//be called first on a blank device since the set calibration function
	//above isn't called unless the calibration data struct is populated.

	if( *status == VL53L1_ERROR_NONE)
	{
	    *status = VL53L1_GetCalibrationData(device, calibration);
		
		if (calibration->customer.algo__crosstalk_compensation_plane_offset_kcps != 0)
			setCrosstalk(device, status, 1);

	}

	

	if( *status == VL53L1_ERROR_NONE )
	{
 		setRegionOfInterest(device,status,ROI);
	}

	if (*status == VL53L1_ERROR_NONE )
	{
		setRangingMeasurementMode(device,status,measurement_mode,measurement_budget);
	}

	setRangingMode(device, status, ranging_mode, measurement_mode, measurement_budget);

    if( *status == VL53L1_ERROR_NONE )
    {
        return true;
    }

    else
    {
        return false;
    }
}

/**
 * \brief Sets the region of interest
 * 
 * \param device
 * \param status
 * \param ROI - region of interest struct
 * 
 * \return void
 */
void setRegionOfInterest(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_UserRoi_t * ROI)
{
    *status = VL53L1_SetUserROI(device,ROI);

	/* Set a default if failed */
	if (*status !=  VL53L1_ERROR_NONE )
	{
		ROI->TopLeftX = 0;
		ROI->TopLeftY = 15;
		ROI->BotRightX = 15;
		ROI->BotRightY = 0;

		*status = VL53L1_SetUserROI(device,ROI);
	}

	/* Get "set" User ROI */
	VL53L1_GetUserROI(device,ROI);
}

/**
 * \brief Wait until device has booted
 * 
 * \param device
 * \param status
 * 
 * \return void
 */
void waitDeviceReady(VL53L1_Dev_t * device, VL53L1_Error * status)
{
	*status = VL53L1_WaitDeviceBooted(device);
}

/**
 * \brief Control the crosstalk compensation
 * 
 * \param device
 * \param status
 * \param enabled - 0 for off, 1 for on
 * 
 * \return void
 */
void setCrosstalk(VL53L1_Dev_t * device, VL53L1_Error * status, uint8_t enabled)
{
    //This has to be called before starting ranging
    *status = VL53L1_SetXTalkCompensationEnable(device,enabled); 

}


/**
 * \brief Set the ranging mode
 * 
 * \param device
 * \param status
 * \param single_mode - 1 for single, 0 for continuous
 * \param measurement_mode
 * \param measurement_budget - in ms
 * 
 * \return void
 */
void setRangingMode(VL53L1_Dev_t * device, VL53L1_Error * status, uint8_t single_mode, uint8_t measurement_mode, uint16_t measurement_budget)
{
    stopContinuous(device, status);
    if (!single_mode)
    {
	    VL53L1DevDataSet(device, LLData.measurement_mode, VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK);
		
		#ifndef NO_INT
		TIMER_2_init();
		#endif

    }
    else
    {
	    TIMER_2_stop();

		VL53L1DevDataSet(device, LLData.measurement_mode, VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT);
    }
	
	VL53L1_StartMeasurement(device);
	resetVL53L1Interrupt(device, status);
	
}

/**
 * \brief Set the limit check values
 * 
 * \param device
 * \param status
 * \param signal - set in KCps (kilo)
 * \param sigma  - set in mm
 * 
 * \return void
 */
void setLimitChecks(VL53L1_Dev_t * device, VL53L1_Error * status, uint16_t signal, uint8_t sigma)
{
    
	*status = VL53L1_SetLimitCheckValue( device,VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,((double)signal/1000*65536));
	*status = VL53L1_SetLimitCheckEnable(device,	VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	if( *status == VL53L1_ERROR_NONE )
	{
	    *status = VL53L1_SetLimitCheckValue( device,VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,(sigma*65536));
		*status = VL53L1_SetLimitCheckEnable(device,	VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	}
	
}

/**
 * \brief Sets the measurement mode parameters
 * 
 * \param device
 * \param status
 * \param measurement_mode
 * \param measurement_budget - in ms
 * 
 * \return uint16_t
 */
 uint16_t setRangingMeasurementMode(VL53L1_Dev_t * device, VL53L1_Error * status, uint8_t measurement_mode, uint16_t measurement_budget)
{
	if (measurement_mode == MED_RANGE) measurement_mode = VL53L1_DISTANCEMODE_MEDIUM;
	else if (measurement_mode == LONG_RANGE) measurement_mode = VL53L1_DISTANCEMODE_LONG;
	else measurement_mode = VL53L1_DISTANCEMODE_SHORT;

	*status = VL53L1_SetPresetMode(device, VL53L1_PRESETMODE_LITE_RANGING, measurement_mode, measurement_budget, measurement_budget);

	return 0;
}


/**
 * \brief Calibrate SPADs
 * 
 * \param device
 * \param status
 * \param calibration - calibration data
 * 
 * \return uint8_t
 */
uint8_t calibrateSPAD(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_CalibrationData_t * calibration)
{
    if (VL53L1_PerformRefSpadManagement(device) != VL53L1_ERROR_NONE) return 1;

	VL53L1_GetCalibrationData(device, calibration);
	
	return 0;
}


/**
 * \brief Calibrate Distance Offset
 * 
 * \param device
 * \param status
 * \param calibration - calibration data
 * \param calibration_distance_mm
 * 
 * \return uint8_t
 */
uint8_t calibrateDistanceOffset(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_CalibrationData_t * calibration, uint16_t calibration_distance_mm)
{
    VL53L1DevDataSet(device, LLData.measurement_mode, VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK); //Needs to be set, otherwise it returns range result 6

    if (VL53L1_PerformOffsetSimpleCalibration(device, calibration_distance_mm) != VL53L1_ERROR_NONE) return 1;
	
	VL53L1_GetCalibrationData(device, calibration);

	return 0;
}


/**
 * \brief Calibrate Crosstalk/Cover
 * 
 * \param device
 * \param status
 * \param calibration - calibration data
 * \param calibration_distance_mm
 * 
 * \return uint8_t
 */
uint8_t calibrateCrosstalk(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_CalibrationData_t * calibration, uint16_t calibration_distance_mm)
{
    VL53L1DevDataSet(device, LLData.measurement_mode, VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK);

	if (VL53L1_PerformSingleTargetXTalkCalibration(device, calibration_distance_mm) != VL53L1_ERROR_NONE) return 1;
	
	VL53L1_GetCalibrationData(device, calibration);

	// Check if no cover detected (getting similar values as no cover; no compensation detected)
	if (calibration->customer.algo__crosstalk_compensation_plane_offset_kcps == 0) return 1;

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
uint16_t resetVL53L1Interrupt(VL53L1_Dev_t * device, VL53L1_Error * status)
{
    //Reset interrupt
	return VL53L1_ClearInterruptAndStartMeasurement(device);
    return 0;
}

/**
 * \brief Perform a Single Ranging Measurement
 * 
 * \param device
 * \param status
 * \param RangingMeasurementData
 * 
 * \return VL53L1_Error
 */
VL53L1_Error startSingleRangingMeasurement(VL53L1_Dev_t * device, VL53L1_Error * status )
{
	*status = VL53L1_ERROR_NONE;

	VL53L1_StartMeasurement(device);

    return 0;
}

/**
 * \brief Stop continuous measurements
 * 
 * \param device
 * \param status
 * 
 * \return void
 */
void stopContinuous(VL53L1_Dev_t * device, VL53L1_Error * status)
{
    TIMER_2_stop();
	VL53L1_StopMeasurement(device);	
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
 */
uint16_t readRange(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_RangingMeasurementData_t *RangingMeasurementData )
{
    VL53L1_GetRangingMeasurementData(device, RangingMeasurementData);
	
    return 0;
}

