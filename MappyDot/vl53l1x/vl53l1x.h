/**
   MappyDot Firmware - VL53L1x.h

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

#include "vl53l1_api.h"
#include <stdbool.h>

/* VL53L1 Address (8bit) */
/* Note the i2c libraries will modify the last bit (LSB) for read/write */
#define VL53L1_I2C_ADDR  0x52

bool init_ranging(VL53L1_Dev_t * device, VL53L1_Error * status, uint8_t ranging_mode, uint8_t measurement_mode, uint16_t measurement_budget, VL53L1_UserRoi_t * ROI,
VL53L1_CalibrationData_t * calibration, uint8_t got_calibration_data);

void setRegionOfInterest(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_UserRoi_t * ROI);
void setLimitChecks(VL53L1_Dev_t * device, VL53L1_Error * status, uint16_t signal, uint8_t sigma);
void setCrosstalk(VL53L1_Dev_t * device, VL53L1_Error * status, uint8_t enabled);
uint16_t readSigma(VL53L1_Dev_t * device, VL53L1_Error * status);


VL53L1_Error startSingleRangingMeasurement(VL53L1_Dev_t * device, VL53L1_Error * status);

uint16_t resetVL53L1Interrupt(VL53L1_Dev_t * device, VL53L1_Error * status);
uint16_t readRange(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_RangingMeasurementData_t *RangingMeasurementData );
void stopContinuous(VL53L1_Dev_t * device, VL53L1_Error * status);
uint16_t setRangingMeasurementMode(VL53L1_Dev_t * device, VL53L1_Error * status, uint8_t measurement_mode, uint16_t measurement_budget);
void setRangingMode(VL53L1_Dev_t * device, VL53L1_Error * status, uint8_t single_mode, uint8_t measurement_mode, uint16_t measurement_budget);

uint8_t calibrateSPAD(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_CalibrationData_t * calibration);
uint8_t calibrateDistanceOffset(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_CalibrationData_t * calibration, uint16_t calibration_distance_mm);
void waitDeviceReady(VL53L1_Dev_t * device, VL53L1_Error * status);
uint8_t calibrateCrosstalk(VL53L1_Dev_t * device, VL53L1_Error * status, VL53L1_CalibrationData_t * calibration, uint16_t calibration_distance_mm);
VL53L1_Error setROI(VL53L1_Dev_t * device, VL53L1_Error * status, uint8_t topLeftX, uint8_t topLeftY, uint8_t bottomRightX, uint8_t bottomRightY );

