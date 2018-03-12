/**
   MappyDot Firmware - vl53l0x.h

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

#include "vl53l0x_api.h"
#include <stdbool.h>
#include "vl53l0x_profiles.h"

/* VL53L0X Address (8bit) */
/* Note the i2c libraries will modify the last bit (LSB) for read/write */
#define VL53L0X_I2C_ADDR  0x52

bool init_ranging(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint8_t ranging_mode, VL53L0X_Measurement_Mode * measurement_mode,
uint32_t refSpadCount, uint8_t ApertureSpads, int32_t offsetMicroMeter,
FixPoint1616_t xTalkCompensationRateMegaCps, uint8_t vhvSettings, uint8_t phaseCal);

VL53L0X_Error startSingleRangingMeasurement(VL53L0X_Dev_t * device, VL53L0X_Error * status, VL53L0X_RangingMeasurementData_t* pRangingMeasurementData );

uint16_t resetVl53l0xInterrupt(VL53L0X_Dev_t * device, VL53L0X_Error * status);
uint16_t readRange(VL53L0X_Dev_t * device, VL53L0X_Error * status, VL53L0X_RangingMeasurementData_t *RangingMeasurementData );
void stopContinuous(VL53L0X_Dev_t * device, VL53L0X_Error * status);
uint16_t setRangingMeasurementMode(VL53L0X_Dev_t * device, VL53L0X_Error * status, VL53L0X_Measurement_Mode *measurement_mode);
void setRangingMode(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint8_t mode);

uint8_t calibrateSPAD(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint32_t * refSpadCount, uint8_t * ApertureSpads);
uint8_t calibrateDistanceOffset(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint16_t referenceDistanceMM, int32_t * pOffsetMicroMeter);
uint8_t calibrateCrosstalk(VL53L0X_Dev_t * device, VL53L0X_Error * status,uint16_t referenceDistanceMM, FixPoint1616_t *pXTalkCompensationRateMegaCps);
uint8_t calibrateTemperature(VL53L0X_Dev_t * device, VL53L0X_Error * status, uint8_t *pVhvSettings, uint8_t *pPhaseCal);


