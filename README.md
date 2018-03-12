# MappyDot Firmware

Firmware sources for use with the MappyDot - https://sensordots.org/mappydot
The firmware is developed for use and compilation with Atmel Studio.

## Compile Size
Please note, the Debug release of this firmware will not fit with the bootloader. When developing and debugging you will need to disable some functions or erase the bootloader and BOOTRST fuses. 
Some non critical functions can be easily disabled with the DEV_DISABLE compiler flag.
When running with the bootloader, the compiled size needs to be under 0x7C00 bytes to work with bootloader.

## Firmware Versions
MappyDots can be queried for their firmware version with the I2C command N (0x4E). It returns a 10 byte character array representing the firmware version. Stable major releases of the binary firmware will be built and placed the the Releases directory.
   - MD_FW_V1.0 (11/09/2017) - Release firmware. 
   - MD_FW_V1.1 (29/10/2017) - Added IDLE sleep to save ~2-5mA and some small bug fixes. 
   - MD_FW_V1.2 (14/01/2018) - Added custom measurement mode profile setting and saving (new documentation also added to https://sensordots.org/mappydotreg). Cleanup of code and API functions. Tweaks to profile setting code to support the custom measurement profiles.
   - MD_FW_V1.3 (15/01/2018) - Implemented auto address recovery and sped up init routine. Auto address recovery will initialise with the previous auto address if auto addressing fails on startup. This will only recover addresses that were obtained through auto addressing (i.e. not the master address of 0x08). This is useful if some devices reboot during operation due to power supply glitches or disconnection.
   
## Features Under Development
   - Watchdog timer.
   - MappyDot Mode.
   - Inter-device crosstalk grouping. Will allow you to assign groups to devices to prevent crosstalk.
   - Add VL53L0X Initisation Error Codes.
   - Assign new start address to master and assign master in firmware.
