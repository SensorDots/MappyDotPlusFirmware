# MappyDot Plus Firmware

Firmware sources for use with the MappyDot Plus - https://sensordots.org/mappydot_plus
The firmware is developed for use and compilation with Atmel Studio.

## Compile Size
Please note, the Debug release of this firmware will not fit with the bootloader. When developing and debugging you will need to disable some functions or erase the bootloader and BOOTRST fuses. 
Some non critical functions can be easily disabled with the DEV_DISABLE compiler flag.
When running with the bootloader, the compiled size needs to be under 0x7C00 bytes to work with bootloader.

## Firmware Versions
MappyDots can be queried for their firmware version with the I2C command N (0x4E). It returns a 10 byte character array representing the firmware version. Stable major releases of the binary firmware will be built and placed the the Releases directory.
   - MDPFW_V1.0 (15/03/2018) - Release firmware. 
    
## Features Under Development
   - Watchdog timer.
   - Inter-device crosstalk grouping. Will allow you to assign groups to devices to prevent crosstalk.
   - Add VL53L1X Initisation Error Codes.
   - Assign new start address to master and assign master in firmware.
