/**
   MappyDot Firmware

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

   ATmega328pb:
   Fuse E: 0xfd (2.7V BOD)
   Fuse H: 0xd4 (512 words bootloader)
   Fuse L: 0xc2 (8Mhz internal RC-Osz.)
   BOOTRST - starts at boot sector on boot.
   Bootloader LED Flashes every ~25ms
   Please note, the Debug release of this firmware will not fit with
   the bootloader. When developing and debugging you will need to 
   disable some functions or disable the bootloader and BOOTRST. 
   Some non critical functions can be disabled with the DEV_DISABLE 
   compiler flag.
   Compiled size needs to be under 7C00 bytes to work with bootloader.

   NO_INT compile flag is used to test the VL53L0X without the interupt
   pin. This should not be used under normal circumstances. This is for
   debugging purposes only.

 */

#include <atmel_start.h>
#include <avr/wdt.h>
#include <string.h>
#include "vl53l0x.h"
#include "addr.h"
#include "i2c_slave.h"
#include "history_buffer.h"
#include "bwlpf.h"
#include "mappydot_reg.h"
#include "tc16.h"
#include "nvmctrl.h"
#include "crc8.h"
#include "stackmon.h"
#include "vl53l0x_types.h"
#include "main.h"
#include "sleeping.h"
#include "vl53l0x_profiles.h"
#include "helper.h" //Main helper function


#ifdef DEV_DISABLE
#warning Some functions are disabled with DEV_DISABLE
#endif

#define VERSION_STRING                "MD_FW_V1.3"

#define EEPROM_BOOTLOADER_BYTE        0x02
#define EEPROM_ADDRESS_BYTE           0x01
#define EEPROM_FACTORY_SETTINGS_START 0x20
#define EEPROM_USER_SETTINGS_START    0x60
#define EEPROM_DEVICE_NAME            0x100
#define EEPROM_CUSTOM_PROFILE_SETINGS 0x120
#define FILTER_ORDER                  2  //Filter order
#define SAMPLING_FREQ                 30 //Samples per second
#define FILTER_FREQUENCY              6  //Hz
#define SETTINGS_SIZE                 29 // first byte is 0
#define CUSTOM_PROFILE_SETTINGS_SIZE  9
#define MIN_DIST                      30 //minimum distance from VL53L0X
#define MAX_DIST                      4000 //Max sanity distance from VL53L0X
#define ACCURACY_LIMIT                1200
#define T_BOOT_MS                     2 //tBOOT is 1.2ms max as per VL53L0X datasheet

/* Private methods */
static void get_settings_buffer(uint8_t * buffer, bool stored_settings);
static void set_settings(uint8_t * buffer);
static void read_default_settings();
static void handle_rx_command(uint8_t command, uint8_t * arg, uint8_t arg_length);


/* State Variables */
#ifndef DEV_DISABLE
circular_history_buffer history_buffer;
#endif
int16_t slave_address;
filter_state low_pass_filter_state;

/* We do this so the struct is allocated as pointers are not allocated */
VL53L0X_Dev_t device;
VL53L0X_Dev_t * pDevice;
VL53L0X_RangingMeasurementData_t measure;
VL53L0X_Error status;
VL53L0X_Measurement_Mode measurement_profile;
uint8_t custom_profile_settings[CUSTOM_PROFILE_SETTINGS_SIZE]; //For custom measurement profiles

uint16_t filtered_distance;
uint16_t real_distance;
uint8_t error_code;
uint16_t distance_error;
uint16_t current_millimeters  = 0;

volatile bool filtering_enabled  = 1;
volatile bool averaging_enabled  = 0;
volatile bool factory_mode       = 0;
volatile bool crosstalk_enabled  = 0;
volatile bool vl53l0x_powerstate = 0;
volatile bool is_master          = 0;

volatile uint8_t led_mode                      = LED_PWM_ENABLED;
volatile uint8_t gpio_mode                     = GPIO_MEASUREMENT_INTERRUPT;
volatile uint8_t current_ranging_mode          = SET_CONTINUOUS_RANGING_MODE; //SET_SINGLE_RANGING_MODE; 
volatile uint8_t current_measurement_mode      = VL53L0X_DEFAULT;
volatile uint32_t led_threshold                = 300;
volatile uint32_t gpio_threshold               = 300;
volatile uint8_t averaging_size                = 4;
volatile uint8_t intersensor_crosstalk_delay   = 0;
volatile uint8_t intersensor_crosstalk_timeout = 40; //40*ticks

/* SPAD Calibration */
uint32_t refSpadCount = 0;
uint8_t ApertureSpads = 0;

/* Distance Calibration */
int32_t offsetMicroMeter = 0;

/* Crosstalk (cover) Calibration */
FixPoint1616_t xTalkCompensationRateMegaCps = 0;

/* Temperature Calibrate */
uint8_t vhvSettings = 0;
uint8_t phaseCal = 0;

int8_t led_pulse = 0;
uint8_t led_pulse_dir = 0;
uint16_t led_pulse_timeout = 0;

/* Command handler */
bool command_to_handle;
uint8_t todo_command;
uint8_t todo_arg[16];
uint8_t todo_arg_length;

static volatile bool measurement_interrupt_fired = false;
static volatile bool interrupt_timeout_interrupt_fired = false;
static volatile bool crosstalk_interrupt_fired = false;
static volatile bool calibrating = false;
uint8_t mappydot_name[16];
uint8_t settings_buffer[SETTINGS_SIZE + 1]; //Last byte is CRC

int main(void)
{	   
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();

	/* Disables analog comparator to save power */
    disable_analog();

    /* Set XSHUT to high to turn on (Shutdown is active low). */
    XSHUT_set_level(true);
	//_delay_ms(T_BOOT_MS);

    /* Set sync to input and no pull mode as it's connected to MST. */
    SYNC_set_dir(PORT_DIR_IN);
    SYNC_set_pull_mode(PORT_PULL_OFF);

	/* Init EEPROM */
    FLASH_0_init();

	/* Get master pin value */
	is_master = !MST_get_level();

    /* Read did we just come from bootloader mode? */ /* EEPROM is 0xFF on erase */
    if (FLASH_0_read_eeprom_byte(EEPROM_BOOTLOADER_BYTE) != 0x01)
    {
        /* ADDR Init */
        slave_address = addr_init(is_master);

		/* Try again once more */
		//if (slave_address == -1) slave_address = addr_init(is_master);

		/* Pull address from eeprom if we failed to get address.
		   If no or incorrect address is stored and will still trigger addressing failure. */
		if (slave_address == -1) {

		    slave_address = FLASH_0_read_eeprom_byte(EEPROM_ADDRESS_BYTE);

			//Don't allow it to auto recover master address (START_ADDRESS)
			if (slave_address <= START_ADDRESS || slave_address > END_ADDRESS) slave_address = -1;

			/* Flash LED Code every 1000ms 4 times */
			flash_led(500, 4, 0);
		}
    }

    else
    {
        /* Restore previous address from EEPROM */
        slave_address = FLASH_0_read_eeprom_byte(EEPROM_ADDRESS_BYTE);
        FLASH_0_write_eeprom_byte(EEPROM_BOOTLOADER_BYTE, 0x00);
    }

    if (slave_address <= 0)
    {
        /* Flash LED Code every 1000ms */
        flash_led(500,-1, 0);
    }

    else if (slave_address >= START_ADDRESS && slave_address <= END_ADDRESS)
    {
        /* I2C Slave Init */
        i2c_slave_init(slave_address);

		/* Write address to EEPROM for recovery of address on fail */
		store_current_address_eeprom(EEPROM_ADDRESS_BYTE, slave_address);
    }
    else
    {
        /* Bad address range */

        /* Flash LED every 200ms */
		flash_led(100,-1, 0);
    }

	/* Enable interrupts */
    sei();

	/* Once we have our address, we can re-purpose the ADDR_IN and OUT pins 
	   for the crosstalk reduction */
	/* Set ADDR_IN to input */
	ADDR_IN_set_dir(PORT_DIR_IN);

	/* Set ADDR_OUT to output and set high */
	ADDR_OUT_set_dir(PORT_DIR_OUT);
	ADDR_OUT_set_level(true);

	/* Setup Crosstalk reduction interrupt */
	/* PB6 - PCINT6 (PCMSK1) */
	PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
	PCMSK0 |= (1 << PCINT6);  // set PCINT6 to trigger an interrupt on state change
#ifndef DEV_DISABLE

    /* Initialise history buffer */
    hb_init(&history_buffer,averaging_size,sizeof(uint16_t));

    /* Initialise Filter */
    bwlpf_init(&low_pass_filter_state,FILTER_ORDER,SAMPLING_FREQ,FILTER_FREQUENCY);

#endif

    /* Ranging Interrupt setup */
    /* Interrupt is active low when fired (after ranging complete), rather than high. */
    /* Set GPIO1 (interrupt) pin to input */
    GPIO1_set_dir(PORT_DIR_IN);
    GPIO1_set_pull_mode(PORT_PULL_OFF);

    /* GPIO POUT Setup */
    SYNC_set_dir(PORT_DIR_OUT);

    /* Setup measurement interrupt */
    /* PC3 - PCINT11 (PCMSK1) */
    PCICR |= (1 << PCIE1);    // set PCIE1 to enable PCMSK1 scan
    PCMSK1 |= (1 << PCINT11);  // set PCINT11 to trigger an interrupt on state change

    /* Read User settings */
    FLASH_0_read_eeprom_block(EEPROM_USER_SETTINGS_START,settings_buffer,SETTINGS_SIZE + 1);

    /* Check Settings CRC */
	uint8_t crc = Crc8(settings_buffer,SETTINGS_SIZE);
	if (crc == 0x00 && settings_buffer[2] == 0x00) crc = 0x01;
    if (settings_buffer[SETTINGS_SIZE] != crc)
    {
        read_default_settings();
    }

    else
    {
        /* Populate user settings */
        set_settings(settings_buffer);
    }

    /* Read device name (this can be bad, we just get the user to reprogram) */
    FLASH_0_read_eeprom_block(EEPROM_DEVICE_NAME,mappydot_name, 16);

	/* Populate custom measurement profile settings */
	FLASH_0_read_eeprom_block(EEPROM_CUSTOM_PROFILE_SETINGS, settings_buffer, CUSTOM_PROFILE_SETTINGS_SIZE);

	memcpy(&custom_profile_settings[0], &settings_buffer[0], CUSTOM_PROFILE_SETTINGS_SIZE * sizeof(uint8_t));
	

	#ifdef FILL_SRAM_DEBUG
	int16_t count = StackCount();
	#endif

	/* VL53L0X Init */

	/* Assign struct to pointer */
	pDevice = &device;

	translate_measurement_mode(current_measurement_mode, &measurement_profile, custom_profile_settings);

	if (!init_ranging(pDevice, &status, translate_ranging_mode(current_ranging_mode), &measurement_profile,
					  refSpadCount,ApertureSpads,offsetMicroMeter,xTalkCompensationRateMegaCps,vhvSettings,phaseCal))
	{
		/* Ops we had an init failure */
		flash_led(5,-1, 1);
	}

	measurement_interrupt_fired = false;
	resetVl53l0xInterrupt(pDevice, &status);

	int32_t crosstalkTimeout = intersensor_crosstalk_timeout * 1000;


	/* Enable PWM timer if PWM enabled */
	if ( gpio_mode == GPIO_PWM_ENABLED) TIMER_0_init();

	if (led_mode == LED_PWM_ENABLED ) TIMER_1_init();

	static uint8_t led_duty_cycle = 0;
    static uint8_t gpio_duty_cycle = 0;

	#ifdef NO_INT
	uint32_t no_interrupt_counter = 0;
	#else
	/* Start no interrupt timer */
	if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) TIMER_2_init();
	#endif


    /* Main code */
    while (1)
    {
	    #ifdef NO_INT
	    no_interrupt_counter++; //TODO check speed of this, we should be running at about 50Hz all the time.
		#endif
	    if (!calibrating)
		{	
		    /* Put the microcontroller to sleep
			   Everything after this is affected by interrupts */
		    sleep_avr();

			if (command_to_handle) 
			{
				handle_rx_command(todo_command, todo_arg, todo_arg_length);
				command_to_handle = false;
			}

			if (crosstalk_enabled && crosstalk_interrupt_fired)
			{
				crosstalk_interrupt_fired = false;

				crosstalkTimeout = intersensor_crosstalk_timeout * 1000;

				startSingleRangingMeasurement(pDevice, &status, &measure);
			} 
		
			if (!crosstalk_interrupt_fired && is_master && crosstalk_enabled)
			{
				crosstalkTimeout--;

				/* trigger crosstalk retrigger timeout */
				if (crosstalkTimeout <= 0)
				{
					crosstalk_interrupt_fired = true;
					crosstalkTimeout = intersensor_crosstalk_timeout * 1000;
				}
			}

			#ifdef NO_INT
			#warning NO_INT set.
			if (no_interrupt_counter > 38) //runs at about 52Hz, this doesn't have to be accurate.
			{
				no_interrupt_counter = 0;
			#else 
			if (measurement_interrupt_fired)
			{
			#endif
				/* We can do a fair bit of work here once the ranging has complete, 
				* because the VL53L0X is now busy getting another range ready. */

				/* Reset the no interrupt timer */
				if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) TIMER_2_reset();

				measurement_interrupt_fired = false;

				if (crosstalk_enabled)
				{
					/* create trigger after measurement finished */
					/* Note that for this to happen, we are in "crosstalk"
					   (single) ranging mode */
					ADDR_OUT_set_level(false);
				}        

				/* If we are in measurement output modes */
				if (led_mode == LED_MEASUREMENT_OUTPUT && !factory_mode) LED_set_level(false);

				if (gpio_mode == GPIO_MEASUREMENT_INTERRUPT && !factory_mode) SYNC_set_level(true);

				/* Read current mm */
				readRange(pDevice, &status, &measure);

				error_code = measure.RangeStatus;
				
				/* If not phase error */
				if (measure.RangeStatus != 4)
				{
					current_millimeters = measure.RangeMilliMeter;
					distance_error = (uint32_t)measure.Sigma >> 16;
				} else {
					current_millimeters = 0;
					distance_error = 0;
				}			

				/* 0 is invalid measurement */

				/* Do some sanity checking (if measurement greater than 4 meters) */
				if (current_millimeters >= MAX_DIST) current_millimeters = 0;

				/* Valid measurements come out of the sensor as values > 30mm. */
				if (current_millimeters <= MIN_DIST && current_millimeters > 0) current_millimeters = MIN_DIST;

				real_distance = current_millimeters;
				filtered_distance = real_distance;

				if (filtered_distance != 0)
				{
                
	#ifndef DEV_DISABLE

					if (filtering_enabled)
						filtered_distance = bwlpf(filtered_distance, &low_pass_filter_state);

					/* Averaging happens after filtering */
					if (averaging_enabled)
					{
						//Push current measurement into ring buffer for filtering
						hb_push_back(&history_buffer, &filtered_distance);
						filtered_distance = avg(history_buffer.buffer,averaging_size);
					}
	#endif
				}
			
				/* Output modes */
				if (!factory_mode) 
				{
					if (led_mode == LED_THRESHOLD_ENABLED)
					{
						if (filtered_distance <= led_threshold && filtered_distance != 0)
						{
							/* Turn on LED */
							LED_set_level(false);
						}

						else
						{
							/* Turn off LED */
							LED_set_level(true);
						}
					}

					if (gpio_mode == GPIO_THRESHOLD_ENABLED)
					{
						if (filtered_distance <= gpio_threshold && filtered_distance != 0)
						{
							/* Turn on GPIO */
							SYNC_set_level(true);
					
						}

						else
						{
							/* Turn off GPIO */
							SYNC_set_level(false);
						}
					}

					/* If we are in measurement output modes */
					/* Note that in some modes this will output very quickly (around 60Hz)
					   when there is no valid measurement */
					if (led_mode == LED_MEASUREMENT_OUTPUT) LED_set_level(true);

					if (gpio_mode == GPIO_MEASUREMENT_INTERRUPT) SYNC_set_level(false);

					/* Crosstalk pulse off - The timing of this is "dont care" */
					if (crosstalk_enabled)
					{
						/* remove pulse */
						ADDR_OUT_set_level(true);
					}

					/* Calculate "Software" (Timer Based) PWM */
					/* This is pretty lightweight, it just uses software to calc the duty cycle.
					 * Timers then work to fire the pins. */

					if (measure.RangeStatus != 3 && measure.RangeStatus != 4) //If min range failure hasn't occurred
					{ 

						if (filtered_distance > led_threshold) led_duty_cycle = 0;
						else if (filtered_distance == 0) led_duty_cycle = 0; //Stop "bounce" when invalid measurement
						else led_duty_cycle = (led_threshold - filtered_distance)*100/(led_threshold - MIN_DIST);

						//if (led_duty_cycle > 99) led_duty_cycle = 99; // Done in set_duty

						if (filtered_distance > gpio_threshold) gpio_duty_cycle = 0;
						else if (filtered_distance == 0) gpio_duty_cycle = 0; //Stop "bounce" when invalid measurement
						else gpio_duty_cycle = (gpio_threshold - filtered_distance)*100/(gpio_threshold - MIN_DIST);

						//if (gpio_duty_cycle > 99) gpio_duty_cycle = 99; // Done in set_duty

					} else {
						led_duty_cycle = 0;
						gpio_duty_cycle = 0;
					
					}
					TIMER_1_set_duty(led_duty_cycle);
					TIMER_0_set_duty(gpio_duty_cycle);
				} 
			}

			else if (!crosstalk_enabled && interrupt_timeout_interrupt_fired)
			{
			    interrupt_timeout_interrupt_fired = 0;

				//Reset pullup on interrupt pin.
				GPIO1_set_pull_mode(PORT_PULL_UP);

				/* Reset interrupt if we have a communication timeout */
                resetVl53l0xInterrupt(pDevice, &status);
				
			}
			/* Pulse LED when in factory mode */
			if (factory_mode)
			{
				led_pulse_timeout++;
				if (led_pulse_timeout > 12) led_pulse_timeout = 0;
				if (led_pulse_timeout == 0) {
					if (led_pulse_dir) led_pulse--;
					else led_pulse++;

					if (led_pulse > 99) 
					{
						led_pulse = 99;
						led_pulse_dir = 1;
					} else if (led_pulse < 0)
					{
						led_pulse = 0;
						led_pulse_dir = 0;
					}
					TIMER_1_set_duty(led_pulse);
				}
			}	
		}
	}
}


/**
 * \brief Reads the factory default settings from EEPROM
 * 
 * 
 * \return void
 */
static void read_default_settings()
{
    /* If Bad, Check Defaults */
    FLASH_0_read_eeprom_block(EEPROM_FACTORY_SETTINGS_START,settings_buffer,SETTINGS_SIZE + 1);
	uint8_t crc = Crc8(settings_buffer,SETTINGS_SIZE);

	/* Do a sanity check on the data if 0x00 (this happens if all zeros are in EEPROM) */
	/* settings_buffer[2] is measurement mode and shouldn't be zero */
	if (crc == 0x00 && settings_buffer[2] == 0x00) crc = 0x01;

    if (settings_buffer[SETTINGS_SIZE] != crc)
    {
        /* Flash LED 4 times */
        flash_led(100,4, 0);

        /* Use program defaults (do nothing) */

    }

    else
    {
        /* Populate factory settings */
        set_settings(settings_buffer);
    }
}

/**
 * \brief Resets the VL53L0X ranging system. We do this here, because we control the XSHUT pin
 * 
 * 
 * \return void
 */
static void reset_vl53l0x_ranging()
{
	/* Set XSHUT to low to turn off (Shutdown is active low). */
	XSHUT_set_level(false);

	/* Wait for shutdown */
	delay_ms(100);

	/* Set XSHUT to high to turn on (Shutdown is active low). */
	XSHUT_set_level(true);

	/* Boot time */
	delay_ms(T_BOOT_MS);

    init_ranging(pDevice, &status, translate_ranging_mode(current_ranging_mode), &measurement_profile,
			        refSpadCount,ApertureSpads,offsetMicroMeter,xTalkCompensationRateMegaCps,vhvSettings,phaseCal);
}

/**
 * \brief I2C write callback command data from master (rx from the point of view of the slave)
 * 
 * \param command
 * \param arg
 * \param arg_length
 * 
 * \return void
 */
void main_process_rx_command(uint8_t command, uint8_t * arg, uint8_t arg_length)
{
	/* Get command ready to execute in main loop */
	command_to_handle = true;
	todo_command = command;
	memcpy(todo_arg, arg, arg_length);
	todo_arg_length = arg_length;
}

/**
 * \brief handle command in main to exit out of interrupt ASAP
 * 
 * \param command
 * \param arg
 * \param arg_length
 * 
 * \return void
 */

void handle_rx_command(uint8_t command, uint8_t * arg, uint8_t arg_length)
{
    if (arg_length == 0)
    {
	    if (command == 0x2e) //Hidden ADDR_IN test command
		{
			ADDR_IN_set_dir(PORT_DIR_OUT);
			ADDR_IN_set_level(true);
			delay_ms(200);
			ADDR_IN_set_dir(PORT_DIR_IN);
		}
        else if (command == VL53L0X_SHUTDOWN)
        {
		    /* Set XSHUT to low to turn off (Shutdown is active low). */
		    XSHUT_set_level(false);

            //vl53l0xShutdown(device, status);
            vl53l0x_powerstate = 0;
        }

        else if (command == VL53L0X_NOT_SHUTDOWN)
        {
		    /* Set XSHUT to high to turn on (Shutdown is active low). */
		    XSHUT_set_level(true);
			
			delay_ms(T_BOOT_MS);

			init_ranging(pDevice, &status, translate_ranging_mode(current_ranging_mode), &measurement_profile,
			refSpadCount,ApertureSpads,offsetMicroMeter,xTalkCompensationRateMegaCps,vhvSettings,phaseCal);
            //vl53l0xStartup(device, status);
            vl53l0x_powerstate = 1;
        }

        else if (command == RESET_VL53L0X_RANGING) //Hard reset
        {
			reset_vl53l0x_ranging();
        }

        else if (command == WRITE_CURRENT_SETTINGS_AS_START_UP_DEFAULT)
        {
            get_settings_buffer(settings_buffer, true);

			/* Settings buffer is always 1 longer than actual settings */
            settings_buffer[SETTINGS_SIZE] = Crc8(settings_buffer, SETTINGS_SIZE);
            if (!factory_mode)
            {
			    //TODO: check crc first and only write when changes are detected.
                FLASH_0_write_eeprom_block(EEPROM_USER_SETTINGS_START, settings_buffer, SETTINGS_SIZE + 1);
                FLASH_0_write_eeprom_block(EEPROM_DEVICE_NAME, mappydot_name, 16);
				FLASH_0_write_eeprom_block(EEPROM_CUSTOM_PROFILE_SETINGS, custom_profile_settings, CUSTOM_PROFILE_SETTINGS_SIZE);
            }

            else
            {
                FLASH_0_write_eeprom_block(EEPROM_FACTORY_SETTINGS_START, settings_buffer, SETTINGS_SIZE + 1);
            }
        }

        else if (command == RESTORE_FACTORY_DEFAULTS) read_default_settings();

        else if (command == INTERSENSOR_CROSSTALK_REDUCTION_ENABLE)
        {
            crosstalk_enabled = true;
            //Set to single ranging mode

            setRangingMode(pDevice, &status, translate_ranging_mode(SET_SINGLE_RANGING_MODE));
        }

        else if (command == INTERSENSOR_CROSSTALK_REDUCTION_DISABLE)
        {
            crosstalk_enabled = false;

            if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
        }

        else if (command == CALIBRATE_SPAD) 
		{
		    calibrating = true;
			//Set to single ranging mode (stop measurement)
			setRangingMode(pDevice, &status, translate_ranging_mode(SET_SINGLE_RANGING_MODE));

		    if(calibrateSPAD(pDevice, &status,&refSpadCount,&ApertureSpads) == 1) //returns 0 if success
			    flash_led(500,4,0); //error
			else
				flash_led(200,1,0);

			//Reset back to original ranging mode
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
			calibrating = false;
        }
        else if (command == TEMPERATURE_CALIBRATION) 
		{
			calibrating = true;
			//Set to single ranging mode (stop measurement)
			setRangingMode(pDevice, &status, translate_ranging_mode(SET_SINGLE_RANGING_MODE));

		    if(calibrateTemperature(pDevice, &status,&vhvSettings,&phaseCal) == 1) //returns 0 if success
			    flash_led(500,4,0); //error
			else
			    flash_led(200,1,0);

			//Reset back to original ranging mode
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
			calibrating = false;
		}

        else if (command == FILTERING_ENABLE) filtering_enabled = true;

        else if (command == FILTERING_DISABLE) filtering_enabled = false;


        else if (command == AVERAGING_ENABLE)
        {
#ifndef DEV_DISABLE
		    /* Re-initialise history buffer */
            hb_init(&history_buffer,averaging_size,sizeof(uint16_t));
#endif
            averaging_enabled = true;
        }

        else if (command == AVERAGING_DISABLE) averaging_enabled = false;

        else if (command == SET_CONTINUOUS_RANGING_MODE)
        {
            current_ranging_mode = SET_CONTINUOUS_RANGING_MODE;

            if (crosstalk_enabled != 1) { 
				setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
			}

        }

        else if (command == SET_SINGLE_RANGING_MODE)
        {
            current_ranging_mode = SET_SINGLE_RANGING_MODE;
            setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
        }

        else if (command == PERFORM_SINGLE_RANGE) {
			if (current_ranging_mode == SET_SINGLE_RANGING_MODE) startSingleRangingMeasurement(pDevice, &status, &measure);
		}
    }

    else if (arg_length == 1)
    {
        //Do a check to see if we need to reboot to bootloader
        if (command == REBOOT_TO_BOOTLOADER && arg[0] == 0x00)
        {
            //store current slave address in eeprom
            store_current_address_eeprom(EEPROM_ADDRESS_BYTE, slave_address);

            //set bootloader bit
            FLASH_0_write_eeprom_byte(EEPROM_BOOTLOADER_BYTE,0x01);
            cli(); //irq's off
            wdt_enable(WDTO_15MS); //wd on,15ms

            while(1); //loop
        }

        else if (command == INTERSENSOR_CROSSTALK_TIMEOUT) intersensor_crosstalk_timeout = arg[0];

        else if (command == INTERSENSOR_CROSSTALK_MEASUREMENT_DELAY) intersensor_crosstalk_delay = arg[0];

        else if (command == AVERAGING_SAMPLES)
        {
            if (arg[0] >= 2 && arg[0] <= 12) averaging_size = arg[0];
        }

        else if (command == SET_LED_MODE)
        {
            /* Disable LED timer */
            TIMER_1_stop();
            //Should check, but we need to save space.
            /*if (*arg[0] == LED_ON || *arg[0] == LED_OFF ||
                *arg[0] == LED_THRESHOLD_ENABLED ||
            	*arg[0] == LED_MEASUREMENT_OUTPUT ||
            	*arg[0] == LED_PWM_ENABLED)*/ led_mode = arg[0];
            LED_set_level(true);
            /* Set state straight away */
            if (led_mode == LED_ON) LED_set_level(false);

            if (led_mode == LED_OFF) LED_set_level(true);

            if (led_mode == LED_PWM_ENABLED) TIMER_1_init();
        }

        else if (command == SET_GPIO_MODE)
        {
            /* Disable PWM Timer */
            TIMER_0_stop();
            //Should check, but we need to save space.
            /*if (*arg[0] == GPIO_HIGH || *arg[0] == GPIO_LOW ||
                *arg[0] == GPIO_THRESHOLD_ENABLED ||
                *arg[0] == GPIO_MEASUREMENT_INTERRUPT ||
                *arg[0] == GPIO_PWM_ENABLED) */ gpio_mode = arg[0];
			SYNC_set_level(false);
            /* Set state straight away */
            if (gpio_mode == GPIO_HIGH) SYNC_set_level(true);

            if (gpio_mode == GPIO_LOW) SYNC_set_level(false);

            if (gpio_mode == GPIO_PWM_ENABLED) TIMER_0_init();
        }

        else if (command == RANGING_MEASUREMENT_MODE)
        {
            //Should check, but we need to save space.
            /*if (*arg[0] == HIGHLY_ACCURATE ||
            	*arg[0] == HIGH_SPEED ||
            	*arg[0] == MAPPYDOT_MODE ||
            	*arg[0] == LONG_RANGE ||
            	*arg[0] == VL53L0X_DEFAULT)
            {*/
            current_measurement_mode = arg[0];

			//Set to single ranging mode (stop measurement)
	        setRangingMode(pDevice, &status, translate_ranging_mode(SET_SINGLE_RANGING_MODE));
			//Change to selected measurement mode
			translate_measurement_mode(current_measurement_mode, &measurement_profile, custom_profile_settings);
            setRangingMeasurementMode(pDevice, &status, &measurement_profile);
			//Reset back to original ranging mode
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));

			//reset_vl53l0x_ranging();

        }
    }

    else if (arg_length == 2)
    {
        if (command == SET_GPIO_THRESHOLD_DISTANCE_IN_MM) gpio_threshold = bytes_to_mm(arg);

        else if (command == SET_LED_THRESHOLD_DISTANCE_IN_MM) led_threshold = bytes_to_mm(arg);

        else if (command == CALIBRATE_DISTANCE_OFFSET) {
		    calibrating = true;
			//Set to single ranging mode (stop measurement)
			setRangingMode(pDevice, &status, translate_ranging_mode(SET_SINGLE_RANGING_MODE));

			if(calibrateSPAD(pDevice, &status,&refSpadCount,&ApertureSpads) == 0) //returns 0 if success
			{
				if(calibrateTemperature(pDevice, &status,&vhvSettings,&phaseCal) == 0) //returns 0 if success
				{
					if (calibrateDistanceOffset(pDevice, &status,bytes_to_mm(arg),&offsetMicroMeter) == 0)  //returns 0 if success
						flash_led(200,1,0);
					else
						flash_led(500,4,0); //error
				} else {
					flash_led(500,4,0); //error
				}
			} else {
					flash_led(500,4,0); //error
			}

			//Reset back to original ranging mode
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));

			calibrating = false;
		}	    

        else if (command == CALIBRATE_CROSSTALK) {
		    calibrating = true;
			//Set to single ranging mode (stop measurement)
			setRangingMode(pDevice, &status, translate_ranging_mode(SET_SINGLE_RANGING_MODE));

		    if (calibrateCrosstalk(pDevice, &status,bytes_to_mm(arg),&xTalkCompensationRateMegaCps))  //returns 0 if success
				flash_led(500,4,0); //error
			else
				flash_led(200,1,0);

			//Reset back to original ranging mode
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
			calibrating = false;
		        
		}
    }

    else if (arg_length == 6)
    {
        //Check if we need to enter/exit factory mode (toggle the current mode)
        if (command == ENTER_FACTORY_MODE &&
                arg[0] == '!' && arg[1] == '#' &&
                arg[2] == '!' && arg[3] == '#' &&
                arg[4] == '!' && arg[5] == '#') {

					factory_mode = !factory_mode;
					if (factory_mode)
					{
						TIMER_1_init();
						led_pulse = 0;
					}
				}
    }

	else if (arg_length == CUSTOM_PROFILE_SETTINGS_SIZE)
	{
		if (command == CUSTOM_PROFILE)
		{
			memcpy(custom_profile_settings, arg, CUSTOM_PROFILE_SETTINGS_SIZE);
		}
	}

    else if (arg_length == 16)
    {
        if (command == NAME_DEVICE)
        {
            memcpy(mappydot_name, arg, 16);
        }
    }


}

/**
 * \brief 
 * 
 * \param buffer - Must have buffer of size = SETTINGS_SIZE
 * \param store_settings - are we storing settings (true) in eeprom or using it for display (false)
 * 
 * \return void
 */
static void get_settings_buffer(uint8_t * buffer, bool store_settings)
{
    uint8_t tmp_buffer[2];

    //ignore this if storing settings as it's irrelevant. We do this to save code space, rather than making a special case for it
    if (!store_settings) buffer[0] = (device.Data.CurrentParameters.MeasurementTimingBudgetMicroSeconds / 1000) & 0xff;

    buffer[1] = current_ranging_mode;
    buffer[2] = current_measurement_mode;
    buffer[3] = led_mode;
    mm_to_bytes(tmp_buffer, led_threshold);
    buffer[4] = tmp_buffer[0];
    buffer[5] = tmp_buffer[1];
    buffer[6] = gpio_mode;
    mm_to_bytes(tmp_buffer, gpio_threshold);
    buffer[7] = tmp_buffer[0];
    buffer[8] = tmp_buffer[1];
    buffer[9] = filtering_enabled;
	buffer[10] = averaging_enabled;
	buffer[11] = averaging_size;
    buffer[12] = crosstalk_enabled;
    buffer[13] = intersensor_crosstalk_delay;
	buffer[14] = intersensor_crosstalk_timeout;
    buffer[15] = vl53l0x_powerstate;
    if (store_settings) { 
	    /* SPAD Calibration */
	    buffer[16] = (refSpadCount >> 24) & 0xff; 
        buffer[17] = (refSpadCount >> 16) & 0xff;
		buffer[18] = (refSpadCount >> 8 ) & 0xff;
		buffer[19] = (refSpadCount      ) & 0xff;
		buffer[20] = ApertureSpads;

	    /* Distance Calibration */
		buffer[21] = (offsetMicroMeter >> 24) & 0xff; 
		buffer[22] = (offsetMicroMeter >> 16) & 0xff; 
		buffer[23] = (offsetMicroMeter >> 8 ) & 0xff; 
		buffer[24] = (offsetMicroMeter      ) & 0xff; 

	    /* Crosstalk (cover) Calibration */
		buffer[25] = (xTalkCompensationRateMegaCps >> 24) & 0xff;
		buffer[26] = (xTalkCompensationRateMegaCps >> 16) & 0xff;
		buffer[27] = (xTalkCompensationRateMegaCps >> 8 ) & 0xff;
		buffer[28] = (xTalkCompensationRateMegaCps      ) & 0xff;

	}
}


/**
 * \brief Set current state values with settings byte buffer
 * 
 * \param buffer - Must have buffer of size 15
 * 
 * \return void
 */
static void set_settings(uint8_t * buffer)
{
    uint8_t tmp_buffer[2];
    //buffer [0] is just zero (for settings get resuse)
    current_ranging_mode = buffer[1];
    current_measurement_mode = buffer[2];
    led_mode = buffer[3];
    tmp_buffer[0] = buffer[4];
    tmp_buffer[1] = buffer[5];
    led_threshold = bytes_to_mm(tmp_buffer);
    gpio_mode = buffer[6];
    tmp_buffer[0] = buffer[7];
    tmp_buffer[1] = buffer[8];
    gpio_threshold = bytes_to_mm(tmp_buffer);
    filtering_enabled = buffer[9];
	averaging_enabled = buffer[10];
	averaging_size = buffer[11];
    crosstalk_enabled = buffer[12];
    intersensor_crosstalk_delay = buffer[13];
	intersensor_crosstalk_timeout = buffer[14];
    vl53l0x_powerstate = buffer[15];

	
	/* SPAD Calibration */
	refSpadCount = (uint32_t)buffer[16] << 24 | (uint32_t)buffer[17] << 16 | (uint32_t)buffer[18] << 8 | buffer[19];
	ApertureSpads = buffer[20];

	/* Distance Calibration */
	offsetMicroMeter = (uint32_t)buffer[21] << 24 | (uint32_t)buffer[22] << 16 | (uint32_t)buffer[23] << 8 | buffer[24];

	/* Crosstalk (cover) Calibration */
	xTalkCompensationRateMegaCps = (uint32_t)buffer[25] << 24 | (uint32_t)buffer[26] << 16 | (uint32_t)buffer[27] << 8 | buffer[28];
	
}

/**
 * \brief I2C read callback command from master (tx from the point of view of the slave)
 * 
 * \param command
 * \param tx_buffer
 * 
 * \return uint8_t Returns the size of the bytes buffer
 */
uint8_t main_process_tx_command(uint8_t command, uint8_t * tx_buffer)
{
    if (command == READ_DISTANCE)
    {
        mm_to_bytes(tx_buffer, filtered_distance);
        return 2;
    }

    else if (command == READ_ACCURACY)
    {
        mm_to_bytes(tx_buffer, distance_error);
        return 2;
    }

    else if (command == READ_ERROR_CODE)
    {
        tx_buffer[0] = error_code;
        return 1;
    }

    else if (command == DEVICE_NAME)
    {
        memcpy(tx_buffer, mappydot_name, 16);
        return 16;
    }

    else if (command == READ_CURRENT_SETTINGS)
    {
        get_settings_buffer(tx_buffer, false);
        return 16;
    }

    else if (command == READ_NONFILTERED_VALUE)
    {
        mm_to_bytes(tx_buffer, real_distance);
        return 2;
    }

    else if (command == FIRMWARE_VERSION)
    {
        memcpy(tx_buffer, VERSION_STRING, 10);
        return 10;
    }

    return 0;
}

#ifndef NO_INT
/* Measurement Interrupt Fired */
ISR (PCINT1_vect)
{
    if(GPIO1_get_level() == 0)
    {
        measurement_interrupt_fired = true;
    }
}
#endif

/* Crosstalk interrupt fired */
ISR (PCINT0_vect)
{
	if(ADDR_IN_get_level() == 0)
	{
		crosstalk_interrupt_fired = true;
	}
}


/* No interrupt overflow (this will fire every ~500ms if no interrupt arrived) */
ISR(TIMER4_OVF_vect)
{
    interrupt_timeout_interrupt_fired = true;
}

