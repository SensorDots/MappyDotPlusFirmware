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
#include "vl53l1x.h"
#include "addr.h"
#include "i2c_slave.h"
#include "history_buffer.h"
#include "bwlpf.h"
#include "mappydot_reg.h"
#include "tc16.h"
#include "nvmctrl.h"
#include "crc8.h"
#include "stackmon.h"
#include "vl53l1_types.h"
#include "main.h"
#include "sleeping.h"
#include "helper.h" //Main helper function


#ifdef DEV_DISABLE
#warning Some functions are disabled with DEV_DISABLE
#endif

#define VERSION_STRING                "MDPFW_V1.2"

#define EEPROM_ADDRESS_BYTE           0x01
#define EEPROM_BOOTLOADER_BYTE        0x02
#define EEPROM_DEVICE_NAME            0x20
#define EEPROM_FACTORY_SETTINGS_START 0x40
#define EEPROM_USER_SETTINGS_START    0x60
#define EEPROM_FACTORY_CALIB_START    0x80 //Size 95 bytes
#define EEPROM_USER_CALIB_START       0xF0

#define FILTER_FREQUENCY              6  //Hz
#define SETTINGS_SIZE                 21
#define CALIB_SIZE                    95
#define MIN_DIST                      30 //minimum distance from VL53L1X
#define MAX_DIST                      5000 //Max sanity distance from VL53L1X
#define ACCURACY_LIMIT                1200
//#define T_BOOT_MS                     2

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
VL53L1_Dev_t device;
VL53L1_Dev_t * pDevice;
VL53L1_RangingMeasurementData_t measure;
VL53L1_Error status;
VL53L1_UserRoi_t ROI;

uint16_t filtered_distance         = 0;
uint16_t real_distance             = 0;
uint8_t error_code                 = 0;
uint16_t distance_error            = 0;
uint16_t current_millimeters       = 0;
uint16_t signal_rate_rtn_mega_cps  = 0;
uint16_t ambient_rate_rtn_mega_cps = 0;

bool filtering_enabled  = 1;
bool averaging_enabled  = 0;
bool factory_mode       = 0;
bool crosstalk_enabled  = 0;
bool vl53l1x_powerstate = 0;
bool is_master          = 0;

uint8_t led_mode                      = LED_PWM_ENABLED;
uint8_t gpio_mode                     = GPIO_MEASUREMENT_INTERRUPT;
uint8_t current_ranging_mode          = SET_CONTINUOUS_RANGING_MODE; //SET_SINGLE_RANGING_MODE; 
uint8_t current_measurement_mode      = MED_RANGE;
uint32_t led_threshold                = 300;
uint32_t gpio_threshold               = 300;
uint8_t averaging_size                = 4;
uint8_t current_average_size          = 0;
uint8_t intersensor_crosstalk_delay   = 0;
uint8_t intersensor_crosstalk_timeout = 40; //40*ticks
uint16_t measurement_budget           = 41; //ms
uint8_t read_interrupt                = 0;
uint16_t signal_limit_check           = 1000;
uint8_t sigma_limit_check             = 15;
uint8_t intersensor_sync              = 0;

VL53L1_CalibrationData_t calibration_data;
bool got_calibration_data = false;

int8_t led_pulse           = 0;
uint8_t led_pulse_dir      = 0;
uint16_t led_pulse_timeout = 0;
uint8_t ignore_next_filter = 0;

#define SETTLE_MEASUREMENTS 2
int8_t run_until_settle    = SETTLE_MEASUREMENTS;


/* Command handler */
bool command_to_handle;
uint8_t todo_command;
uint8_t todo_arg[16];
uint8_t todo_arg_length;

static volatile bool measurement_interrupt_fired = false;
static volatile bool interrupt_timeout_interrupt_fired = false;
static volatile bool crosstalk_sync_interrupt_fired = false;
static volatile bool calibrating = false;
uint8_t mappydot_name[16];
uint8_t settings_buffer[SETTINGS_SIZE + 2]; //(+2 is for code reuse)

int main(void)
{	   
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();

	/* Disables analog comparator to save power */
    disable_analog();

    /* Set XSHUT to high to turn on (Shutdown is active low). */
	//This is now done in start_init
    //XSHUT_set_level(true);
	//_delay_ms(T_BOOT_MS); //no longer required

    /* Set sync to input and no pull mode as if it's connected to MST. */
    //This is now done in start_init
    //SYNC_set_dir(PORT_DIR_IN);
    //SYNC_set_pull_mode(PORT_PULL_OFF);

	/* Init EEPROM */
    FLASH_0_init();

	/* Get master pin value */
	is_master = !MST_get_level();

    /* Read did we just come from bootloader mode? */ /* EEPROM is 0xFF on erase */
    if (FLASH_0_read_eeprom_byte(EEPROM_BOOTLOADER_BYTE) != 0x01)
    {
        /* ADDR Init */
		#ifndef DEBUG
        slave_address = addr_init(is_master);
		#else
		slave_address = 0x08;
		#warning Slave address set to 0x08!!!!!
		#endif

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

	//TODO "De-boilerplate" this:

    /* Read User settings */
    FLASH_0_read_eeprom_block(EEPROM_USER_SETTINGS_START, settings_buffer, SETTINGS_SIZE);

	/* Read calibration data */
	uint8_t * calib_ptr = (uint8_t *)&calibration_data;

	FLASH_0_read_eeprom_block(EEPROM_USER_CALIB_START, calib_ptr, CALIB_SIZE);
	uint8_t crc_calib = Crc8(calib_ptr, CALIB_SIZE);

    /* Check Settings CRC */
	uint8_t crc = Crc8(settings_buffer, SETTINGS_SIZE);

	/* Sanity checks CRC when EEPROM is all zeros */
	if (crc == 0x00 && settings_buffer[2] == 0x00) crc = 0x01;
	if (crc_calib == 0x00 && calibration_data.customer.global_config__spad_enables_ref_0 == 0x00
		&& calibration_data.customer.global_config__spad_enables_ref_1 == 0x00
		&& calibration_data.customer.global_config__spad_enables_ref_2 == 0x00
		&& calibration_data.customer.global_config__spad_enables_ref_3 == 0x00) crc_calib = 0x01;

    if (FLASH_0_read_eeprom_byte(EEPROM_USER_SETTINGS_START + SETTINGS_SIZE) != crc || FLASH_0_read_eeprom_byte(EEPROM_USER_CALIB_START + CALIB_SIZE + 1) != crc_calib)
    {
	    
        read_default_settings();
    }

    else
    {
	    got_calibration_data = true;
        /* Populate user settings */
        set_settings(settings_buffer);
    }

    /* Read device name (this can be bad, we just get the user to reprogram) */
    FLASH_0_read_eeprom_block(EEPROM_DEVICE_NAME,mappydot_name, 16);
	

	#ifdef FILL_SRAM_DEBUG
	//int16_t count = StackCount();
	#endif

	/* VL53L1X Init */

	/* Assign struct to pointer */
	pDevice = &device;

	/* Run init in continous mode (0) until measurements settle */
	if (!init_ranging(pDevice, &status, 0, current_measurement_mode, measurement_budget, &ROI,
					  &calibration_data, got_calibration_data))
	{
		/* Ops we had an init failure */
		flash_led(5,-1, 1);
	}

	#ifndef DEV_DISABLE

	/* Initialise history buffer */
	hb_init(&history_buffer,averaging_size,sizeof(uint16_t));

	/* Initialise Filter */
	bwlpf_init(&low_pass_filter_state,1000/measurement_budget,FILTER_FREQUENCY);

	#endif

	measurement_interrupt_fired = false;

	int32_t crosstalkTimeout = intersensor_crosstalk_timeout * 1000;

	/* Enable PWM timer if PWM enabled */
	if ( gpio_mode == GPIO_PWM_ENABLED) TIMER_0_init();

	if (led_mode == LED_PWM_ENABLED ) TIMER_1_init();

	static uint8_t led_duty_cycle = 0;
    static uint8_t gpio_duty_cycle = 0;

	//Reset pullup on interrupt pin.
	GPIO1_set_pull_mode(PORT_PULL_UP);

	run_until_settle    = SETTLE_MEASUREMENTS;

	resetVL53L1Interrupt(pDevice, &status);

	#ifdef NO_INT
	uint32_t no_interrupt_counter = 0;
	#else
	/* Start no interrupt timer */
	//if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) TIMER_2_init(); //Now set in vl53l1x.c
	#endif


    /* Main code */
    while (1)
    {
	    #ifdef NO_INT
	    no_interrupt_counter++;
		#endif
	    if (!calibrating)
		{	
		    /* Put the microcontroller to sleep
			   Everything after this is affected by interrupts */
		    #ifndef DEBUG
				sleep_avr();
			#endif

			if (command_to_handle) 
			{
				handle_rx_command(todo_command, todo_arg, todo_arg_length);
				command_to_handle = false;
			}

			if (crosstalk_sync_interrupt_fired && 
			    ((crosstalk_enabled) || (intersensor_sync && !is_master)))
			{
				crosstalk_sync_interrupt_fired = false;

				crosstalkTimeout = intersensor_crosstalk_timeout * 1000;

				resetVL53L1Interrupt(pDevice, &status);
			} 
		
			if (!crosstalk_sync_interrupt_fired && is_master && crosstalk_enabled)
			{
				crosstalkTimeout--;

				/* trigger crosstalk re-trigger timeout */
				if (crosstalkTimeout <= 0)
				{
					crosstalk_sync_interrupt_fired = true;
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
				* because the VL53L1X is now busy getting another range ready. */

				/* Reset the no interrupt timer */
				if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) TIMER_2_reset();

				measurement_interrupt_fired = false;

				if (crosstalk_enabled || (intersensor_sync && is_master && current_ranging_mode == SET_CONTINUOUS_RANGING_MODE))
				{
					/* Create trigger after measurement finished.
					   For sync, we assume a new measurement is already underway on master */
					ADDR_OUT_set_level(false);
				}     

				/* If we are in measurement output modes */
				if (led_mode == LED_MEASUREMENT_OUTPUT && !factory_mode) LED_set_level(false);

				if (gpio_mode == GPIO_MEASUREMENT_INTERRUPT && !factory_mode) SYNC_set_level(true);

				/* Read current mm */
				readRange(pDevice, &status, &measure);   

				if (intersensor_sync || crosstalk_enabled)
				{
					if (is_master && intersensor_sync)
					{
						if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) resetVL53L1Interrupt(pDevice, &status);
						else if (current_ranging_mode == SET_SINGLE_RANGING_MODE) stopContinuous(pDevice, &status);
					}
					// else do nothing

				} else {
					if (run_until_settle < 0)
					{
						if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) resetVL53L1Interrupt(pDevice, &status);
						else if (current_ranging_mode == SET_SINGLE_RANGING_MODE) stopContinuous(pDevice, &status);
						} else {
						resetVL53L1Interrupt(pDevice, &status);
					}
				}


				error_code = measure.RangeStatus;
				
				/* If phase error */
				if (measure.RangeStatus == 4)
				{
				    current_millimeters = 0;
				} else {
					current_millimeters = measure.RangeMilliMeter;
				}			

				distance_error = measure.SigmaMilliMeter >> 16; 
				signal_rate_rtn_mega_cps = measure.SignalRateRtnMegaCps >> 16;
				ambient_rate_rtn_mega_cps = measure.AmbientRateRtnMegaCps >> 16;

				/* 0 is invalid measurement */

				/* Do some sanity checking */
				if (current_millimeters >= MAX_DIST) current_millimeters = 0;

				/* Valid measurements come out of the sensor as values > 30mm. */
				if (current_millimeters <= MIN_DIST && current_millimeters > 0) current_millimeters = MIN_DIST;

				real_distance = current_millimeters;
				filtered_distance = real_distance;

#ifndef DEV_DISABLE
				/* Push current measurement into ring buffer for averaging */
				hb_push_back(&history_buffer, &filtered_distance);

				if (filtered_distance != 0)
				{
					if (filtering_enabled)
					{
						filtered_distance = bwlpf(filtered_distance, &low_pass_filter_state);

						/* Ignore filtered distance if over max, this can happen when there is a rapid change 
						   from no measurement to large value. This is because the filter does take time to settle
						   but during this time the error code will generally be 7 */
						if (ignore_next_filter > 0)
						{
							filtered_distance = real_distance;
							ignore_next_filter--;
						}

						if ((filtered_distance > MAX_DIST || distance_error >= 7) && !ignore_next_filter)
						{
							filtered_distance = real_distance;
							ignore_next_filter = FILTER_ORDER * 2;
						}
					}

					/* Averaging happens after filtering */
					if (averaging_enabled)
					{
						/* Wait for history buffer to fill */
						if (current_average_size < averaging_size)
						{
							filtered_distance = real_distance;
							current_average_size++;
						}
						else
						{
							filtered_distance = avg(history_buffer.buffer,averaging_size);
						}
					}

				}
	#endif
				read_interrupt = 1;

				/* We run continuous measurements until the device settles after init. When in single shot mode
				   on startup, the interrupts don't fire after first few tries due to VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL */
				if (run_until_settle == 0)
				{
					setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
					run_until_settle--;
				}
				else if (run_until_settle > 0)
				{
					run_until_settle--;
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

					/* Crosstalk/sync pulse off - The timing of this is "dont care" */
					if (crosstalk_enabled || (intersensor_sync && is_master && current_ranging_mode == SET_CONTINUOUS_RANGING_MODE))
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
                resetVL53L1Interrupt(pDevice, &status);
				
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
    FLASH_0_read_eeprom_block(EEPROM_FACTORY_SETTINGS_START,settings_buffer,SETTINGS_SIZE);
	uint8_t crc = Crc8(settings_buffer, SETTINGS_SIZE);

	uint8_t * calib_ptr = (uint8_t *)&calibration_data;

	FLASH_0_read_eeprom_block(EEPROM_FACTORY_CALIB_START, calib_ptr, CALIB_SIZE);
	uint8_t crc_calib = Crc8(calib_ptr, CALIB_SIZE);

	/* Do a sanity check on the data if 0x00 (this happens if all zeros are in EEPROM) */
	/* settings_buffer[2] is measurement mode and shouldn't be zero */
	if (crc == 0x00 && settings_buffer[2] == 0x00) crc = 0x01;

	if (crc_calib == 0x00 && calibration_data.customer.global_config__spad_enables_ref_0 == 0x00
		&& calibration_data.customer.global_config__spad_enables_ref_1 == 0x00
		&& calibration_data.customer.global_config__spad_enables_ref_2 == 0x00
		&& calibration_data.customer.global_config__spad_enables_ref_3 == 0x00) crc_calib = 0x01;

    if (FLASH_0_read_eeprom_byte(EEPROM_FACTORY_SETTINGS_START + SETTINGS_SIZE) != crc || FLASH_0_read_eeprom_byte(EEPROM_FACTORY_CALIB_START + CALIB_SIZE + 1) != crc_calib)
    {
        /* Flash LED 4 times */
        flash_led(100, 4, 0);

        /* Use program defaults (do nothing) */
		got_calibration_data = false;

    }

    else
    {
        /* Populate factory settings */
        set_settings(settings_buffer);

		got_calibration_data = true;
    }
}

/**
 * \brief Resets the VL53L0X ranging system. We do this here, because we control the XSHUT pin
 * 
 * 
 * \return void
 */
static void reset_vl53l1x_ranging()
{
	stopContinuous(pDevice, &status);

	/* Set XSHUT to low to turn off (Shutdown is active low). */
	XSHUT_set_level(false);

	/* Wait for shutdown */
	delay_ms(100);

	/* Set XSHUT to high to turn on (Shutdown is active low). */
	XSHUT_set_level(true);

	/* Boot time */
	//delay_ms(T_BOOT_MS);
	waitDeviceReady(pDevice,&status);

	run_until_settle    = SETTLE_MEASUREMENTS;

    init_ranging(pDevice, &status, 0, current_measurement_mode, measurement_budget, &ROI,
			        &calibration_data, got_calibration_data);

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
	    if (command == 0x2e && factory_mode) //Hidden ADDR test command high
		{
			ADDR_IN_set_dir(PORT_DIR_OUT);
			ADDR_IN_set_level(true);
			//delay_ms(200);
			ADDR_IN_set_dir(PORT_DIR_IN);

			ADDR_OUT_set_dir(PORT_DIR_OUT);
			ADDR_OUT_set_level(true);
			//delay_ms(200);
			ADDR_OUT_set_dir(PORT_DIR_IN);
		}
		else if (command == 0x2f && factory_mode) //Hidden ADDR test command low
		{
			ADDR_IN_set_dir(PORT_DIR_OUT);
			ADDR_IN_set_level(false);
			//delay_ms(200);
			ADDR_IN_set_dir(PORT_DIR_IN);

			ADDR_OUT_set_dir(PORT_DIR_OUT);
			ADDR_OUT_set_level(false);
			//delay_ms(200);
			ADDR_OUT_set_dir(PORT_DIR_IN);
		}
		else if (command == INTERSENSOR_SYNC_ENABLE)
		{
			intersensor_sync = true;

			/* Disables crosstalk reduction */
			crosstalk_enabled = false;

			/* Set to continuous if not master, if not and if master it's whatever the mode is
			   This is since the master can be in single or continuous mode and it triggers
			   the other devices whenever it fires off a new read */
			if (!is_master) {
				current_ranging_mode = SET_CONTINUOUS_RANGING_MODE;
				setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
			}

		}
		else if (command == INTERSENSOR_SYNC_DISABLE)
		{
			intersensor_sync = false;
		}
        else if (command == VL53L1X_SHUTDOWN)
        {
		    /* Set XSHUT to low to turn off (Shutdown is active low). */
		    XSHUT_set_level(false);

            //vl53l0xShutdown(device, status);
            vl53l1x_powerstate = 0;
        }

        else if (command == VL53L1X_NOT_SHUTDOWN)
        {
		    /* Set XSHUT to high to turn on (Shutdown is active low). */
		    XSHUT_set_level(true);
			
			//delay_ms(T_BOOT_MS);
			waitDeviceReady(pDevice,&status);

			run_until_settle    = SETTLE_MEASUREMENTS;

			init_ranging(pDevice, &status, 0, current_measurement_mode, measurement_budget, &ROI,
			&calibration_data, got_calibration_data);
            //vl53l0xStartup(device, status);
            vl53l1x_powerstate = 1;
        }

        else if (command == RESET_VL53L1X_RANGING) //Hard reset
        {
			reset_vl53l1x_ranging();
        }

        else if (command == WRITE_CURRENT_SETTINGS_AS_START_UP_DEFAULT)
        {
            get_settings_buffer(settings_buffer, true);

			uint8_t * calib_ptr = (uint8_t *)&calibration_data;
			uint8_t crc_calib = Crc8(calib_ptr, CALIB_SIZE);

			/* Settings buffer is always bigger than actual settings for reduced code */
            settings_buffer[SETTINGS_SIZE] = Crc8(settings_buffer, SETTINGS_SIZE);
            if (!factory_mode)
            {
				
                FLASH_0_write_eeprom_block(EEPROM_USER_SETTINGS_START, settings_buffer, SETTINGS_SIZE + 1);
                FLASH_0_write_eeprom_block(EEPROM_DEVICE_NAME, mappydot_name, 16);
				FLASH_0_write_eeprom_block(EEPROM_USER_CALIB_START, calib_ptr, CALIB_SIZE);
				FLASH_0_write_eeprom_byte(EEPROM_USER_CALIB_START + CALIB_SIZE + 1, crc_calib);

            }

            else
            {
                FLASH_0_write_eeprom_block(EEPROM_FACTORY_SETTINGS_START, settings_buffer, SETTINGS_SIZE + 1);
				FLASH_0_write_eeprom_block(EEPROM_FACTORY_CALIB_START, calib_ptr, CALIB_SIZE);
				FLASH_0_write_eeprom_byte(EEPROM_FACTORY_CALIB_START + CALIB_SIZE + 1, crc_calib);
            }
        }

        else if (command == RESTORE_FACTORY_DEFAULTS) {
			read_default_settings();
			reset_vl53l1x_ranging();
		}

        else if (command == INTERSENSOR_CROSSTALK_REDUCTION_ENABLE)
        {
            crosstalk_enabled = true;

			/* Disable intersensor sync */
			intersensor_sync = false;

            if (!is_master) {
	            current_ranging_mode = SET_CONTINUOUS_RANGING_MODE;
	            setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
            }
        }

        else if (command == INTERSENSOR_CROSSTALK_REDUCTION_DISABLE)
        {
            crosstalk_enabled = false;
        }

		else if (command == ENABLE_CROSSTALK_COMPENSATION)
		{
		    stopContinuous(pDevice, &status);
			setCrosstalk(pDevice, &status, 1);
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
		}

		else if (command == DISABLE_CROSSTALK_COMPENSATION)
		{
		    stopContinuous(pDevice, &status);
			setCrosstalk(pDevice, &status, 0);
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
		}

        else if (command == CALIBRATE_SPAD) 
		{
		    calibrating = true;

			/* Set to single ranging mode (stop measurement) */
			setRangingMode(pDevice, &status, translate_ranging_mode(SET_SINGLE_RANGING_MODE));

		    if(calibrateSPAD(pDevice, &status, &calibration_data) == 0) //returns 0 if success
			{
				flash_led(200,1,0);
				got_calibration_data = true;
			}   
			else
				flash_led(500,4,0); //error

			/* Reset back to original ranging mode */
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
			setRangingMeasurementMode(pDevice, &status, current_measurement_mode, measurement_budget, &ROI);

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
			/* Disables intersensor sync/crosstalk */
			intersensor_sync = false;
			crosstalk_enabled = false;

			current_ranging_mode = SET_CONTINUOUS_RANGING_MODE;
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));

        }

        else if (command == SET_SINGLE_RANGING_MODE)
        {
		    /* Disables intersensor sync/crosstalk */
		    intersensor_sync = false;
			crosstalk_enabled = false;

            current_ranging_mode = SET_SINGLE_RANGING_MODE;
            setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
        }

        else if (command == PERFORM_SINGLE_RANGE) {
			if (current_ranging_mode == SET_SINGLE_RANGING_MODE) 
			{
				startSingleRangingMeasurement(pDevice, &status);

				if (intersensor_sync && is_master)
				{
					/* Create trigger sync */
					ADDR_OUT_set_level(false);
				}       
			} 
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

		else if (command == SIGMA_LIMIT_CHECK_VALUE) {
			signal_limit_check = arg[0]; 
			setLimitChecks(pDevice,&status,signal_limit_check,sigma_limit_check);
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
            /*if (*arg[0] == SHORT_RANGE ||
            	*arg[0] == MED_RANGE ||
            	*arg[0] == LONG_RANGE)
            {*/
            current_measurement_mode = arg[0];

		    stopContinuous(pDevice,&status);

			//Change to selected measurement mode
            setRangingMeasurementMode(pDevice, &status, current_measurement_mode, measurement_budget, &ROI);

			//Reset back to original ranging mode
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));

			//reset_vl53l0x_ranging();

        }
    }

    else if (arg_length == 2)
    {
        if (command == SET_GPIO_THRESHOLD_DISTANCE_IN_MM) gpio_threshold = bytes_to_mm(arg[0],arg[1]);

        else if (command == SET_LED_THRESHOLD_DISTANCE_IN_MM) led_threshold = bytes_to_mm(arg[0],arg[1]);

		else if (command == MEASUREMENT_BUDGET) {
			measurement_budget = bytes_to_mm(arg[0],arg[1]);
			if (measurement_budget < 10) measurement_budget = 10;
			else if (measurement_budget > 1000) measurement_budget = 1000;
			//TODO consolidate this reset into one function
			stopContinuous(pDevice,&status);
			setRangingMeasurementMode(pDevice, &status, current_measurement_mode, measurement_budget, &ROI);

			#ifndef DEV_DISABLE
			/* Initialise Filter */
			bwlpf_init(&low_pass_filter_state,1000/measurement_budget,FILTER_FREQUENCY);
			#endif
			//Reset back to original ranging mode
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
		}

		else if (command == SIGNAL_LIMIT_CHECK_VALUE) {
			signal_limit_check = bytes_to_mm(arg[0],arg[1]);
			setLimitChecks(pDevice,&status,signal_limit_check,sigma_limit_check);
		}

        else if (command == CALIBRATE_DISTANCE_OFFSET) {
		    calibrating = true;

			/* Disable measurement interrupt */
			/* PC3 - PCINT11 (PCMSK1) */
			PCICR &= ~(1 << PCIE1);    // unset PCIE1 to enable PCMSK1 scan
			PCMSK1 &= ~(1 << PCINT11);  // unset PCINT11 to trigger an interrupt on state change

			/* Set to single ranging mode (stop measurement) */
			setRangingMode(pDevice, &status, translate_ranging_mode(SET_SINGLE_RANGING_MODE));

			if (calibrateDistanceOffset(pDevice, &status, &calibration_data, bytes_to_mm(arg[0],arg[1])) == 0)  //returns 0 if success
			{
				flash_led(200,1,0);
				got_calibration_data = true;
			}
			else
				flash_led(500,4,0); //error

			/* Enable measurement interrupt */
			/* PC3 - PCINT11 (PCMSK1) */
			PCICR |= (1 << PCIE1);    // set PCIE1 to enable PCMSK1 scan
			PCMSK1 |= (1 << PCINT11);  // set PCINT11 to trigger an interrupt on state change

			/* Reset back to original ranging mode */
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
			setRangingMeasurementMode(pDevice, &status, current_measurement_mode, measurement_budget, &ROI);

			calibrating = false;
		}	    

		//The order the calibration functions are called does matter : RefSPAD first, offset second and crosstalk third.

        else if (command == CALIBRATE_CROSSTALK) {
		    calibrating = true;

			/* Disable measurement interrupt */
			/* PC3 - PCINT11 (PCMSK1) */
			PCICR &= ~(1 << PCIE1);    // unset PCIE1 to enable PCMSK1 scan
			PCMSK1 &= ~(1 << PCINT11);  // unset PCINT11 to trigger an interrupt on state change

			/* Set to single ranging mode (stop measurement) */
			setRangingMode(pDevice, &status, translate_ranging_mode(SET_SINGLE_RANGING_MODE));

		    if (calibrateCrosstalk(pDevice, &status, &calibration_data, bytes_to_mm(arg[0],arg[1])) == 0)  //returns 0 if success
			{	
				flash_led(200,1,0);
				got_calibration_data = true;
			}
			else
			{
			    flash_led(500,4,0); //error
			}
			
			/* Enable measurement interrupt */
			/* PC3 - PCINT11 (PCMSK1) */
			PCICR |= (1 << PCIE1);    // set PCIE1 to enable PCMSK1 scan
			PCMSK1 |= (1 << PCINT11);  // set PCINT11 to trigger an interrupt on state change

			/* Reset back to original ranging mode */
			setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode));
			setRangingMeasurementMode(pDevice, &status, current_measurement_mode, measurement_budget, &ROI);

			calibrating = false;
		        
		}
    }
	else if (arg_length == 4)
	{
		if (command == REGION_OF_INTEREST)
		{
			//stopContinuous(pDevice, &status);
			ROI.TopLeftX = ((uint8_t)arg[0]) % 16;
			ROI.TopLeftY = ((uint8_t)arg[1]) % 16;
			ROI.BotRightX = ((uint8_t)arg[2]) % 16;
			ROI.BotRightY = ((uint8_t)arg[3]) % 16;
			setRegionOfInterest(pDevice,&status,&ROI);
			//setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode), current_measurement_mode, measurement_budget);

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
				//led_pulse = 0;
			}
		}
		// Erase all settings/EEPROM (when in factory mode)
		if (command == WIPE_ALL_SETTINGS &&
			arg[0] == '>' && arg[1] == '<' &&
			arg[2] == '>' && arg[3] == '<' &&
			arg[4] == '>' && arg[5] == '<' && factory_mode) {
			for (int i = 0; i < 512; i++)
			{
				FLASH_0_write_eeprom_byte(i, 0xFF);
			}

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

    
    buffer[0] = (measurement_budget >> 8 ) & 0xff;
    buffer[1] = (measurement_budget      ) & 0xff;
    buffer[2] = current_ranging_mode;
    buffer[3] = current_measurement_mode;
    buffer[4] = led_mode;
    mm_to_bytes(tmp_buffer, led_threshold);
    buffer[5] = tmp_buffer[0];
    buffer[6] = tmp_buffer[1];
    buffer[7] = gpio_mode;
    mm_to_bytes(tmp_buffer, gpio_threshold);
    buffer[8] = tmp_buffer[0];
    buffer[9] = tmp_buffer[1];
    buffer[10] = filtering_enabled;
	buffer[11] = averaging_enabled;
	buffer[12] = averaging_size;
    buffer[13] = crosstalk_enabled;
    buffer[14] = intersensor_crosstalk_delay;
	buffer[15] = intersensor_crosstalk_timeout;
    buffer[16] = vl53l1x_powerstate;
	buffer[17] = (uint8_t)ROI.TopLeftX;
	buffer[18] = (uint8_t)ROI.TopLeftY;
	buffer[19] = (uint8_t)ROI.BotRightX;
	buffer[20] = (uint8_t)ROI.BotRightY;
    if (!store_settings) {
		buffer[21] = calibration_data.optical_centre.x_centre >> 4;
		buffer[22] = calibration_data.optical_centre.y_centre >> 4;
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
    measurement_budget = bytes_to_mm(buffer[0],buffer[1]);
    current_ranging_mode = buffer[2];
    current_measurement_mode = buffer[3];
    led_mode = buffer[4];
    led_threshold = bytes_to_mm(buffer[5],buffer[6]);
    gpio_mode = buffer[7];
    gpio_threshold = bytes_to_mm(buffer[8],buffer[9]);
    filtering_enabled = buffer[10];
	averaging_enabled = buffer[11];
	averaging_size = buffer[12];
    crosstalk_enabled = buffer[13];
    intersensor_crosstalk_delay = buffer[14];
	intersensor_crosstalk_timeout = buffer[15];
    vl53l1x_powerstate = buffer[16];
	ROI.TopLeftX = buffer[17];
	ROI.TopLeftY = buffer[18];
	ROI.BotRightX = buffer[19];
	ROI.BotRightY =	buffer[20];
	
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
		read_interrupt = 0;
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

	else if (command == CHECK_INTERRUPT)
	{
		tx_buffer[0] = read_interrupt;
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
        return 23;
    }

    else if (command == READ_NONFILTERED_VALUE)
    {
        mm_to_bytes(tx_buffer, real_distance);
        return 2;
    }

	else if (command == AMBIENT_RATE_RETURN)
	{
		mm_to_bytes(tx_buffer, ambient_rate_rtn_mega_cps);
		return 2;
	}

	else if (command == SIGNAL_RATE_RETURN)
	{
		mm_to_bytes(tx_buffer, signal_rate_rtn_mega_cps);
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

/* Crosstalk/sync interrupt fired */
ISR (PCINT0_vect)
{

	/* Pass on pulse straight away to next in chain */
	if (intersensor_sync)
	{
		ADDR_OUT_set_level(ADDR_IN_get_level());
	}

	if(ADDR_IN_get_level() == 0)
	{
		crosstalk_sync_interrupt_fired = true;
	}
}


/* No interrupt overflow (this will fire every ~2000ms if no interrupt arrived) */
ISR(TIMER4_OVF_vect)
{
    interrupt_timeout_interrupt_fired = true;
}

