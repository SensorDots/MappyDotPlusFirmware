#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include "main.h"

#include "I2C_slave.h"
#include "mappydot_reg.h"

volatile uint8_t command_size = 0;

volatile uint8_t send_size = 0;

void i2c_slave_init(uint8_t address)
{
	register_address = 0xff;
    // load address into TWI address register
    TWAR1 = (address << 1);
    // set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
    TWCR1 = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
}

void i2c_slave_stop(void)
{
    // clear acknowledge and enable bits
    TWCR1 &= ~( (1<<TWEA) | (1<<TWEN) );
}

ISR( TWI1_vect )
{
    //Temp data
    uint8_t data;

	switch( (TWSR1 & 0xF8) )
	{
		case TW_SR_SLA_ACK:				// 0x60 Own SLA+W has been received ACK has been returned. Expect to receive data.
//		case TW_SR_ARB_LOST_SLA_ACK:	// 0x68 Own SLA+W has been received; ACK has been returned. RESET interface.
			register_address = 0xFF;   
			txbuffer_count = 0;
			rxbuffer_count = 0;
			register_address_reset = 1;

			TWCR1 = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be DATA.
			break;

		case TW_SR_DATA_ACK:			// 0x80 Previously addressed with own SLA+W; Data received; ACK'd
		case TW_SR_GCALL_DATA_ACK:			// 0x90 Previously addressed with general call; Data received; ACK'd

			data = TWDR1;
			if (register_address == 0xFF) {
				register_address = data;
				if (!is_read_command(register_address))
				{
					//Check command
					command_size = check_command_size(register_address);

					//if command is only single byte (register), we react here
					if (command_size == 0) main_process_rx_command(register_address, rxbuffer, 0);
				}
			} else {
			    // store the data at the current address
				rxbuffer[rxbuffer_count] = data;
				rxbuffer_count++;

				// if command has more data to receive
				if(rxbuffer_count < command_size)
				{
					// clear TWI interrupt flag, prepare to receive next byte and acknowledge
					//TWCR1 |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
				}
				else if (rxbuffer_count == command_size)
				{
					//react to long command here
					main_process_rx_command(register_address, rxbuffer, command_size);

					// clear TWI interrupt flag, prepare to receive next byte and acknowledge
					//TWCR1 |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
				}

				else
				{
					// Incorrect command length, clear TWI interrupt flag, prepare to receive more bytes and don't acknowledge
					TWCR1 |= (1<<TWIE) | (1<<TWINT) | (0<<TWEA) | (1<<TWEN);
					return;
				}
				command_size = check_command_size(register_address);
				//if command is only single byte (register), we react here
				if (command_size == 0) main_process_rx_command(register_address, rxbuffer, 0);
			}		
			
			TWCR1 = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be more DATA.
  			break;
			
		case TW_SR_GCALL_ACK:				// 0x70 General call address has been received; ACK has been returned
//		case TW_SR_ARB_LOST_GCALL_ACK:	// 0x78 General call address has been received; ACK has been returned
			// TODO: Set General Address flag
			TWCR1 = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be DATA.
			break;

		case TW_ST_SLA_ACK:				// 0xA8 Own SLA+R has been received; ACK has been returned. Load DATA.

			//if (register_address == 0xff) register_address = READ_DISTANCE; 
			if (!is_read_command(register_address) || !register_address_reset) register_address = READ_DISTANCE; //If we haven't received read register yet, set it to read distance
			send_size = main_process_tx_command(register_address, txbuffer);
			// copy the specified buffer address into the TWDR register for transmission
			register_address_reset = 0;
			TWDR1 = txbuffer[0];
			//Set to next byte.
			txbuffer_count = 1;
			// clear TWI interrupt flag, prepare to send next byte and receive acknowledge
			TWCR1 |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
			break;

//		case TW_ST_ARB_LOST_SLA_ACK:	// 0xB0 Own SLA+R has been received; ACK has been returned
		case TW_ST_DATA_ACK:				// 0xB8 Data byte in TWDR has been transmitted; ACK has been received. Load DATA.
			if (txbuffer_count < send_size)
			{
				TWDR1 = txbuffer[txbuffer_count];
					//Set to next byte.
					txbuffer_count++;
			}
			else
			{
				
				// the buffer is empty. Too much data was asked for.
				TWDR1 = 0xff;
			}
			TWCR1 = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event.
			break;

		case TW_ST_DATA_NACK:				// 0xC0 Data byte in TWDR has been transmitted; NOT ACK has been received. End of Sending.
			TWCR1 = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be new message.
			break;

		case TW_SR_STOP:			// 0xA0 A STOP condition or repeated START condition has been received while still addressed as Slave
		    //repeated_start = 1;
			TWCR1 = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);
			break;

		case TW_SR_DATA_NACK:			// 0x88 Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
		case TW_SR_GCALL_DATA_NACK:			// 0x98 Previously addressed with general call; data has been received; NOT ACK has been returned
		case TW_ST_LAST_DATA:	// 0xC8 Last byte in TWDR has been transmitted (TWEA = 0); ACK has been received
		case TW_NO_INFO:					// 0xF8 No relevant state information available; TWINT = 0
		case TW_BUS_ERROR:					// 0x00 Bus error due to an illegal START or STOP condition
						
			// Recover from TWI_BUS_ERROR
			TWCR1 &= ~( (1<<TWEA) | (1<<TWEN) ); //Stop bus

			TWCR1 = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);
			break;
		default:
			TWCR1 = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);		// Prepare for next event. Should be more DATA.
			break;
	}
}