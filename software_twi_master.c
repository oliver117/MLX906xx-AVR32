/*
* software_twi_master.c
*
* Created: 08.09.2013 12:33:22
*  Author: Oliver Kleinke
*/

#include "software_twi_master.h"

#include "delay.h"
#include "gpio.h"
#include "cycle_counter.h"
#include "compiler.h"

#include "../basics.h"

#ifndef cpu_relax
#define cpu_relax() __asm__ __volatile__("sub pc, pc, -4" ::: "memory", "cc")
#endif

#ifndef delay_us
#define delay_us(delay) cpu_delay_us(delay, CPU_SPEED)
#endif

/************************************************************************/
/* LOW LEVEL FUNCTIONS                                                  */
/************************************************************************/

void soft_twi_init(software_twi_t *twi) {
	// gpio unit controls scl and sda pins
	gpio_enable_gpio_pin(twi->scl_pin);
	gpio_enable_gpio_pin(twi->sda_pin);
	
	// enable open drain mode & pull up for both pins
	gpio_local_init_gpio_open_drain_pin(twi->scl_pin);
	gpio_local_init_gpio_open_drain_pin(twi->sda_pin);
	
	// enable the local bus interface
	gpio_local_init();
	
	return;
}

void soft_twi_start(software_twi_t *twi) {
	gpio_local_clr_gpio_open_drain_pin(twi->sda_pin);
	soft_twi_delay();
	gpio_local_clr_gpio_open_drain_pin(twi->scl_pin);
	soft_twi_delay();

	return;
}

void soft_twi_stop(software_twi_t *twi)  {
	// pull down the sda line
	gpio_local_clr_gpio_open_drain_pin(twi->sda_pin);
	soft_twi_delay();

	// release both lines to generate a stop signal
	gpio_local_set_gpio_open_drain_pin(twi->scl_pin);
	soft_twi_delay();
	gpio_local_set_gpio_open_drain_pin(twi->sda_pin);
	soft_twi_delay();

	return;
}

void soft_twi_repeat_start(software_twi_t *twi) {
	// release sda
	gpio_local_set_gpio_open_drain_pin(twi->sda_pin);
	soft_twi_delay();
	
	// relase scl
	gpio_local_set_gpio_open_drain_pin(twi->scl_pin);
	soft_twi_delay();
	
	// generate a start signal
	soft_twi_start(twi);
	
	return;
}

void soft_twi_write_bit(software_twi_t *twi, uint8_t bit) {
	if (bit) {
		// set the sda line
		gpio_local_set_gpio_open_drain_pin(twi->sda_pin);
		soft_twi_delay();
		
		// let scl rise
		gpio_local_set_gpio_open_drain_pin(twi->scl_pin);
		soft_twi_delay();

		// clock stretching: wait until SCL is actually HIGH
		while (!gpio_local_get_pin_value(twi->scl_pin)) {
			cpu_relax();
		}

		// pull down scl
		gpio_local_clr_gpio_open_drain_pin(twi->scl_pin);
		soft_twi_delay();
		} else {
		// set the sda line to LOW
		gpio_local_clr_gpio_open_drain_pin(twi->sda_pin);
		soft_twi_delay();
		
		// let scl rise
		gpio_local_set_gpio_open_drain_pin(twi->scl_pin);
		soft_twi_delay();

		// clock stretching: wait until SCL is actually HIGH
		while (!gpio_local_get_pin_value(twi->scl_pin)) {
			cpu_relax();
		}
		
		// pull down scl
		gpio_local_clr_gpio_open_drain_pin(twi->scl_pin);
		soft_twi_delay();
	}
}

uint8_t soft_twi_read_bit(software_twi_t *twi) {
	// release the sda line, so the slave can take control
	gpio_local_set_gpio_open_drain_pin(twi->sda_pin);
	soft_twi_delay();

	// release SCL
	gpio_local_set_gpio_open_drain_pin(twi->scl_pin);
	soft_twi_delay();
	
	// clock stretching: wait until SCL is actually HIGH
	while (!gpio_local_get_pin_value(twi->scl_pin)) {
		cpu_relax();
	}
	
	// get the value, set by the slave
	uint8_t bit = gpio_local_get_pin_value(twi->sda_pin);
	
	// pull down SCL
	gpio_local_clr_gpio_open_drain_pin(twi->scl_pin);
	soft_twi_delay();
	
	return bit;
}

SOFTWARE_TWI_STATUS soft_twi_write(software_twi_t *twi, uint8_t data) {
	// send 8 bits
	soft_twi_write_bit(twi, (data >> 7 & 1));
	soft_twi_write_bit(twi, (data >> 6 & 1));
	soft_twi_write_bit(twi, (data >> 5 & 1));
	soft_twi_write_bit(twi, (data >> 4 & 1));
	soft_twi_write_bit(twi, (data >> 3 & 1));
	soft_twi_write_bit(twi, (data >> 2 & 1));
	soft_twi_write_bit(twi, (data >> 1 & 1));
	soft_twi_write_bit(twi, (data >> 0 & 1));
	
	// read ACK/NACK bit
	
	uint8_t bit = soft_twi_read_bit(twi);

	if (0 == bit) { // ACK = SDA pulled LOW by slave
		return SOFTWARE_TWI_SUCCESS;
	} else { // NACK = SDA not pulled down (=HIGH)
		return SOFTWARE_TWI_NACK;
	}
}

SOFTWARE_TWI_STATUS soft_twi_write_address(software_twi_t *twi, uint8_t address, bool write) {
	if (write) {
		return soft_twi_write(twi, address << 1);
	} else {
		return soft_twi_write(twi, (address << 1) | 1);
	}
}

SOFTWARE_TWI_STATUS soft_twi_read(software_twi_t *twi, uint8_t *data) {
	// clear
	*data = 0x00;
	// receive the bits from MSB to LSB
	*data |= soft_twi_read_bit(twi) << 7;
	*data |= soft_twi_read_bit(twi) << 6;
	*data |= soft_twi_read_bit(twi) << 5;
	*data |= soft_twi_read_bit(twi) << 4;
	*data |= soft_twi_read_bit(twi) << 3;
	*data |= soft_twi_read_bit(twi) << 2;
	*data |= soft_twi_read_bit(twi) << 1;
	*data |= soft_twi_read_bit(twi) << 0;
	
	// ACK the byte
	soft_twi_write_bit(twi, 0x00);
	
	return SOFTWARE_TWI_SUCCESS;
}

SOFTWARE_TWI_STATUS soft_twi_write_n(software_twi_t *twi, uint8_t *data, unsigned int length) {
	unsigned int i;
	for (i = 0; i < length; ++i) {
		if (SOFTWARE_TWI_SUCCESS != soft_twi_write(twi, data[i])) {
			return SOFTWARE_TWI_FAIL;
		}
	}
	
	return SOFTWARE_TWI_SUCCESS;
}

SOFTWARE_TWI_STATUS soft_twi_read_n(software_twi_t *twi, uint8_t *data, unsigned int length) {
	unsigned int i;
	for (i = 0; i < length; ++i) {
		if (SOFTWARE_TWI_SUCCESS != soft_twi_read(twi, &data[i])) {
			return SOFTWARE_TWI_FAIL;
		}
	}
	
	return SOFTWARE_TWI_SUCCESS;
}

/************************************************************************/
/* HIGH LEVEL FUNCTIONS                                                 */
/************************************************************************/

SOFTWARE_TWI_STATUS soft_twi_send(software_twi_t *twi, uint8_t twi_address, uint8_t *command, unsigned int command_length, uint8_t *data, unsigned int data_length)
{
	SOFTWARE_TWI_STATUS status;

	// send the slave device the command / address
	soft_twi_start(twi);
	status = soft_twi_write_address(twi, twi_address, SOFTWARE_TWI_WRITE);
	if (SOFTWARE_TWI_SUCCESS != status) { goto end; }
		
	status = soft_twi_write_n(twi, command, command_length);
	if (SOFTWARE_TWI_SUCCESS != status) { goto end; }

	status = soft_twi_write_n(twi, data, data_length);
	if (SOFTWARE_TWI_SUCCESS != status) { goto end; }

end:
	// free the bus
	soft_twi_stop(twi);

	return status;
}

SOFTWARE_TWI_STATUS soft_twi_receive(software_twi_t *twi, uint8_t twi_address, uint8_t *command, unsigned int command_length, uint8_t *data, unsigned int data_length) {
	SOFTWARE_TWI_STATUS status;

	// send the slave device the command / address
	soft_twi_start(twi);
	status = soft_twi_write_address(twi, twi_address, SOFTWARE_TWI_WRITE);
	if (SOFTWARE_TWI_SUCCESS != status) { goto end; }
		
	status = soft_twi_write_n(twi, command, command_length);
	if (SOFTWARE_TWI_SUCCESS != status) { goto end; }

	// repeated start to initiate the data transfer
	soft_twi_repeat_start(twi);

	status = soft_twi_write_address(twi, twi_address, SOFTWARE_TWI_READ);
	if (SOFTWARE_TWI_SUCCESS != status) { goto end; }

	status = soft_twi_read_n(twi, data, data_length);
	if (SOFTWARE_TWI_SUCCESS != status) { goto end; }

end:
	// free the bus
	soft_twi_stop(twi);

	return status;
}
