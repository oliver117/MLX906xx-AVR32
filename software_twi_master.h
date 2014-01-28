/*
 * software_twi_master.h
 *
 * Created: 07.09.2013 13:54:48
 *  Author: Oliver Kleinke
 */ 


#ifndef SOFTWARE_TWI_MASTER_H_
#define SOFTWARE_TWI_MASTER_H_

#include "stdint.h"
#include "compiler.h" // for bool

// adjust the delay duration to change the bus speed
#define soft_twi_delay() (delay_us (1))

#define SOFTWARE_TWI_WRITE	true
#define SOFTWARE_TWI_READ	false

typedef enum SOFTWARE_TWI_STATUS {
	SOFTWARE_TWI_SUCCESS = 0,
	SOFTWARE_TWI_FAIL = -1,
	SOFTWARE_TWI_NACK = -2
} SOFTWARE_TWI_STATUS;

typedef struct {
	uint32_t scl_pin;
	uint32_t sda_pin;
} software_twi_t;

/************************************************************************/
/* LOW LEVEL FUNCTIONS                                                  */
/************************************************************************/

// configures the I/O pins given by twi for GPIO control and switch on the local bus interface
void soft_twi_init(software_twi_t *twi);

// sends a twi start signal
void soft_twi_start(software_twi_t *twi);

// sends a twi stop signal
void soft_twi_stop(software_twi_t *twi);

// sends a repeated start condition to keep control of the bus
void soft_twi_repeat_start(software_twi_t *twi);

// sends one bit over the bus
void soft_twi_write_bit(software_twi_t *twi, uint8_t bit);

// reads one bit from the bus
uint8_t soft_twi_read_bit(software_twi_t *twi);

// writes a whole byte to the bus
SOFTWARE_TWI_STATUS soft_twi_write(software_twi_t *twi, uint8_t data);

// sends a slave address with write/read bit
SOFTWARE_TWI_STATUS soft_twi_write_address(software_twi_t *twi, uint8_t address, bool write);

// reads a whole byte from the bus
SOFTWARE_TWI_STATUS soft_twi_read(software_twi_t *twi, uint8_t *data);

// writes multiple bytes to the bus
SOFTWARE_TWI_STATUS soft_twi_write_n(software_twi_t *twi, uint8_t *data, unsigned int length);

// reads multiple bytes from the bus
SOFTWARE_TWI_STATUS soft_twi_read_n(software_twi_t *twi, uint8_t *data, unsigned int length);

/************************************************************************/
/* HIGH LEVEL FUNCTIONS                                                 */
/************************************************************************/

// sends multiple bytes to a slave device after sending a command(+internal address) to the slave device
SOFTWARE_TWI_STATUS soft_twi_send(software_twi_t *twi, uint8_t twi_address, uint8_t *command, unsigned int command_length, uint8_t *data, unsigned int data_length);

// receives multiple bytes from a slave device after sending a command(+internal address) to the slave device
SOFTWARE_TWI_STATUS soft_twi_receive(software_twi_t *twi, uint8_t twi_address, uint8_t *command, unsigned int command_length, uint8_t *data, unsigned int data_length);

#endif /* SOFTWARE_TWI_MASTER_H_ */
