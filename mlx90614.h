/*
 * mlx90614.h
 *
 * Created: 08.09.2013 10:57:05
 *  Author: Oliver Kleinke
 */ 


#ifndef MLX90614_H_
#define MLX90614_H_

#include "software_twi_master.h"

// opcodes, 'OR' them with the address
#define MLX90614_RAM_ACCESS	0x00
#define MLX90614_EEPROM_ACCESS	0x20

// EEPROM addresses
#define MLX90614_EEPROM_TO_MAX	0x00
#define MLX90614_EEPROM_TO_MIN	0x01
#define MLX90614_EEPROM_PWMCTRL	0x02
#define MLX90614_EEPROM_TA_RANGE	0x03
#define MLX90614_EEPROM_EMISSIVITY_CORRECTION_COEFFICIENT	0x04
#define MLX90614_EEPROM_CONFIG_REGISTER_1	0x05
#define MLX90614_EEPROM_SMBUS_ADDRESS	0x0E
#define MLX90614_EEPROM_ID_NUMBER_1	0x1C
#define MLX90614_EEPROM_ID_NUMBER_2	0x1D
#define MLX90614_EEPROM_ID_NUMBER_3	0x1E
#define MLX90614_EEPROM_ID_NUMBER_4	0x1F

// RAM addresses
#define MLX90614_RAM_RAW_DATA_IR_CHANNEL_1	0x04
#define MLX90614_RAM_RAW_DATA_IR_CHANNEL_2	0x05
#define MLX90614_RAM_TEMPERATURE_AMBIENT	0x06
#define MLX90614_RAM_TEMPERATURE_OBJECT_1	0x07
#define MLX90614_RAM_TEMPERATURE_OBJECT_2	0x08

// soft twi functions
// reads one word (high byte, low byte) from the designated slave after sending the given command
SOFTWARE_TWI_STATUS mlx90614_read(software_twi_t *twi, uint8_t twi_address, uint8_t command, uint8_t *msbyte, uint8_t *lsbyte);

// shortcuts
// reads the ambient temperature raw value
bool mlx90614_read_temperature_ambient(software_twi_t *twi, uint8_t twi_address, uint8_t *msbyte, uint8_t *lsbyte);

// reads the object1 temperature raw value
bool mlx90614_read_temperature_object1(software_twi_t *twi, uint8_t twi_address, uint8_t *msbyte, uint8_t *lsbyte);

// reads the object2 temperature raw value
bool mlx90614_read_temperature_object2(software_twi_t *twi, uint8_t twi_address, uint8_t *msbyte, uint8_t *lsbyte);

// reads and calculates the ambient temperature value (in degrees celsius)
bool mlx90614_get_temperature_ambient(software_twi_t *twi, uint8_t twi_address, float *temp_ambient);

// reads and calculates the object1 temperature value (in degrees celsius)
bool mlx90614_get_temperature_object1(software_twi_t *twi, uint8_t twi_address, float *temp_object1);

// reads and calculates the object2 temperature value (in degrees celsius)
bool mlx90614_get_temperature_object2(software_twi_t *twi, uint8_t twi_address, float *temp_object2);

// calculates a temperate value (in degrees celsius) for a given word (high byte, low byte)
float mlx90614_calculate_temperature(uint8_t msbyte, uint8_t lsbyte);

#endif /* MLX90614_H_ */