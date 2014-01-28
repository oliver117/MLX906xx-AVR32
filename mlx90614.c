/*
 * mlx90614.c
 *
 * Created: 08.09.2013 12:32:24
 *  Author: Oliver Kleinke
 */

#include "mlx90614.h"
#include "software_twi_master.h"

SOFTWARE_TWI_STATUS mlx90614_read(software_twi_t *twi, uint8_t twi_address, uint8_t command, uint8_t *msbyte, uint8_t *lsbyte) {
	
	uint8_t data[3]; // lsbyte, msbyte, pec
	
	SOFTWARE_TWI_STATUS status = soft_twi_receive(twi, twi_address, &command, 1, data, 3);
	
	*lsbyte = data[0];
	*msbyte = data[1];
	// pec discarded
	
	return status;
}

bool mlx90614_read_temperature_ambient(software_twi_t *twi, uint8_t twi_address, uint8_t *msbyte, uint8_t *lsbyte) {
	return (SOFTWARE_TWI_SUCCESS == mlx90614_read(twi, twi_address, MLX90614_RAM_ACCESS | MLX90614_RAM_TEMPERATURE_AMBIENT, msbyte, lsbyte));
}

bool mlx90614_read_temperature_object1(software_twi_t *twi, uint8_t twi_address, uint8_t *msbyte, uint8_t *lsbyte) {
	return (SOFTWARE_TWI_SUCCESS == mlx90614_read(twi, twi_address, MLX90614_RAM_ACCESS | MLX90614_RAM_TEMPERATURE_OBJECT_1, msbyte, lsbyte));
}

bool mlx90614_read_temperature_object2(software_twi_t *twi, uint8_t twi_address, uint8_t *msbyte, uint8_t *lsbyte) {
	return (SOFTWARE_TWI_SUCCESS == mlx90614_read(twi, twi_address, MLX90614_RAM_ACCESS | MLX90614_RAM_TEMPERATURE_OBJECT_2, msbyte, lsbyte));
}

bool mlx90614_get_temperature_ambient(software_twi_t *twi, uint8_t twi_address, float *temp_ambient) {
	uint8_t msbyte, lsbyte;

	bool status = mlx90614_read_temperature_ambient(twi, twi_address, &msbyte, &lsbyte);
	
	if (status) {
		*temp_ambient = mlx90614_calculate_temperature(msbyte, lsbyte);
	}
	
	return status;
}

bool mlx90614_get_temperature_object1(software_twi_t *twi, uint8_t twi_address, float *temp_object1) {
	uint8_t msbyte, lsbyte;

	bool status = mlx90614_read_temperature_object1(twi, twi_address, &msbyte, &lsbyte);
	
	if (status) {
		*temp_object1 = mlx90614_calculate_temperature(msbyte, lsbyte);
	}
	
	return status;
}

bool mlx90614_get_temperature_object2(software_twi_t *twi, uint8_t twi_address, float *temp_object2) {
	uint8_t msbyte, lsbyte;

	bool status = mlx90614_read_temperature_object2(twi, twi_address, &msbyte, &lsbyte);
	
	if (status) {
		*temp_object2 = mlx90614_calculate_temperature(msbyte, lsbyte);
	}
	
	return status;
}

float mlx90614_calculate_temperature(uint8_t msbyte, uint8_t lsbyte) {
	uint16_t temp = msbyte * 256 + lsbyte;
	
	return 0.02 * temp - 273.15;
}