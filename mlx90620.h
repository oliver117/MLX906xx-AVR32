/*
 * mlx90620.h
 *
 * Created: 08.09.2013 15:00:06
 *  Author: Oliver Kleinke
 */ 


#ifndef MLX90620_H_
#define MLX90620_H_

// note: row numbers from 0 to 3, index i -- column numbers from 0 to 15, index j
// offset = row + 4 * column, or: i + 4 * j

#include "software_twi_master.h"

#define MLX90620_FOV_PIXEL_X	3.75f // 60 / 16
#define MLX90620_FOV_PIXEL_Y	4.10f // 16.4 / 16

// addresses, hard coded
#define MLX90620_EEPROM_TWI_ADDRESS	0x50
#define MLX90620_RAM_TWI_ADDRESS	0x60

// opcodes, 'OR' them with the address
#define MLX90620_READ_RAM	0x02
#define MLX90620_WRITE_CONFIG_REGISTER	0x03
#define MLX90620_WRITE_TRIMMING_REGISTER	0x04

// EEPROM addresses, datasheet, table 12 ff.
#define MLX90620_EEPROM_A_FIRST	0x00
#define MLX90620_EEPROM_A_LAST	0x3F

#define MLX90620_EEPROM_B_FIRST	0x40
#define MLX90620_EEPROM_B_LAST	0x7F

#define MLX90620_EEPROM_DELTA_ALPHA_FIRST	0x80
#define MLX90620_EEPROM_DELTA_ALPHA_LAST	0xBF

#define MLX90620_EEPROM_A_CP	0xD4
#define MLX90620_EEPROM_B_CP	0xD5

#define MLX90620_EEPROM_ALPHA_CP_L	0xD6
#define MLX90620_EEPROM_ALPHA_CP_H	0xD7

#define MLX90620_EEPROM_TGC	0xD8
#define MLX90620_EEPROM_B_SCALE	0xD9

#define MLX90620_EEPROM_PTAT_V_TH_L	0xDA
#define MLX90620_EEPROM_PTAT_V_TH_H	0xDB

#define MLX90620_EEPROM_PTAT_K_T1_L	0xDC
#define MLX90620_EEPROM_PTAT_K_T1_H	0xDD

#define MLX90620_EEPROM_PTAT_K_T2_L	0xDE
#define MLX90620_EEPROM_PTAT_K_T2_H	0xDF

#define MLX90620_EEPROM_ALPHA_0_L	0xE0
#define MLX90620_EEPROM_ALPHA_0_H	0xE1

#define MLX90620_EEPROM_ALPHA_0_SCALE	0xE2
#define MLX90620_EEPROM_DELTA_ALPHA_SCALE	0xE3

#define MLX90620_EEPROM_EPSILON_L	0xE4
#define MLX90620_EEPROM_EPSILON_H	0xE5

#define MLX90620_EEPROM_CONFIG_L	0xF5
#define MLX90620_EEPROM_CONFIG_H	0xF6

#define MLX90620_EEPROM_OSCILLATOR_TRIM	0xF7

#define MLX90620_EEPROM_CHIP_ID_FIRST	0xF8
#define MLX90620_EEPROM_CHIP_ID_LAST	0xFF

typedef struct {
	// datasheet table 12
	uint8_t a[4][16];
	uint8_t b[4][16];
	uint8_t delta_alpha[4][16];
	uint8_t reserved1[20];
	// datasheet table 13
	uint8_t a_cp;
	uint8_t b_cp;
	uint8_t alpha_cp_low; // a.k.a. delta_alpha_cp_low
	uint8_t alpha_cp_high; // a.k.a. delta_alpha_cp_high
	// datasheet table 14
	uint8_t tgc;
	uint8_t b_scale;
	uint8_t v_th_low;
	uint8_t v_th_high;
	uint8_t k_t1_low;
	uint8_t k_t1_high;
	uint8_t k_t2_low;
	uint8_t k_t2_high;
	// datasheet table 15
	uint8_t alpha_0_low;
	uint8_t alpha_0_high;
	uint8_t alpha_0_scale;
	uint8_t delta_alpha_scale;
	uint8_t epsilon_low;
	uint8_t epsilon_high;
	uint8_t reserved2[15];
	// datasheet table 16
	uint8_t config_low;
	uint8_t config_high;
	uint8_t osc_trim;
	uint8_t chip_id[8];
} __attribute__ ((packed)) mlx90620_eeprom;

// RAM addresses
#define MLX90620_RAM_IR_FIRST	0x00
#define MLX90620_RAM_IR_LAST	0x3F
#define MLX90620_RAM_PTAT	0x90
#define MLX90620_RAM_V_CP	0x91

// config register addresses
#define MLX90620_CONFIG_REGISTER	0x92
#define MLX90620_TRIMMING_REGISTER	0x93

// dumps the whole EEPROM image into the given array (256x8)
SOFTWARE_TWI_STATUS mlx90620_read_eeprom(software_twi_t *twi, mlx90620_eeprom *eeprom);

// writes the oscillator trimming value provided by melexis (in EEPROM) into the trimming register
SOFTWARE_TWI_STATUS mlx90620_write_oscillator_trimming_register(software_twi_t *twi, mlx90620_eeprom *eeprom);

// writes the configuration value provided by melexis (in EEPROM) into the config register
SOFTWARE_TWI_STATUS mlx90620_write_config_register(software_twi_t *twi, mlx90620_eeprom *eeprom);

// reads the ptat data (ambient temperature measurement)
SOFTWARE_TWI_STATUS mlx90620_read_ptat(software_twi_t *twi, uint16_t *ptat_data);

SOFTWARE_TWI_STATUS mlx90620_read_compensation_pixel(software_twi_t *twi, int16_t *V_cp);

SOFTWARE_TWI_STATUS mlx90620_read_ir_array(software_twi_t *twi, int16_t ir_data[4][16]);


// converts a received word (msbyte, lsbyte) in twos complement to an int16_t
int16_t mlx90620_twos_complement_word(uint8_t msbyte, uint8_t lsbyte);

// converts a received byte in twos complement to an int8_t
int8_t mlx90620_twos_complement_byte(uint8_t byte);

// calculates the ambient temperature from the given PTAT sensor value
float mlx90620_calculate_ambient_temperature(mlx90620_eeprom *eeprom, uint16_t ptat_data);

// calculates the object temperature for a single ir array pixel
float mlx90620_calculate_pixel_temperature(mlx90620_eeprom *eeprom, float T_ambient, uint8_t row, uint8_t column, int16_t ir_data[4][16], int16_t V_CP);

// calculates the object temperatures for the whole ir array
void mlx90620_calculate_whole_frame_pixel_temperature(mlx90620_eeprom *eeprom, float T_ambient, int16_t ir_data[4][16], int16_t V_cp, float temperatures[4][16]);

// finds the hottest pixel in the ir array and returns the temperature + direction in degrees
bool mlx90620_find_hot_spot(float temperatures[4][16], float T_Ambient, float threshold, float *hot_temp, float *deg_x, float *deg_y);

#endif /* MLX90620_H_ */