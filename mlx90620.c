
/*
 * mlx90620.c
 *
 * Created: 09.09.2013 04:43:01
 *  Author: Oliver Kleinke
 */

#include "mlx90620.h"

#include "software_twi_master.h"

#include "math.h"

SOFTWARE_TWI_STATUS mlx90620_read_eeprom(software_twi_t *twi, mlx90620_eeprom *eeprom) {
	// read 255 bytes from address 0x00
	uint8_t command = 0x00;
	return soft_twi_receive(twi, MLX90620_EEPROM_TWI_ADDRESS, &command, 1, (uint8_t *) eeprom, 256);
}

SOFTWARE_TWI_STATUS mlx90620_write_oscillator_trimming_register(software_twi_t *twi, mlx90620_eeprom *eeprom) {	
	uint8_t command = MLX90620_WRITE_TRIMMING_REGISTER;
	
	// order: lsbyte_check, lsbyte, msbyte_check, msbyte
	// check byte calculation: check = byte - 0xAA
	// msbyte is always 0x00
	uint8_t msbyte = 0x00;

	uint8_t data[] = {
		eeprom->osc_trim -  0xAA,
		eeprom->osc_trim,
		msbyte - 0xAA,
		msbyte
	};
	
	return soft_twi_send(twi, MLX90620_RAM_TWI_ADDRESS, &command, 1, data, 4);
}

SOFTWARE_TWI_STATUS mlx90620_write_config_register(software_twi_t *twi, mlx90620_eeprom *eeprom) {	
	uint8_t command = MLX90620_WRITE_CONFIG_REGISTER;
		
	// order: lsbyte_check, lsbyte, msbyte_check, msbyte
	// check byte calculation: check = byte - 0x55
	uint8_t data[] = {
		eeprom->config_low -  0x55,
		eeprom->config_low,
		eeprom->config_high -  0x55,
		eeprom->config_high
	};
		
	return soft_twi_send(twi, MLX90620_RAM_TWI_ADDRESS, &command, 1, data, 4);
}

SOFTWARE_TWI_STATUS mlx90620_read_ptat(software_twi_t *twi, uint16_t *ptat_data) {
	SOFTWARE_TWI_STATUS status;

	uint8_t command[] = {
		MLX90620_READ_RAM,
		MLX90620_RAM_PTAT, // start address
		0x00, // single value no step
		0x01 // single read
	};
	
	uint8_t data[] = {
		0, // lsbyte
		0  // msbyte
	};
	
	status = soft_twi_receive(twi, MLX90620_RAM_TWI_ADDRESS, command, 4, data, 2);
	
	*ptat_data = data[1] * 256 + data[0];
	
	return status;
}

SOFTWARE_TWI_STATUS mlx90620_read_compensation_pixel(software_twi_t *twi, int16_t *V_cp) {
	SOFTWARE_TWI_STATUS status;

	uint8_t command[] = {
		MLX90620_READ_RAM,
		MLX90620_RAM_V_CP, // start address
		0x00, // single value no step
		0x01 // single read
	};
	
	uint8_t data[] = {
		0, // lsbyte
		0  // msbyte
	};
	
	status = soft_twi_receive(twi, MLX90620_RAM_TWI_ADDRESS, command, 4, data, 2);
	
	*V_cp = data[1] * 256 + data[0];
	
	return status;
}

SOFTWARE_TWI_STATUS mlx90620_read_ir_array(software_twi_t *twi, int16_t ir_data[4][16]) {
	SOFTWARE_TWI_STATUS status;

	uint8_t command[] = {
		MLX90620_READ_RAM,
		MLX90620_RAM_IR_FIRST, // start address
		0x01, // single step
		0x40 // 64 pixel
	};

	uint8_t data[128]; // lsbyte - msbyte
		
	status = soft_twi_receive(twi, MLX90620_RAM_TWI_ADDRESS, command, 4, data, 128);

	int j;
	int i;

	for (j = 0; j < 16; ++j) {
		for (i = 0; i < 4; ++i) {
			unsigned int offset = (i + j * 4) * 2; // x2 because data holds lsbyte+msbyte

			ir_data[i][j] = mlx90620_twos_complement_word(data[offset + 1], data[offset]);
		}
	}

	return status;
}

int16_t mlx90620_twos_complement_word(uint8_t msbyte, uint8_t lsbyte) {
	// compose word
	uint16_t word = msbyte * 256 + lsbyte;

	int16_t result;

	// sign check
	if (word <= 32767) { // in positive range?
		result = word;
	} else {
		result = word - 65536; // minus 2^16
	}
	
	return result;
}

int8_t mlx90620_twos_complement_byte(uint8_t byte) {
	int8_t result;

	// sign check
	if (byte <= 127) { // in positive range?
		result = byte;
	} else {
		result = byte - 256; // minus 2^8
	}
	
	return result;
}

float mlx90620_calculate_ambient_temperature(mlx90620_eeprom *eeprom, uint16_t ptat_data) {
	// extract the necessary constant, see datasheet, section 7.3
	int16_t V_th = mlx90620_twos_complement_word(eeprom->v_th_high, eeprom->v_th_low);
	int16_t K_t1 = mlx90620_twos_complement_word(eeprom->k_t1_high, eeprom->k_t1_low);
	int16_t K_t2 = mlx90620_twos_complement_word(eeprom->k_t2_high, eeprom->k_t2_low);
	
	float Kt1 = (float) K_t1 / 1024.0;
	float Kt2 = (float) K_t2 / (1024.0 * 1024.0);
	
	// calculate the ambient temperature
	float term_1 = (pow(Kt1, 2) - (4 * Kt2 * (V_th - ptat_data)));
	
	float T_ambient = ((-Kt1 + sqrt(term_1)) / (2 * Kt2)) + 25;
	
	return T_ambient;
}

// TODO: optimize.
float mlx90620_calculate_pixel_temperature(mlx90620_eeprom *eeprom, float T_ambient, uint8_t row, uint8_t column, int16_t ir_data[4][16], int16_t V_cp) {
	// extract constants from the eeprom image, see datasheet, section 7.3
	int8_t A = mlx90620_twos_complement_byte(eeprom->a[row][column]);
	int8_t B = mlx90620_twos_complement_byte(eeprom->b[row][column]);
	uint8_t delta_alpha = mlx90620_twos_complement_byte(eeprom->delta_alpha[row][column]);
	
	int8_t A_cp = mlx90620_twos_complement_byte(eeprom->a_cp);
	int8_t B_cp = mlx90620_twos_complement_byte(eeprom->b_cp);
	uint16_t alpha_cp = eeprom->alpha_cp_high * 256 + eeprom->alpha_cp_low;
	
	int8_t TGC = eeprom->tgc;
	
    uint8_t B_scale = eeprom->b_scale;
	
	uint16_t alpha_zero = eeprom->alpha_0_high * 256 + eeprom->alpha_0_low;
	
	uint8_t alpha_zero_scale = mlx90620_twos_complement_byte(eeprom->alpha_0_scale);
	uint8_t delta_alpha_scale = mlx90620_twos_complement_byte(eeprom->delta_alpha_scale);
	
	float epsilon = (eeprom->epsilon_high * 256 + eeprom->epsilon_low) / 32768.0;
	
	// calculation, see datasheet, section 7.3
 	float V_CP_offset_compensated = (float) V_cp - ((float) A_cp + ((float) B_cp / pow(2, B_scale)) * (T_ambient - 25.0));

 	float V_IR_offset_compensated = (float) ir_data[row][column] - ((float) A + ((float) B / pow(2, B_scale)) * (T_ambient - 25.0));
 	
	float V_IR_TGC_compensated = V_IR_offset_compensated - ((float) TGC / 32.0) * V_CP_offset_compensated;

 	float V_IR_compensated = V_IR_TGC_compensated / epsilon;
	 
	float alpha = ((float) alpha_zero - ((float) TGC / 32.0) * (float) alpha_cp) / pow(2, alpha_zero_scale) + (float) delta_alpha / pow(2, delta_alpha_scale);
 	
 	float T_object = sqrt(sqrt(V_IR_compensated / alpha + pow((T_ambient + 273.15), 4))) - 273.15;
	
	return T_object;
}

// TODO: optimize.
void mlx90620_calculate_whole_frame_pixel_temperature(mlx90620_eeprom *eeprom, float T_ambient, int16_t ir_data[4][16], int16_t V_cp, float temperatures[4][16]) {
	int8_t A_cp = mlx90620_twos_complement_byte(eeprom->a_cp);
	int8_t B_cp = mlx90620_twos_complement_byte(eeprom->b_cp);
	uint16_t alpha_cp = eeprom->alpha_cp_high * 256 + eeprom->alpha_cp_low;
	
	int8_t TGC = eeprom->tgc;
	
	uint8_t B_scale = eeprom->b_scale;
	
	uint16_t alpha_zero = eeprom->alpha_0_high * 256 + eeprom->alpha_0_low;
	
	uint8_t alpha_zero_scale = mlx90620_twos_complement_byte(eeprom->alpha_0_scale);
	uint8_t delta_alpha_scale = mlx90620_twos_complement_byte(eeprom->delta_alpha_scale);
	
	float epsilon = (eeprom->epsilon_high * 256 + eeprom->epsilon_low) / 32768.0;

	// calculation, see datasheet, section 7.3
	float V_CP_offset_compensated = (float) V_cp - ((float) A_cp + ((float) B_cp / pow(2, B_scale)) * (T_ambient - 25.0));

	int j;
	int i;

	for (j = 0; j < 16; ++j) {
		for (i = 0; i < 4; ++i) {
			int8_t A = mlx90620_twos_complement_byte(eeprom->a[i][j]);
			int8_t B = mlx90620_twos_complement_byte(eeprom->b[i][j]);
			uint8_t delta_alpha = mlx90620_twos_complement_byte(eeprom->delta_alpha[i][j]);

			float V_IR_offset_compensated = (float) ir_data[i][j] - ((float) A + ((float) B / pow(2, B_scale)) * (T_ambient - 25.0));
			
			float V_IR_TGC_compensated = V_IR_offset_compensated - ((float) TGC / 32.0) * V_CP_offset_compensated;

			float V_IR_compensated = V_IR_TGC_compensated / epsilon;

			float alpha = ((float) alpha_zero - ((float) TGC / 32.0) * (float) alpha_cp) / pow(2, alpha_zero_scale) + (float) delta_alpha / pow(2, delta_alpha_scale);
			
			temperatures[i][j] = sqrt(sqrt(V_IR_compensated / alpha + pow((T_ambient + 273.15), 4))) - 273.15;
		}
	}
	
	return;
}

bool mlx90620_find_hot_spot(float temperatures[4][16], float T_Ambient, float threshold, float *hot_temp, float *deg_x, float *deg_y) {
	bool found = false;
	int x = 0;
	int y = 0;
	
	// linear search
	float max_temp = -999.9;

	int j;
	int i;

	for (j = 0; j < 16; ++j) {
		for (i = 0; i < 4; ++i) {
			if (temperatures[i][j] - T_Ambient > threshold && temperatures[i][j] > max_temp) {
				found = true;
				max_temp = temperatures[i][j];
				*hot_temp = temperatures[i][j];
				x = j;
				y = i;
			}
		}
	}

	// calculate the angular direction to the hot spot relative to the center/middle of the sensor.
	if (found) {
		*deg_x = MLX90620_FOV_PIXEL_X * x - 7.5 * MLX90620_FOV_PIXEL_X; // 16 pixels from 0 to 15, so -15/2 = -7.5
		*deg_y = MLX90620_FOV_PIXEL_Y * y - 1.5 * MLX90620_FOV_PIXEL_Y; // 4 pixels from 0 to 3, so -3/2 = -1.5
	}
	
	return found;
}
