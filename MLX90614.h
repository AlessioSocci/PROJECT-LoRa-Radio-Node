/*
 * MLX90614.h
 *
 *  Created on: 19 mar 2019
 *      Author: Alessio
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"

#include "time.h"

#define MLX90614_READADDRESS							0xB5 // 0xB5 : (5A << 1) + 1
#define MLX90614_WRITEADDRESS							0xB4

typedef enum
{
	MLX90614_REGISTER_TAMBIENT						= 0x03,
	MLX90614_REGISTER_TOBJECT1						= 0x04,
	MLX90614_REGISTER_LINEARIZED_TAMBIENT			= 0x06,
	MLX90614_REGISTER_LINEARIZED_TOBJECT1			= 0x07,

	MLX90614A_EEPROM_Read_REGISTER_T0_max 			= 0x00,
	MLX90614A_EEPROM_Read_REGISTER_T0_min 			= 0x01,
	MLX90614A_EEPROM_Read_REGISTER_PWM_control 		= 0x02,
	MLX90614A_EEPROM_Read_REGISTER_TA_range 		= 0x03,
	MLX90614A_EEPROM_Read_REGISTER_Emiss_Corr 		= 0x04,
	MLX90614A_EEPROM_Read_REGISTER_Config_Reg1 		= 0x05,
	MLX90614A_EEPROM_Read_REGISTER_SMBus_Address	= 0x0E, 		// (LSByte only)
	MLX90614A_EEPROM_Read_REGISTER_ID_Number1 		= 0x1C,
	MLX90614A_EEPROM_Read_REGISTER_ID_Number2 		= 0x1D,
	MLX90614A_EEPROM_Read_REGISTER_ID_Number3 		= 0x1E,
	MLX90614A_EEPROM_Read_REGISTER_ID_Number4 		= 0x1F,
} MLX90614_RAM_Or_EEPROM_Read_Address;

typedef enum
{
	 MLX90614A_EEPROM_Write_REGISTER_T0_max 		= 0x00,
	 MLX90614A_EEPROM_Write_REGISTER_T0_min 		= 0x01,
	 MLX90614A_EEPROM_Write_REGISTER_PWM_control 	= 0x02,
	 MLX90614A_EEPROM_Write_REGISTER_TA_range 		= 0x03,
	 MLX90614A_EEPROM_Write_REGISTER_Emiss_Corr 	= 0x04,
	 MLX90614A_EEPROM_Write_REGISTER_Config_Reg1 	= 0x05,
	 MLX90614A_EEPROM_Write_REGISTER_SMBus_Address 	= 0x0E, 		// (LSByte only)
} MLX90614A_EEPROM_Write_Address;

typedef enum _MLX90614_Scale
{
    MLX90614_SCALE_KELVIN,
    MLX90614_SCALE_CELSIUS,
    MLX90614_SCALE_FARENHEIT,
} MLX90614_Scale;

extern uint16_t value;

void MLX90614_init(uint32_t inputBaudRate);
float MLX90614_getTemp(MLX90614_Scale scale, MLX90614_RAM_Or_EEPROM_Read_Address reg);
uint16_t I2C_readRegister(MLX90614_RAM_Or_EEPROM_Read_Address reg);
