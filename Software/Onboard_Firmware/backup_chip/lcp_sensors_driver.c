/****************************************************************************/
/*                                                                          */
/* This Source Code Form is subject to the terms of the Mozilla Public      */
/* License, v. 2.0. If a copy of the MPL was not distributed with this      */
/* file, You can obtain one at http://mozilla.org/MPL/2.0/.                 */
/*                                                                          */
/****************************************************************************/

/*
 * This File is part of the LCP project
 */

/**
 * @file lcp_sensors_driver.c
 *
 * @author Bruno Agusto Casu
 *
 * @revisor Bruno Duarte
 *
 * @brief Driver for the attached sensros in the LCP
 */


#include "string.h"
#include "stdio.h"
#include "stdlib.h"
// applilet includes
#include "r_driver/r_cg_macrodriver.h"
//#include "r_driver/r_cg_adc.c"
#include "r_driver/r_cg_cgc.h"
#include "r_driver/r_cg_intc.h"
#include "r_driver/r_cg_port.h"
#include "r_driver/r_cg_serial.h"
#include "r_driver/r_cg_timer.h"
#include "r_driver/r_cg_userdefine.h"
// RL78G13 includes
#include "ior5f100lg.h"
#include "ior5f100lg_ext.h"
#include "intrinsics.h"
// LCP development includes
#include "lcp_radio_driver.h"
#include "lcp_fsm.h"
#include "lcp_ax25.h"
#include "lcp_sensors_driver.h"

uint8_t bmp280_read_register (uint8_t addr);
uint8_t bmp280_write_register (uint8_t addr, uint8_t data);

/**
 * @brief Function to operate all the ADC channels in the microcontroller
 *
 * @param data return the readings of the ADCs in for of a byte array
 */
void analog_sensors_read (uint8_t *data)
{
    uint16_t analog_value; //revised: (included)
    uint16_t * const channel_reading = &analog_value; //revised: (included)
    int data_position = 0;

    R_ADC_Set_OperationOn();

    ADS = _10_AD_INPUT_CHANNEL_16;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    data_position += 2;
    R_ADC_Stop();

    ADS = _11_AD_INPUT_CHANNEL_17;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    data_position += 2;
    R_ADC_Stop();

    ADS = _02_AD_INPUT_CHANNEL_2;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    data_position += 2;
    R_ADC_Stop();

    ADS = _03_AD_INPUT_CHANNEL_3;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    data_position += 2;
    R_ADC_Stop();

    ADS = _04_AD_INPUT_CHANNEL_4;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    data_position += 2;
    R_ADC_Stop();

    ADS = _05_AD_INPUT_CHANNEL_5;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    data_position += 2;
    R_ADC_Stop();

    ADS = _06_AD_INPUT_CHANNEL_6;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    data_position += 2;
    R_ADC_Stop();

    ADS = _07_AD_INPUT_CHANNEL_7;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    data_position += 2;
    R_ADC_Stop();

    ADS = _12_AD_INPUT_CHANNEL_18;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    data_position += 2;
    R_ADC_Stop();

    ADS = _13_AD_INPUT_CHANNEL_19;
    R_ADC_Start();
    system_delay_ms (1);
    R_ADC_Get_Result(channel_reading);

    data[data_position] = (uint8_t)(analog_value>>8); //revised:
    data[data_position+1] = (uint8_t)(analog_value&0xFF); //revised:
    R_ADC_Stop();

    R_ADC_Set_OperationOff();
    // SEQUENCE OF DATA:
    // B19  B18  B17  B16  B15  B14  B13  B12  B11  B10  B9   B8   B7   B6   B5   B4   B3   B2   B1   B0
    // MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB
    // AMP  AMP  VCC  VCC  LDR5 LDR5 LDR4 LDR4 LDR3 LDR3 LDR2 LDR2 LDR1 LDR1 LDR0 LDR0 NTC1 NTC1 NTC0 NTC0
}


/**
 * @brief Function to read the temperature a pressure raw measurements from the BMP280 digital sensor using a custom SPI bus
 *
 * @param data return the readings of the 6 registers in the sensor (8 byte reisters)
 *
 * @note Because the BMP280 operate with a different SPI clock polarity of the SX1276 Radio, before the readings,
 *  the SPI driver must be reconfigured (clock mode 4 in Aplillet3), and after the reading return to the previous
 *  operation mode (clock mode 2 in Aplillet3).
 */
void bmp280_read (uint8_t *data)
{
    int i;

    R_CSI00_Stop();
    // void R_CSI00_Create(void)
    // Recreate the SPI channel with inverted clock polarity to access the BMP280 registers
    ST0 |= _0001_SAU_CH0_STOP_TRG_ON;    /* disable CSI00 */
    CSIMK00 = 1U;    /* disable INTCSI00 interrupt */
    CSIIF00 = 0U;    /* clear INTCSI00 interrupt flag */
    /* Set INTCSI00 high priority */
    CSIPR100 = 0U;
    CSIPR000 = 0U;
    SIR00 = _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;    /* clear error flag */
    SMR00 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS |
            _0000_SAU_TRIGGER_SOFTWARE | _0000_SAU_MODE_CSI | _0000_SAU_TRANSFER_END;
    SCR00 = _C000_SAU_RECEPTION_TRANSMISSION | _3000_SAU_TIMING_4 | _0000_SAU_MSB | _0007_SAU_LENGTH_8;
    SDR00 = _1E00_CSI00_DIVISOR;            /* _1000_SAU_TIMING_2 for the SX1276*/
    SO0 &= ~_0100_SAU_CH0_CLOCK_OUTPUT_1;    /* CSI00 clock initial level */
    SO0 &= ~_0001_SAU_CH0_DATA_OUTPUT_1;    /* CSI00 SO initial level */
    SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;    /* enable CSI00 output */
    /* Set SI00 pin */
    PM1 |= 0x02U;
    /* Set SO00 pin */
    P1 |= 0x04U;
    PM1 &= 0xFBU;
    /* Set SCK00 pin */
    P1 |= 0x01U;
    PM1 &= 0xFEU;
    system_delay_ms (1);
    R_CSI00_Start();
    system_delay_ms (1);

    uint8_t chip_id = bmp280_read_register (0xd0);

    if (chip_id == 0x58)
    {
        bmp280_write_register (0xf4, 0x49);
        // this config allows for 17 bit readings for temp and pressure: Low Power mode
        data[0] = bmp280_read_register (0xfa); // MSB of temperature
        data[1] = bmp280_read_register (0xfb); // LSB of temperature
        data[2] = bmp280_read_register (0xfc); // XLSB of temperature [7:4]

        data[3] = bmp280_read_register (0xf7); // MSB of pressure
        data[4] = bmp280_read_register (0xf8); // LSB of pressure
        data[5] = bmp280_read_register (0xf9); // XLSB of pressure [7:4]
    }
    else // if chip id is incorrect return error values for the measurements
    {
        for (i=0; i<=5; i++)
            data[i] = 0xee;
    }

    // Recreate and restart the SPI for the Radio
    R_CSI00_Stop();
    R_CSI00_Create();
    R_CSI00_Start();
}

/**
 * @brief Function to read the BMP280 pressure sensor via customized SPI bus
 *
 * @param addr 7 bit register address
 *
 * @return current value in the target register
 */
uint8_t bmp280_read_register (uint8_t addr)
{
    uint8_t tx_buff_ptr[2];
    uint8_t rx_buff_ptr[2];
    tx_buff_ptr[0] = addr;
    tx_buff_ptr[1] = 0;

    CSS_BME_280_PORT = 0; // chip select down
    R_CSI00_Send_Receive(&tx_buff_ptr[0], 2, &rx_buff_ptr[0]); // SPI freq. is 1MHz - set down spi_completion_flag
    while (spi_completion_flag == 0)
        __no_operation(); // wait for transaction to be completed

    return rx_buff_ptr[1];
}

/**
 * @brief Function to write the BMP280 pressure sensor via customized SPI bus
 *
 * @param addr 7 bit register address
 * @param data value to be written
 *
 * @return current value in the target register
 */
uint8_t bmp280_write_register (uint8_t addr, uint8_t data)
{
    uint8_t tx_buff_ptr[2];
    uint8_t rx_buff_ptr[2];
    tx_buff_ptr[0] = addr & 0x7f;
    tx_buff_ptr[1] = data;

    CSS_BME_280_PORT = 0; // chip select down
    R_CSI00_Send_Receive(&tx_buff_ptr[0], 2, &rx_buff_ptr[0]); // SPI freq. is 1MHz - set down spi_completion_flag
    while (spi_completion_flag == 0)
        __no_operation(); // wait for transaction to be completed

    return rx_buff_ptr[1];
}
