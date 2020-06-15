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
 * @file lcp_radio_driver.c
 * 
 * @author Bruno Agusto Casu
 *
 * @brief Driver for the LCP radio device
 */


#include "string.h"
#include "stdio.h"
#include "stdlib.h"
// applilet includes
#include "r_driver/r_cg_macrodriver.h"
#include "r_driver/r_cg_adc.h"
#include "r_driver/r_cg_cgc.h"
#include "r_driver/r_cg_intc.h"
#include "r_driver/r_cg_port.h"
#include "r_driver/r_cg_serial.h"
#include "r_driver/r_cg_timer.h"
#include "r_driver/r_cg_userdefine.h"
// RL78G13 includes
#include "ior5f100le.h"
#include "ior5f100le_ext.h"
#include "intrinsics.h"
// LCP development includes
#include "lcp_radio_driver.h"

/**
 * @brief Function to write the sx1276 register via SPI bus
 * 
 * @param addr register address (7 bit)
 * @param data byte to be written in the register
 * 
 * @return current value in the target register
 */
uint8_t sx1276_write_register (uint8_t addr, uint8_t data)
{
    uint8_t tx_buff_ptr[2];
    uint8_t rx_buff_ptr[2];
    tx_buff_ptr[0] = (0x80) | addr;
    tx_buff_ptr[1] = data;

    CSS_RADIO_PORT = 0; // chip select down
    R_CSI00_Send_Receive(&tx_buff_ptr[0], 2, &rx_buff_ptr[0]); // SPI freq. is 1MHz - set down spi_completion_flag
    while (spi_completion_flag == 0)
        __no_operation(); // wait for transaction to be completed

    return rx_buff_ptr[1]; // return register current value
}

/**
 * @brief Function to read the radio (sx1276) register via SPI bus
 * 
 * @param addr 7 bit register address
 * 
 * @return current value in the target register
 */
uint8_t sx1276_read_register (uint8_t addr)
{
    uint8_t tx_buff_ptr[2];
    uint8_t rx_buff_ptr[2];
    tx_buff_ptr[0] = (0x7f) & addr;
    tx_buff_ptr[1] = 0;

    CSS_RADIO_PORT = 0; // chip select down
    R_CSI00_Send_Receive(&tx_buff_ptr[0], 2, &rx_buff_ptr[0]); // SPI freq. is 1MHz - set down spi_completion_flag
    while (spi_completion_flag == 0)
        __no_operation(); // wait for transaction to be completed

    return rx_buff_ptr[1];
}

/**
 * @brief Function to write one byte of data into the Radio FIFO
 * 
 * @param data byte to be written in the FIFO
 */
void sx1276_write_fifo (uint8_t data)
{
    sx1276_write_register(0, data);
}

/**
 * @brief Function to read one byte of data from the FIFO 
 * 
 * @return read byte from FIFO
 */
uint8_t sx1276_read_fifo (void)
{
    uint8_t data = sx1276_read_register(0);
        return data;
}

/**
 * @brief Switch the radio mode to Receiver
 */
void radio_tx_mode (void)
{
    sx1276_write_register (0x01, 0x00);
    sx1276_write_register (0x01, 0x03);
    system_delay_ms (5); // time to config the mode
}

/**
 * @brief Switch the radio mode to Transmitter
 */
void radio_rx_mode (void)
{
    sx1276_write_register (0x01, 0x00);
    sx1276_write_register (0x01, 0x05);
    int n = 0;
    while (DIO_2_PORT == 0 && n < 1000000)
    {
        __no_operation(); // hold until config is done
        n++;
    }
}

/**
 * @brief Switch the radio mode to Sleep mode
 */
void radio_sleep_mode (void)
{
    sx1276_write_register (0x01, 0x00);
    system_delay_ms (1);
}

 /**
  * @brief Configure all the radio register with the choosen operation parameters
  * \par
  * Data Transmission Mode: FSK \par
  * Carrier Frequency: 915MHz \par
  * Power output: 20dBm (100mW) Maximum output power \par
  * Preamble config: 1010 1010 (0xAA default) size of 16 bytes \par
  * Synchronization Word: enabled with size of 8 bytes \par
  * Packet configuration: Fixed Packet Length of 64bytes (payload)
  */
void radio_register_config (void)
{
    sx1276_write_register (0x01, 0x01); // set to Standby mode (for IQ and RSSI calibration Radio must be in Standby mode)
    sx1276_write_register (0x09, 0x00); // power output to zero

    // 915MHz carrier freq. config
    sx1276_write_register (0x06, 0xe4); // MSB of carrier freq. where f=R(23:0)*61,035
    sx1276_write_register (0x07, 0xc0);
    sx1276_write_register (0x08, 0x26); // LSB of carrier freq.

    // calibration process (IQ and RSSI calibration) for 915MHz radio frquency
    sx1276_write_register (0x3b, 0xc2);
    system_delay_ms (1);

    sx1276_write_register (0x09, 0xff); // power output to maximum value
    sx1276_write_register (0x01, 0x00); // put radio in sleep mode

    // set preamble size to 16 symbols
    sx1276_write_register (0x25, 0x00); // Preamble size MSB
    sx1276_write_register (0x26, 0x10); // preamble size LSB - must be the same for Tx and Rx!

    // Sync Configuration
    sx1276_write_register (0x27, 0x97); // Sync Config: set 8 bytes(max) to the Sync Word
    sx1276_write_register (0x28, 0x20); // Following byte are for the generation and confimation of the Sync Word (must be the same for Tx and Rx)
    sx1276_write_register (0x29, 0x20);
    sx1276_write_register (0x2a, 0x02);
    sx1276_write_register (0x2b, 0x27);
    sx1276_write_register (0x2c, 0x54);
    sx1276_write_register (0x2d, 0x52);
    sx1276_write_register (0x2e, 0x49);
    sx1276_write_register (0x2f, 0x45);

    // PACKET CONFIGURATION
    sx1276_write_register (0x30, 0x10); // fixed packet length / no encaoding / CRC on / Clear FIFO when CRC check fail / No address filtering
    sx1276_write_register (0x32, 0x40); // Payload length (LSB) = 64bytes
    sx1276_write_register (0x33, 0x8a); // Node addr for Tx and Rx (not being used)
    sx1276_write_register (0x35, 0x3f); // Interrupt for Start transmitting is FifoLevel with FifoThreshold = 63

    sx1276_write_register (0x40, 0x04); // DIO0 - 00 / DIO1 - 00 / DIO2 - 01
}

/**
 * @brief System delay function
 * 
 * @param time delay time in ms (max of 255ms)
 */
void system_delay_ms(unsigned char time)
{
    unsigned int temp;
    for (;time;time--)
        for (temp=32000/9;temp;temp--)
            __no_operation();
}

/**
 * @brief This function reads the register 0x42 of the sx1276 radio which contains the chip version, the register value must be 0x12
 *        in the case of a version missmatch the program is informed (reseting the devices may be needed)
 * 
 * This function is called in the configuration process of the Flight Software, it may indicates an error in the SPI bus (in the case of an
 * reading error) or may inticate malfunction in the radio device
 * 
 * @note If an error is returned the Flight Software should not continue runing
 * 
 * @return one if the version matches
 */
int radio_version_check (void)
{
    uint8_t const lora_chip_version = sx1276_read_register(0x42);
    if (lora_chip_version == 0x12)
        return 1;
    else 
    {
        for(;;); // lock the processor if radio fault
    }
}

/**
 * @brief Force reset of the radio device (reset pin pulled down)
 * 
 * @param time value in multiple of 10ms of the amount of time the radio is disabled
 * 
 * @note WARNING: if param time has a value of zero the radio is permanentely disabled
 */
void radio_cold_reset (int time)
{
    if (time > 0)
    {
        RESET_RADIO_PORT = 0;
        LED_PORT = 1;
        system_delay_ms (1);
        for (int i=0; i<time; i++)
            system_delay_ms (10);
        
        RESET_RADIO_PORT = 1;
        system_delay_ms (5);
        radio_register_config ();
        LED_PORT = 0;
    }
    else
    {
        RESET_RADIO_PORT = 0;
        LED_PORT = 1;
        for(;;); // shut down the radio device
    }
}

/**
 * @brief Clear the radio FIFO and the flags, enabling the radio for the next transaction
 */
void radio_reset_fifo (void)
{
    sx1276_write_register( 0x3f, sx1276_read_register(0x3f) | 0x10); // reset FIFO
}

/**
 * @brief Function to write in the FIFO the packet to be transmited
 * 
 * @param ptk pointer to the packet array to be sent
 * @param pkt_length total size of the packet length (sx1276 frame payload size)
 */
void radio_send_packet (uint8_t* pkt, uint8_t pkt_length)
{
    radio_tx_mode();

    com_status=TX_MODE_TRANSMITING;
    R_INTC1_Start(); // PORT 50 interrup in rising edge
    for (int i=0; i<pkt_length; i++)
        sx1276_write_fifo(pkt[i]);

    while (com_status==TX_MODE_TRANSMITING) // wait for transmission to be completed
        __no_operation();

    system_delay_ms(1);
    R_INTC1_Stop();
    radio_reset_fifo();
    radio_sleep_mode ();
    system_delay_ms(2);
}

/**
 * @brief Function to read from the FIFO a received packet
 * 
 * @param pkt return the packet read from the FIFO
 * @param fixed_length number of bytes to be read from the FIFO (must be eqaul to the packet size in this case)
 * 
 * @note Use this function only in Fixed Packet size mode of the sx1276 radio
 */
void radio_read_fixed_packet (uint8_t* pkt, uint8_t fixed_length)
{
    if (com_status==RX_MODE_PKT_RECEIVED)
    {
        for(int i=0; i<fixed_length; i++)
            pkt[i]=sx1276_read_fifo();
    }
    radio_reset_fifo();
    radio_sleep_mode ();
    system_delay_ms(2);
}

void radio_config_bit_rate (uint8_t bit_rate_MSB, uint8_t bit_rate_LSB, uint8_t FDA_MSB, uint8_t FDA_LSB)
{
    radio_sleep_mode ();
    sx1276_write_register (0x02, bit_rate_MSB);
    sx1276_write_register (0x03, bit_rate_LSB);
    system_delay_ms(1);
    sx1276_write_register (0x04, FDA_MSB & 0x3f);
    sx1276_write_register (0x05, FDA_LSB);
    system_delay_ms(2);
}
