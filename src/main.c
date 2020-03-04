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
 * @file main.c
 * 
 * @author Bruno Agusto Casu
 *
 * @brief main function and hardware configuration
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
#include "MyRL78.h"
// LCP development includes
#include "lcp_radio_driver.h"
#include "lcp_fsm.h"
#include "lcp_ax25.h"
#include "lcp_ax25.h"
/* Set option bytes */
#pragma location = "OPTBYTE"
__root const uint8_t opbyte0 = 0xEFU;
#pragma location = "OPTBYTE"
__root const uint8_t opbyte1 = 0xFFU;
#pragma location = "OPTBYTE"
__root const uint8_t opbyte2 = 0xE8U;
#pragma location = "OPTBYTE"
__root const uint8_t opbyte3 = 0x84U;
/* Set security ID */
#pragma location = "SECUID"
__root const uint8_t secuid[10] =
    {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    
    
int system_config (void);
void R_Systeminit(void);
uint8_t count = 5;

void main (void) // LCP MAIN SOFTWARE
{
    system_config ();

    for(;;)
    {
        lcp_state_machine ();
            
    }
    //lcp_state_machine ();
}

/**
 * @brief System configuration for th GPIOs, SPI bus, Interruption and timers and the connected peripheral devices (sx1276 LoRa radio, etc..)
 */
int system_config (void)
{
    R_Systeminit(); // System setup and SPI00 configuration
    
    // SPI BUS
    // PINS: CLK(P10) / MISO(11)input / MOSI(12)output 
    // DATA RATE: 1MHz
    // POLARITY: CLK starts in low, DATA is sampled in the DOWN edge of CLK
    // MSB First

    EI(); // Enable Interruptions
    R_INTC1_Start(); // INTC1 enable - PORT50 Rising Edge
    R_CSI00_Start(); // init SPI channel (00)

    // CHIP SELECT FOR RADIO DEVICE
    PM7_bit.no0 = 0; // PIN as OUTPUT
    PU7_bit.no0 = 1; // Pull-Up on
    CSS_RADIO_PORT = 1; // Start with High value
    
    // RADIO RESET
    PM7_bit.no1 = 0; // PIN as OUTPUT
    PU7_bit.no1 = 1; // Pull-Up on
    RESET_RADIO_PORT = 1; // Start with High value
    
    // RED LED
    PM5_bit.no4 = 0; // PIN as OUTPUT
    PU5_bit.no4 = 1; // Pull-Up on
    LED_PORT = 0; // Start with Low value
    
    // DIO 0 - in TX: PacketSent Flag - in RX: PayloadReady Flag
    PM5_bit.no0 = 1; // PIN50 set the INTP1 in Rising Edge
    PU5_bit.no0 = 0; // Pull-Up off
    // DIO_0_PORT
    
    // INTERRUPT PORT 2 (can be used to check the battery status)
    PM5_bit.no1 = 1; // PIN51 set the INTP2 in Rising Edge
    PU5_bit.no1 = 0; // Pull-Up off
    
    // DIO 2 - in : PacketSent Flag
    PM5_bit.no2 = 1; // PIN52 input for DIO2 signal: Rx ready (for eficeiency in mode configuration)
    PU5_bit.no2 = 0; // Pull-Up off
    // DIO_2_PORT
    
    LED_PORT = 1;
    // Radio Register configuration for FSK packet mode
    // Data rate is 4.8kbps
    // Pkt size is 64 bytes
    radio_version_check(); // check if version matches (v12)
    radio_register_config (); // run configuration of sx1276 registers
    LED_PORT = 0;
    
    return (1);
}
/*
        uint8_t buff[64];
        radio_rx_mode();
        com_status = RX_MODE_WAITING_PKT; // set global communication state
        timeout_flag = 0; // clear flag
        R_INTC1_Start(); // PORT 50 interrup in rising edge
        R_TAU0_Channel0_Start(); // start timer
        
        // Wait for incoming message or timeout
        while ( (com_status != RX_MODE_PKT_RECEIVED) && (timeout_flag < 15) )
            __no_operation();

        R_INTC1_Stop();
        R_TAU0_Channel0_Stop(); // stop the timer
        
        if (com_status == RX_MODE_PKT_RECEIVED)
        {
            radio_read_fixed_packet (&buff[0], 64);
            system_delay_ms(10);
            buff[2]=count++;
            radio_send_packet (&buff[0], 64);
        }
        else
        {
            led_1_blink ();
        }
 */
