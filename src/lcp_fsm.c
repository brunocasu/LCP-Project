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
 * @file lcp_fsm.c
 * 
 * @author Bruno Agusto Casu
 *
 * @brief Finite State Machine for the LCP Flight Software: implements the receiver routine and replies for commands form the Base Stations
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
#include "myRL78.h"
// LCP development includes
#include "lcp_radio_driver.h"
#include "lcp_fsm.h"
#include "lcp_ax25.h"


lcp_communication_status_t com_status;
int timeout_flag=0;

/**
 * @brief Finite State Machine for the Flight Software
 * 
 * The FSM keeps the radio waiting for a Command from the Base Station
 * If a message is not received in 10 seconds, a timeout flag is raised and
 * the FSM is reseted
 * 
 * @return the function returns when a timout occours, reseting the FSM
 */
int lcp_state_machine (void)
{
    // Flight Software main Loop
    for (;;)
    {
        int receiver_exit_status;
        ax25_ret_code_t unpack_result;
        uint8_t source_address[6]={0};
        uint8_t ax25_frame_type=0;
        uint8_t received_ns_value=0;
        uint8_t received_nr_value=0;
        uint8_t info[44]={0};
        //uint8_t buff[64];

        // run receiver mode
        receiver_exit_status = lcp_receiver_routine(10);

        // decision if there is a timeout flag or an incomming message
        switch (receiver_exit_status)
        {
            case TIMEOUT:
                led_1_blink ();
                return 0; // relaunch the fsm
                break;

            case MSG_RECEIVED:
                // to bybass the ax25 protocol and retransmmit the received message use the following code:
                //radio_read_fixed_packet (&buff[0], 64);
                //system_delay_ms(10);
                //radio_send_packet (&buff[0], 64);

                // aplly the custom ax25 protocol, return the message result
                unpack_result = unpack_ax25_protocol (&source_address[0], &ax25_frame_type, &received_ns_value, &received_nr_value, &info[0]);
                // provide a reply (ax25_message_manager)
                ax25_message_manager (unpack_result, &source_address[0], ax25_frame_type, received_ns_value, received_nr_value, &info[0]);
                break;

            default:
                return 0;
                break;
        }
    // end of FS main loop
    }
}

/**
 * @brief Receiver routine function: holds the processor waiting for an incomming message
 * 
 * @param time value in seconds for the timeout flag to be raised
 * 
 * @return TIMEOUT if no message arrives in time, or MSG_RECEIVED if the radio raises the PacketReady flag
 */
int lcp_receiver_routine (int time)
{
    radio_rx_mode();
    com_status = RX_MODE_WAITING_PKT; // set global communication state
    timeout_flag = 0; // clear timeout flag
    R_INTC1_Start(); // PORT 50 interrupt in rising edge (connected to DIO0 of the sx1276)
    R_TAU0_Channel0_Start(); // start timer

    // Wait for incoming message or timeout
    while ( (com_status != RX_MODE_PKT_RECEIVED) && (timeout_flag < time) )
        __no_operation();

    R_INTC1_Stop();
    R_TAU0_Channel0_Stop(); // stop the timer

    if (com_status == RX_MODE_PKT_RECEIVED)
        return MSG_RECEIVED;
    else
        return TIMEOUT;

}


/**
 * @brief blink the debug LED once (Means receiver TIMEOUT)
 */
void led_1_blink (void)
{
    LED_PORT=1;
    system_delay_ms(40);
    LED_PORT=0;
}

/**
 * @brief blink the debug LED once (Means message received and REPLY SENT)
 */
void led_2_blink (void)
{
    LED_PORT=1;
    system_delay_ms(50);
    LED_PORT=0;
    system_delay_ms(100);
    LED_PORT=1;
    system_delay_ms(50);
    LED_PORT=0;
}


/**
 * @brief blink the debug LED once (Means message received with error NO REPLY SENT)
 */
void led_3_blink (void)
{
    LED_PORT=1;
    system_delay_ms(70);
    LED_PORT=0;
    system_delay_ms(100);
    LED_PORT=1;
    system_delay_ms(70);
    LED_PORT=0;
    system_delay_ms(100);
    LED_PORT=1;
    system_delay_ms(70);
    LED_PORT=0;
}
