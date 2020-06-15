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
// LCP development includes
#include "lcp_radio_driver.h"
#include "lcp_fsm.h"
#include "lcp_ax25.h"

void base_satation_supervision (void);

lcp_communication_status_t com_status;
int timeout_flag=0;
uint8_t global_timeout_value = 10;

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
    int receiver_exit_status;
    ax25_ret_code_t unpack_result;
    uint8_t source_address[6]={0};
    uint8_t ax25_frame_type=0;
    uint8_t received_ns_value=0;
    uint8_t received_nr_value=0;
    uint8_t info[44]={0};
    uint8_t aux_addr[6] = {0};

    led_3_blink ();
    
    base_satation_supervision ();
    if (BATTERY_FAUL_PORT != 0)
        PWR_ON_LED_PORT = 1;
    else
        FAULT_LED_PORT = 1;
        
    for (;;)
    {
        // run receiver mode
        receiver_exit_status = lcp_receiver_routine(global_timeout_value);

        // decision if there is a timeout flag or an incomming message
        switch (receiver_exit_status)
        {
            case TIMEOUT:
                
                if (BATTERY_FAUL_PORT == 0)
                {
                    FAULT_LED_PORT = 1;
                    aux_addr[0] = base_satation_1[0];
                    aux_addr[1] = base_satation_1[1];
                    aux_addr[2] = base_satation_1[2];
                    aux_addr[3] = base_satation_1[3];
                    aux_addr[4] = base_satation_1[4];
                    aux_addr[5] = base_satation_1[5];
                    send_s_frame_command (&aux_addr[0], 0x00, 0); // RNR
                }
                else
                    base_satation_supervision ();
                
                led_1_blink ();
                break;

            case MSG_RECEIVED:
                // to bybass the ax25 protocol and retransmmit the received message use the following code:
                //radio_read_fixed_packet (&buff[0], 64);
                //system_delay_ms(10);
                //radio_send_packet (&buff[0], 64);

                // apply the custom ax25 protocol, return the message result
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

void base_satation_supervision (void)
{
    uint8_t s_target_addr[6];
    static uint8_t seq_num=0;

    for (int k=0; k<=5; k++)
        s_target_addr[k]=base_satation_1[k];

    seq_num++;
    send_s_frame_command (&s_target_addr[0], seq_num, 1);

    system_delay_ms(100);
    for (int k=0; k<=5; k++)
        s_target_addr[k]=base_satation_2[k];

    send_s_frame_command (&s_target_addr[0], seq_num, 1);
}
/**
 * @brief blink the debug LED once (Means receiver TIMEOUT)
 */
void led_1_blink (void)
{
    LED_PORT=1;
    system_delay_ms(50);
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
