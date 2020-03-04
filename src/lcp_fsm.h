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
 * @file lcp_fsm.h
 * 
 * @author Bruno Agusto Casu
 *
 * @brief Finite State Machine for the LCP Flight Software: implements the receiver routine and replies for commands form the Base Stations
 */


#ifndef LCP_FSM_H
#define LCP_FSM_H
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

/**
 * @{
 * @brief macros used in the AX25 payload: the byte zero of the info filed is used 
 *        to identify what kind of message the payload is carring
 */
#define READ_SENSORS_HEX                0xa1
#define SENSORS_CAPABILITIES_INFO_HEX   0xa2
#define STRING_MESSAGE_HEX              0xa3
#define GET_TIME_INFO_HEX               0xa4
#define LCP_NO_LAYER3_ERROR_HEX         0xe1
#define INFO_CHECKSUM_ERROR_HEX         0xe2
#define SEQUENCE_NUMBER_ERROR_HEX       0xe3
///@}

#define TIMEOUT                     1
#define MSG_RECEIVED                2

/**
 * global status satellite communication routine
 */
typedef enum {  RX_MODE_WAITING_PKT,
                RX_MODE_PKT_RECEIVED, 
                TX_MODE_TRANSMITING,
                TX_MODE_PKT_SEND}
lcp_communication_status_t;

extern lcp_communication_status_t com_status;

int lcp_state_machine (void);

int lcp_receiver_routine (int time);

void led_1_blink (void);
void led_2_blink (void);
void led_3_blink (void);

#endif
