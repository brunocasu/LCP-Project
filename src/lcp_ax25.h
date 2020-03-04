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
 * @file lcp_ax25.h
 * 
 * @author Bruno Agusto Casu
 *
 * @brief Driver for implementing the communication using the AX.25 Link Access (Version 2.2)
 */


#ifndef LCP_AX25_H
#define LCP_AX25_H
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

// AX25 Control frame type
#define COMMAND                     0x01
#define RESPONSE                    0x02

#define I_FRAME_BIT                 0xfe // 1111 1110
#define S_FRAME_BIT                 0x01 // 0000 0001

extern uint8_t const lcp_address[6]; // 7bit ASCII code
extern uint8_t const base_satation_1[6];
extern uint8_t const base_satation_2[6];

/**
 * Return code for the AX25 unpacking
 */
typedef enum {  INITIAL_FLAG_ERROR,
                END_FLAG_ERROR,
                HEADER_CHECKSUM_ERROR,
                ADDRESS_ERROR,
                DESTINY_ADDR_SSID_ERROR,
                SOURCE_ADDR_SSID_ERROR,
                SSID_PARING_ERROR,
                LCP_NO_LAYER3_ERROR,
                INFO_CHECKSUM_ERROR,
                SUPERVISORY_FRAME,
                INFORMATION_FRAME}
ax25_ret_code_t;

// AX25 protocol adapted frame
typedef struct {uint8_t destiny_address[6];
                uint8_t destiny_ssid;
                uint8_t source_address[6];
                uint8_t source_ssid;
                uint8_t control;
                uint8_t protocol_identification; // Layer 3 protocol (not used in the LCP info field - code is 0xf0)
                uint8_t info[44];} // fixed in 44 bytes for the LCP
lcp_ax25_protocol_t; // the ax25 protocol implemented in the LCP has a fixed frame size of 64 bytes

ax25_ret_code_t unpack_ax25_protocol (  uint8_t *source_address,
                                        uint8_t *ax25_frame_type,
                                        uint8_t *received_ns_value,
                                        uint8_t *received_nr_value,
                                        uint8_t *info);

int ax25_message_manager ( ax25_ret_code_t msg_ret_code,
                           uint8_t         *source_address,
                           uint8_t         ax25_frame_type,
                           uint8_t         received_ns_value,
                           uint8_t         received_nr_value,
                           uint8_t         *info);

// void send_s_frame_command (uint8_t *destiny_address); TODO
// void send_s_frame_reply (uint8_t *destiny_address); TODO

void send_i_frame_command (uint8_t *destiny_address, uint8_t ns_value, uint8_t nr_value, uint8_t *info); 
void send_i_frame_reply (uint8_t *destiny_address, uint8_t ns_value, uint8_t nr_value, uint8_t *info);

void ax25_to_binary (lcp_ax25_protocol_t frame_to_send);

#endif
