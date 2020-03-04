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
 * @file lcp_radio_driver.h
 * 
 * @author Bruno Agusto Casu
 *
 * @brief Driver for the LCP radio device
 */


#ifndef LCP_RADIO_DRIVER_H
#define LCP_RADIO_DRIVER_H
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

#define PACKET_SENT     1
#define PAYLOAD_READY   1
#define RX_READY        1

uint8_t sx1276_write_register (uint8_t addr, uint8_t data);
uint8_t sx1276_read_register (uint8_t addr);

void sx1276_write_fifo (uint8_t data);
uint8_t sx1276_read_fifo (void);

void radio_tx_mode (void);
void radio_rx_mode (void);
void radio_sleep_mode (void);
void radio_register_config (void);

void system_delay_ms(unsigned char time);

int radio_version_check (void);

void radio_cold_reset (void);

void radio_reset_fifo (void);

void radio_send_packet (uint8_t* pkt, uint8_t pkt_length);
void radio_read_fixed_packet (uint8_t* pkt, uint8_t fixed_length);


#endif
