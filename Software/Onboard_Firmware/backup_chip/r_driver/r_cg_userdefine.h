/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIESREGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2017 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_userdefine.h
* Version      : Applilet3 for RL78/G13 V2.04.00.03 [05 May 2017]
* Device(s)    : R5F100LE
* Tool-Chain   : IAR Systems iccrl78
* Description  : This file includes user definition.
* Creation Date: 16/02/2020
***********************************************************************************************************************/

#ifndef _USER_DEF_H
#define _USER_DEF_H

//#include "../lcp_radio_driver.h"
#include "../lcp_fsm.h"
//#include "../lcp_ax25.h"

#define CSS_RADIO_PORT          P7_bit.no0 // out
#define RESET_RADIO_PORT        P7_bit.no1 // out
#define CSS_BME_280_PORT        P7_bit.no2 // out
#define CSS_GPS_PORT            P7_bit.no3 // out
#define BATTERY_FAUL_PORT       P7_bit.no4 // in
#define LED_PORT                P7_bit.no5 // out
#define PWR_ON_LED_PORT         P7_bit.no6 // out
#define FAULT_LED_PORT          P7_bit.no7 // out

#define DIO_0_PORT              P5_bit.no0 // in - interrupt driven
#define DIO_2_PORT              P5_bit.no1 // in - interrupt driven
// #define DIO_2_PORT           P5_bit.no2  
// #define LED_PORT             P5_bit.no4

#define MCU_ALIVE_PORT          P3_bit.no0 // main - out // backup - in

#define SECONDARY_MCU_RESET     P0_bit.no5 // out
#define BATTERY_SWITCH_PORT     P0_bit.no6 // out

 /*
  *     SPI_CLK     P10
  *     SPI_MISO    P11
  *     SPI_MOSI    P12
  * 
  * 
  */
extern int spi_completion_flag;

extern lcp_communication_status_t com_status;
extern int timeout_flag;


#endif
