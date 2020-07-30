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
 * @file lcp_ax25.c
 * 
 * @author Bruno Agusto Casu
 *
 * @revisor Bruno Duarte
 *
 * @brief Driver for implementing the communication using the AX.25 Link Access Protocol (Version 2.2)
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
#include "ior5f100lg.h"
#include "ior5f100lg_ext.h"
#include "intrinsics.h"
// LCP development includes
#include "lcp_radio_driver.h"
#include "lcp_fsm.h"
#include "lcp_ax25.h"
#include "lcp_sensors_driver.h"


// addr : F  E  I  L  C  P
// 7bit : 8c 8a 92 98 86 a0
// ASCII: 46 45 49 4c 43 50
uint8_t const lcp_address[6] = {0x8c, 0x8a, 0x92, 0x98, 0x86, 0xa0}; // 7bit ASCII: "FEILCP"
uint8_t const base_satation_1[6] = {0x8c, 0x8a, 0x92, 0x84, 0xa6, 0x62}; // 7bit ASCII: "FEIBS1"
uint8_t const base_satation_2[6] = {0x8c, 0x8a, 0x92, 0x84, 0xa6, 0x64}; // 7bit ASCII: "FEIBS2"
uint8_t const ack_reply[4]= {0x41, 0x43, 0x4b, 0x00}; // ASCII: "ACK" - string ends with a NULL
uint8_t const devices_info[43] = {0x43, 0x50, 0x55, 0x3a, 0x20, 0x52, 0x4c, 0x37, 0x38, 0x52, 0x35, 0x46, 0x31,
                                  0x30, 0x30, 0x4c, 0x47, 0x20, 0x52, 0x46 ,0x3a, 0x20, 0x53, 0x58, 0x31, 0x32,
                                  0x37, 0x36, 0x20, 0x39, 0x31, 0x35, 0x20, 0x46, 0x53, 0x4b, 0x20, 0x41, 0x58,
                                  0x32, 0x35, 0x00, 0x00}; // ASCII: "CPU: RL78R5F100LG RF: SX1276 915 FSK AX25"

/**
 * @brief Function to analyze the received ax25 frame
 * 
 * @param source_address return the address of the source of this message
 * @param ax25_frame_type return if the message received is a COMMAND or a COMMAND type
 * @param received_ns_value return the received N(S) value (AX25 sequence number)
 * @param received_nr_value return the received N(R) value (AX25 next msg sequence number)
 * @param info return the info part of the frame
 * 
 * @return the result of the unpacking @link{ax25_ret_code_t}
 */
ax25_ret_code_t unpack_ax25_protocol (  uint8_t *source_address,
                                        uint8_t *ax25_frame_type,
                                        uint8_t *received_ns_value,
                                        uint8_t *received_nr_value,
                                        uint8_t *info) // TODO implement the RR and RNR functions, add new parameter in this function
{
    uint8_t aux_buffer[64] = {0};
    *ax25_frame_type = 0;
    uint8_t checksum = 0;
    int i;

    radio_read_fixed_packet (&aux_buffer[0], 64);

    if (aux_buffer[0] != 0x7e) // check initial flag
        return INITIAL_FLAG_ERROR;

    if (aux_buffer[63] != 0x7e) // check end flag
        return END_FLAG_ERROR;

    // header checksum check (addr + control + PID)
    for (i=1; i<=16; i++)
        checksum += aux_buffer[i];

    checksum = (~checksum) +1;
    if (checksum != aux_buffer[61])
        return HEADER_CHECKSUM_ERROR;

    /**********************ADDRESS**************************/
    // Destiny address analysis
    if ((aux_buffer[1]!=lcp_address[0])||
        (aux_buffer[2]!=lcp_address[1])||
        (aux_buffer[3]!=lcp_address[2])||
        (aux_buffer[4]!=lcp_address[3])||
        (aux_buffer[5]!=lcp_address[4])||
        (aux_buffer[6]!=lcp_address[5]))
        return ADDRESS_ERROR;

    // byte following the destiny address is SSID: CRRSSID0
    // C bit: 0=command / 1=response
    // R bit: reserved=1
    // SSID :0000b
    // LSbit is 1 for the end of the address field

    // destiny SSID byte analysis
    if ( ((aux_buffer[7]&0x60)!=0x60) || ((aux_buffer[7]&0x01)>0) ) // check SSID reserved bits
        return DESTINY_ADDR_SSID_ERROR;

    // Obtain Source address
    for(i=0; i<=5; i++)
        source_address[i] = aux_buffer[i+8];

    // Source SSID byte check
    if ( ((aux_buffer[14]&0x60)!=0x60) || ((aux_buffer[14]&0x01)==0) )
        return SOURCE_ADDR_SSID_ERROR;

    // Check if message is a command or a response
    if ( ((aux_buffer[7]&0x80)>0) && ((aux_buffer[14]&0x80)==0) )
        *ax25_frame_type = COMMAND;
    else if ( ((aux_buffer[7]&0x80)==0) && ((aux_buffer[14]&0x80)>0) )
        *ax25_frame_type = RESPONSE;
    else 
        return SSID_PARING_ERROR;

    /**********************CONTROL**************************/
    // Control byte (item 6.2.1) Information frame (I frame)
    // bits 7, 6, 5 :expected sequence number for the response frame (0 to 7)
    // bit  4       :polling bit - 1=request an immediate reply to this frame
    // bits 3, 2, 1 :send sequence number for this frame (0 to 7)
    // bit  0       :to identify the Information Control Byte always use 0

    if ((aux_buffer[15]&0x01)==0x01) // Supervisory frame
    {
        *received_nr_value = (aux_buffer[15]>>5)&0x07;
        *received_ns_value = 0; // there is no N(S) sequence number in the ax25 Supervisory frame
        // TODO implement the RR and RNR functions
        return SUPERVISORY_FRAME; // Supervisory frames have no info field
    }
    else if ((aux_buffer[15]&0x01)==0) // Information frame
    {
        *received_nr_value = (aux_buffer[15]>>5)&0x07;
        *received_ns_value = (aux_buffer[15]>>1)&0x07;
    }
    else
        return CONTROL_BYTE_ERROR;

    /************************PID****************************/
    // Protocol used in the Network layer of the frame (Layer 3 protocol used in the info field)
    if (aux_buffer[16] != 0xf0) // the LCP software does not use a Layer 3 protocol (code is 0xf0) in the payload
        return LCP_NO_LAYER3_ERROR;

    /************************INFO***************************/
    // retrieve the info bytes from the ax25 frame
    checksum = 0;
    for (i=0; i<=43; i++)
    {
        info[i] = aux_buffer[i+17];
        checksum += info[i];
    }

    checksum = (~checksum) +1;
    if ( checksum != aux_buffer[62] )
        return INFO_CHECKSUM_ERROR;

    return INFORMATION_FRAME;
}

/**
 * @brief As the packet is obtained the message manager provides the response process for each type os message
 * 
 * @param msg_ret_code return code from the unpacking
 * @param source_address pointer passing source address of the message
 * @param ax25_frame_type byte to inform the manager if a COMMAND or a RESPONSE has arrived
 * @param received_ns_value N(S)value of the message
 * @param received_nr_value N(R)value of the message
 * @param info pointer of the info field to be analyzed
 * 
 * @return zero if an error is detected in the message, one if a reply has been sent
 */
int ax25_message_manager ( ax25_ret_code_t msg_ret_code,
                           uint8_t         *source_address,
                           uint8_t         ax25_frame_type,
                           uint8_t         received_ns_value,
                           uint8_t         received_nr_value,
                           uint8_t         *info)
{
    uint8_t reply_info[44]={0};
    uint8_t reply_ns_value;
    int aux;
    uint8_t buff[64]={0};
    uint8_t target_address[6]={0};
    uint8_t msg_forward[44]={0};
    uint8_t analog_data[20]={0};
    uint8_t bmp280_data[6]={0};

    // if an error is detected in the address or flag fields there is no response message
    switch (msg_ret_code)
    {
        case INITIAL_FLAG_ERROR:
            led_3_blink ();
            return 0;
            break;

        case END_FLAG_ERROR:
            led_3_blink ();
            return 0;
            break;

        case HEADER_CHECKSUM_ERROR:
            led_3_blink ();
            return 0;
            break;

        case ADDRESS_ERROR:
            led_3_blink ();
            return 0;
            break;

        case DESTINY_ADDR_SSID_ERROR:
            led_3_blink ();
            return 0;
            break;

        case SOURCE_ADDR_SSID_ERROR:
            led_3_blink ();
            return 0;
            break;

        case SSID_PARING_ERROR:
            led_3_blink ();
            return 0;
            break;

        default:
            break; // if no compromising error is detected the manager continues
    }

    if (ax25_frame_type == COMMAND)
    {
        switch (msg_ret_code)
        {
            // in the other error cases a reply message is sent to inform the error if received a COMMAND
            case CONTROL_BYTE_ERROR:
                reply_info[0] = CONTROL_BYTE_ERROR_HEX; // 0xe1
                for (aux=1;aux<=43;aux++)
                    reply_info[aux]=0xa5; // empty info field

                system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                send_i_frame_reply(&source_address[0], 0, 1, &reply_info[0]);
                led_2_blink ();
                break;

            case LCP_NO_LAYER3_ERROR:
                reply_info[0] = LCP_NO_LAYER3_ERROR_HEX; // 0xe2
                for (aux=1;aux<=43;aux++)
                    reply_info[aux]=0xa5; // empty info field

                reply_ns_value = (received_nr_value&0x07);
                system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                led_2_blink ();
                break;

            case INFO_CHECKSUM_ERROR:
                reply_info[0] = INFO_CHECKSUM_ERROR_HEX; // 0xe3
                for (aux=1;aux<=43;aux++)
                    reply_info[aux]=0xa5; // empty info field

                reply_ns_value = (received_nr_value&0x07);
                system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                led_2_blink ();
                break;

            // if the frame is ok send the reply with the response bytes
            case SUPERVISORY_FRAME:
                system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                if (BATTERY_FAUL_PORT == 0)
                    send_s_frame_reply (&source_address[0], (received_nr_value&0x07), 0);
                else
                    send_s_frame_reply (&source_address[0], (received_nr_value&0x07), 1);
                    
                led_2_blink ();
                break;

            case INFORMATION_FRAME:
                switch (info[0])
                {
                    case READ_SENSORS_HEX: // 0xa1
                        reply_info[0] = READ_SENSORS_HEX;

                        analog_sensors_read (&analog_data[0]);
                        for (aux=0;aux<=19;aux++)
                            reply_info[aux+1]=analog_data[aux];

                        bmp280_read (&bmp280_data[0]);
                        for (aux=0;aux<=5;aux++)
                            reply_info[aux+21]=bmp280_data[aux];
                        
                        reply_ns_value = (received_nr_value&0x07);
                        system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                        send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                        led_2_blink ();
                        break;

                    case DEVICES_INFO_HEX: // 0xa2
                        reply_info[0] = DEVICES_INFO_HEX;
                        for (aux=0; aux<=42; aux++)
                            reply_info[aux+1] = devices_info[aux];
                        
                        reply_ns_value = (received_nr_value&0x07);
                        system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                        send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                        led_2_blink ();
                        break;
                        
                    case STRING_MESSAGE_HEX: // 0xa3
                        // reply th source station with an ACK
                        reply_info[0]=STRING_MESSAGE_HEX;
                        for (aux=0;aux<=38;aux++)
                            reply_info[aux+5]= 0x89; 

                        for(aux=0; aux<=3; aux++)
                            reply_info[aux+1]=ack_reply[aux]; //send string "ACK" to source Base Station
                            
                        reply_ns_value = received_nr_value&0x07;
                        system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                        send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                        led_2_blink ();
                        // forward the string received to the target station
                        for (aux=0;aux<=5;aux++)
                            target_address[aux]=info[aux+1]; // the addres to send the string is in the 1-6 bytes of the received payload

                        for (aux=0;aux<=43;aux++)
                            msg_forward[aux]=info[aux];

                        for (aux=0;aux;aux++)
                            msg_forward[aux+1]=source_address[aux];

                        system_delay_ms(200); // delay between messages
                        send_i_frame_command(&target_address[0], 0x04, 0x05, &msg_forward[0]); // retransmmit the payload to target
                        led_2_blink ();
                        break;

                    case RESET_RADIO_HEX: // 0xa4
                        // reply th source station with an ACK
                        reply_info[0]=RESET_RADIO_HEX;
                        for (aux=0;aux<=38;aux++)
                            reply_info[aux+5]= 0x89; 

                        for(aux=0; aux<=3; aux++)
                            reply_info[aux+1]=ack_reply[aux]; //send string "ACK" to source Base Station
                            
                        reply_ns_value = received_nr_value&0x07;
                        system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                        send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                        led_2_blink ();
                        radio_cold_reset (info[1]);
                        break;
                    
                    case SET_SUPERVISION_TIME_HEX: // 0xa5
                        // reply th source station with an ACK
                        reply_info[0]=SET_SUPERVISION_TIME_HEX;
                        for (aux=0;aux<=38;aux++)
                            reply_info[aux+5]= 0x89; 

                        for(aux=0; aux<=3; aux++)
                            reply_info[aux+1]=ack_reply[aux]; //send string "ACK" to source Base Station
                            
                        reply_ns_value = received_nr_value&0x07;
                        system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                        send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                        led_2_blink ();
                        if (info[1] > 1)
                            global_timeout_value = info[1];
                        else
                            global_timeout_value = 10;

                        break;
                        
                    case TEST_LED_BLINK_HEX: // 0xa6
                        // reply th source station with an ACK
                        reply_info[0]=TEST_LED_BLINK_HEX;
                        for (aux=0;aux<=38;aux++)
                            reply_info[aux+5]= 0x89; 

                        for(aux=0; aux<=3; aux++)
                            reply_info[aux+1]=ack_reply[aux]; //send string "ACK" to source Base Station
                            
                        reply_ns_value = received_nr_value&0x07;
                        system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                        send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                        if (info[1]>0)
                        {
                            for (aux=0; aux<=info[1]; aux++)
                            {
                                FAULT_LED_PORT = 0;
								PWR_ON_LED_PORT	= 0;
								LED_PORT = 1;
                                system_delay_ms(50);
                                LED_PORT = 0;
                                system_delay_ms(100);
                            }
                        }
                        else
                            led_2_blink ();
                        
                        break;
                        
                    case CHANGE_BIT_RATE_HEX: // 0x0a7
                        // reply th source station with an ACK
                        reply_info[0]=CHANGE_BIT_RATE_HEX;
                        for (aux=0;aux<=38;aux++)
                            reply_info[aux+5]= 0x89; 

                        for(aux=0; aux<=3; aux++)
                            reply_info[aux+1]=ack_reply[aux]; //send string "ACK" to source Base Station
                            
                        reply_ns_value = received_nr_value&0x07;
                        system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                        send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                        
                        if (info[1] > 0 || info[2] > 0)
                            radio_config_bit_rate (info[1], info[2], info[3], info[4]);
                        else
                            radio_config_bit_rate (0x1a, 0x0b, 0x00, 0x52);
                        
                        led_2_blink ();
                        
                        break;
                        
                    case SWITCH_BATTERY_HEX: // 0xa8
                        reply_info[0]=SWITCH_BATTERY_HEX;
                        reply_info[1]=BATTERY_SWITCH_PORT; // Status atual do GPIO
                        
                        reply_ns_value = received_nr_value&0x07;
                        
                        system_delay_ms(10); // necessary delay to synchronize the reply with the base station
                        send_i_frame_reply(&source_address[0], reply_ns_value, (reply_ns_value+1), &reply_info[0]);
                        
                        if (info[1] == 0xff)
                            BATTERY_SWITCH_PORT = 0;
                        else if (info[1] == 0xf0)
                            BATTERY_SWITCH_PORT = 1;
                        else
                            __no_operation();
                            
                        led_2_blink ();
                        break;
                        
                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }
    else if (ax25_frame_type == RESPONSE) // LCP does not expect replies from the Base Station
    {                                     // An non AX25 payload is send to the radios in this case
        switch (msg_ret_code)
        {
            case LCP_NO_LAYER3_ERROR:
                system_delay_ms(10);
                buff[0]=0xf1;
                radio_send_packet (&buff[0], 64);
                break;

            case INFO_CHECKSUM_ERROR:
                system_delay_ms(10);
                buff[0]=0xf2;
                radio_send_packet (&buff[0], 64);
                break;

            case SUPERVISORY_FRAME: // no response for supervisory frames expected
                system_delay_ms(10);
                buff[0]=0xf3;
                radio_send_packet (&buff[0], 64);
                break;

            case INFORMATION_FRAME:
                system_delay_ms(10);
                buff[0]=0xf4;
                radio_send_packet (&buff[0], 64);
                break;

            default:
                break;
        }
    }

    return 1;
}

/**
 * @brief Send an AX25 Information frame as a reply (through the radio device, sx1276 frame)
 * 
 * @param destiny_address the address of the reply message
 * @param ns_value N(S) sequential number (WARNING: the reply message must send the N(S) number equal to the N(R) number of the COMMAND msg received)
 * @param nr_value N(R) sequence number of the reply
 * @param info payload of the reply
 */
void send_i_frame_reply (uint8_t *destiny_address, uint8_t ns_value, uint8_t nr_value, uint8_t *info)
{
    lcp_ax25_protocol_t frame_to_send;
    int aux;

    for (aux=0;aux<=5;aux++)
        frame_to_send.destiny_address[aux] = destiny_address[aux];

    frame_to_send.destiny_ssid = 0x60;

    for (aux=0;aux<=5;aux++)
        frame_to_send.source_address[aux] = lcp_address[aux];

    frame_to_send.source_ssid = 0xe1;
    
    frame_to_send.control = (((nr_value<<5)|(ns_value<<1))|0x10);

    frame_to_send.protocol_identification = 0xf0; // no Layer 3 protocol used

    for (aux=0;aux<=43;aux++)
        frame_to_send.info[aux] = info[aux];

    ax25_to_binary (frame_to_send);
}

/**
 * @brief Send an AX25 Information frame as a command (through the radio device, sx1276 frame)
 * 
 * @param destiny_address the target address of the message
 * @param ns_value N(S) sequential number
 * @param nr_value N(R) sequence number (this value is used to inform the expected N(S) value from the reply message)
 * @param info payload of the command
 */
void send_i_frame_command (uint8_t *destiny_address, uint8_t ns_value, uint8_t nr_value, uint8_t *info)
{
    lcp_ax25_protocol_t frame_to_send;
    int aux;

    for (aux=0;aux<=5;aux++)
        frame_to_send.destiny_address[aux] = destiny_address[aux];

    frame_to_send.destiny_ssid = 0xe0;

    for (aux=0;aux<=5;aux++)
        frame_to_send.source_address[aux] = lcp_address[aux];

    frame_to_send.source_ssid = 0x61;

    frame_to_send.control = (((nr_value<<5)|(ns_value<<1))|0x10);

    frame_to_send.protocol_identification = 0xf0; // no Layer 3 protocol used

    for (aux=0;aux<=43;aux++)
        frame_to_send.info[aux] = info[aux];

    ax25_to_binary (frame_to_send);
}

/**
 * @brief Send an AX25 Supervisory frame as a reply (through the radio device, sx1276 frame)
 * 
 * @param destiny_address the address of the reply message
 * @param nr_value N(R) sequence number of the reply
 * 
 * @note in the Supervisory frames there is no Info field neither an N(S) value
 */
void send_s_frame_reply (uint8_t *destiny_address, uint8_t nr_value, uint8_t ready)
{
    lcp_ax25_protocol_t frame_to_send;
    int aux;

    for (aux=0;aux<=5;aux++)
        frame_to_send.destiny_address[aux] = destiny_address[aux];

    frame_to_send.destiny_ssid = 0x60;

    for (aux=0;aux<=5;aux++)
        frame_to_send.source_address[aux] = lcp_address[aux];

    frame_to_send.source_ssid = 0xe1;
    
    if(ready == 1)
        frame_to_send.control = ((nr_value<<5)|0x01); // RR
    else
        frame_to_send.control = ((nr_value<<5)|0x05); // RNR
        
    frame_to_send.protocol_identification = 0xf0; // no Layer 3 protocol used

    for (aux=0;aux<=43;aux++)
        frame_to_send.info[aux] = 0xa5;

    ax25_to_binary (frame_to_send);
}

/**
 * @brief Send an AX25 Supervisory frame as a command (through the radio device, sx1276 frame)
 * 
 * @param destiny_address the target address of the message
 * @param nr_value N(R) sequence number
 * 
 * @note in the Supervisory frames there is no Info field neither an N(S) value
 */
void send_s_frame_command (uint8_t *destiny_address, uint8_t nr_value, uint8_t ready)
{
    lcp_ax25_protocol_t frame_to_send;
    int aux;

    for (aux=0;aux<=5;aux++)
        frame_to_send.destiny_address[aux] = destiny_address[aux];

    frame_to_send.destiny_ssid = 0xe0;

    for (aux=0;aux<=5;aux++)
        frame_to_send.source_address[aux] = lcp_address[aux];

    frame_to_send.source_ssid = 0x61;
    if (ready == 1)
        frame_to_send.control = ((nr_value<<5)|0x01); // Receiver Ready (RR)
    else
        frame_to_send.control = ((nr_value<<5)|0x05); // Receiver Not Ready (RNR) - System FAULT
        
    frame_to_send.protocol_identification = 0xf0; // no Layer 3 protocol used

    for (aux=0;aux<=43;aux++)
        frame_to_send.info[aux] = 0xa5;

    ax25_to_binary (frame_to_send);
}

/**
 * @brief Get the AX25 frame strcut and assemble the array to be transmitted through the radio device.
 *        Also calculate the Checksum values of the header and info fields.
 * 
 * @param frame_to_send struct with the data of the AX25 protocol information
 */
void ax25_to_binary (lcp_ax25_protocol_t frame_to_send)
{
    uint8_t buffer [64];
    int aux;
    uint8_t checksum = 0;

    buffer[0] = 0x7e;

    for (aux=0;aux<=5;aux++)
        buffer[aux+1] = frame_to_send.destiny_address[aux];

    buffer[7] = frame_to_send.destiny_ssid;

    for (aux=0;aux<=5;aux++)
        buffer[aux+8] = frame_to_send.source_address[aux];

    buffer[14] = frame_to_send.source_ssid;
    buffer[15] = frame_to_send.control;
    buffer[16] = frame_to_send.protocol_identification;

    for (aux=0;aux<=43;aux++)
    {
        buffer[aux+17] = frame_to_send.info[aux];
        checksum += frame_to_send.info[aux];
    }
    buffer[62] = (~checksum) +1; // info field checksum

    checksum = 0;
    for (aux=1;aux<=16;aux++)
        checksum += buffer[aux];

    buffer[61] = (~checksum) +1; // address, ssid, control and PID checksum (header checksum)

    buffer[63] = 0x7e;

    radio_send_packet (&buffer[0], 64);
}
