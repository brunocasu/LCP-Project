import sys
import RPi.GPIO as GPIO
import time
import spidev
import random
import math

spi_ch = 0

# Enable SPI
spi = spidev.SpiDev(0, spi_ch)
spi.max_speed_hz = 1000000
spi.mode         = 0b01 #01 for sx1276 and 00 for bmp280

def radio_read(addr):
    # Read mode: first byte to be sent  corresponds to the register address/ second byte read is the register value
    byte0 = 0b00000000 + (addr & 0b01111111)
    byte1 = 0b00000000
    reply = spi.xfer2([byte0, byte1])
    return reply[1]

def radio_write(addr, value):
    # Write mode: firrst byte to be send is the register address, second byte i the desired value to be written in the register
    byte0 = 0b10000000 + (addr & 0b01111111)
    byte1 = 0b11111111 & value
    reply = spi.xfer2([byte0, byte1])
    return reply[1]


def radio_rssi_floor ():
    rssi = radio_read(0x11)
    print("FLOOR RSSI: -", (rssi/2), "dBm")
    return

def radio_reg_cfg():
    #put in stby mode and power output to zero
    radio_write(0x01, 0x01)
    radio_write(0x09,0x00)
    #frequency config (915MHz):
    radio_write(0x06,0xe5)
    radio_write(0x07,0xc0)
    radio_write(0x08,0x26)
    #calibration:
    radio_write(0x3b, 0xc2)
    time.sleep(0.001)
    #return power to max and put radio in sleep mode
    radio_write(0x09,0xff)
    radio_write(0x01,0x00)
    #set preamble:
    radio_write(0x25,0x00)
    radio_write(0x26,0x10)
    #sync addr config
    radio_write(0x27,0x97)
    radio_write(0x28,0x20)
    radio_write(0x29,0x20)
    radio_write(0x2a,0x02)
    radio_write(0x2b,0x27)
    radio_write(0x2c,0x54)
    radio_write(0x2d,0x52)
    radio_write(0x2e,0x49)
    radio_write(0x2f,0x45)
    #pkt config
    radio_write(0x30,0x10)
    radio_write(0x32,0x40)
    radio_write(0x33,0x8a)
    radio_write(0x35,0x3f)
    radio_write(0x40,0x04)
    if radio_read(0x42) == 0x12:
        print("Radio Configuration OK")
    else:
        print("SX1276 Configuration ERROR")
    
    return

def radio_rx_cfg():
    sx1276_DIO2 = 15
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(sx1276_DIO2, GPIO.IN)

    radio_write(0x01,0x00)
    radio_write(0x01,0x05)
    dio2 = GPIO.input(sx1276_DIO2)

    while (dio2!=1):
        dio2=GPIO.input(sx1276_DIO2)

    result = radio_read(0x01)
    return result

def radio_tx_cfg():
    radio_write(0x01,0x00)
    time.sleep(0.003)
    radio_write(0x01,0x03)
    time.sleep(0.005)

    result = radio_read(0x01)
    return result

def base(timeout): #timeout value inserted is in seconds
    sx1276_DIO0 = 11
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(sx1276_DIO0, GPIO.IN)
    buff = [0]*64
    counter=0
    if timeout > 0:
        radio_rx_cfg()
        dio0=GPIO.input(sx1276_DIO0)
        while ( dio0!=1 and (counter < (timeout*10000)) ):
            dio0=GPIO.input(sx1276_DIO0)
            counter+=1
            time.sleep(0.00001)
        
        if dio0==1:
            k=0
            rssi = radio_read(0x11)
            while k<64:
                buff[k]=radio_read(0)
                k+=1
            
            reg_value = radio_read(0x3f)
            radio_write(0x3f, reg_value | 0x10) #reset FIFO
            unpack_ax25 (buff)
            print("TIME TO RESPOND:", (counter/10), "ms")
            print("RECEIVED SIGNAL STRENGTH INDICATION: -", (rssi/2), "dBm")
            print("******************************************************************")
            time.sleep(0.1)
        else:
            if timeout != 2:
                print("TIMEOUT...")
    
    else:
        print("\nRECEIVER MODE")
        while True:
            time.sleep(0.01)
            reg_value = radio_read(0x3f)
            radio_write(0x3f, reg_value | 0x10) #reset FIFO
            radio_rx_cfg()
            dio0=GPIO.input(sx1276_DIO0)
            while ( dio0!=1):
                dio0=GPIO.input(sx1276_DIO0)
                time.sleep(0.00001)
            
            k=0
            rssi = radio_read(0x11)
            while k<64:
                buff[k]=radio_read(0)
                k+=1
                
            reg_value = radio_read(0x3f)
            radio_write(0x3f, reg_value | 0x10) #reset FIFO
            ret_val = unpack_ax25 (buff)
            if ret_val == 1:    
                print("RSSI: -", (rssi/2), "dBm")
                print("******************************************************************")
            
    
    return

def unpack_ax25 (pkt_data):
    
    checksum=0
    for h in range (1, 17):
        checksum+=pkt_data[h]
        
    if ((checksum+pkt_data[61])&0x00ff)!=0:
        print("\nHEADER CHECKSUM ERROR")
        return 0
    
    checksum=0
    for p in range (17, 61):
        checksum+=pkt_data[p]
        
    if ((checksum+pkt_data[62])&0x00ff)!=0:
        print("\nINFO CHECKSUM ERROR")
        return 0
    
    if ((pkt_data[1]!=0x8c) or (pkt_data[2]!=0x8a) or (pkt_data[3]!=0x92) or (pkt_data[4]!=0x84) or (pkt_data[5]!=0xa6) or (pkt_data[6]!=0x62)): 
        #print("\nMESSAGE RECEIVED WITH WRONG ADDRESS")
        return 0

    source_addr = [0]*6
    source_addr_ascii = [0]*6
    for k in range(0, 6):
        source_addr[k] = pkt_data[k+8]
        source_addr_ascii[k] = (pkt_data[k+8])>>1
        
    source_addr_str = "".join([chr(c) for c in source_addr_ascii])
    if ( ((pkt_data[7]&0x80)>0) and ((pkt_data[14]&0x80)==0) ):
        frame_type = 1 #COMMAND
    elif ( ((pkt_data[7]&0x80)==0) and ((pkt_data[14]&0x80)>0) ):
        frame_type = 2 #RESPONSE
    else :
        print("\nSSID PARING ERROR")
        return 0
    
    rec_ns = (pkt_data[15]>>1)&0x07;
    rec_nr = (pkt_data[15]>>5)&0x07;
    str_source_addr_ascii=[0]*6
    msg_str_ascii=[0]*37
    if frame_type == 1:
        print("\nMESSAGE RECEIVED (COMMAND)")
        if (pkt_data[15] & 0x05) == 0x01 :
            print("SUPERVISORY FRAME FROM:", source_addr_str, "| N(R):",hex(rec_nr))
            return 1
        elif (pkt_data[15] & 0x05) == 0x05 :
            print("SUPERVISORY FRAME FROM:", source_addr_str, "| N(R):",hex(rec_nr))
            print("WARNING: CONTROL BYTE RECEIVED WITH RNR - BATTERY FAULT")
            
        elif (pkt_data[15] & 0x01) == 0:
            print("INFORMATION FRAME FROM:", source_addr_str, "| SEQUENCE N(S):", hex(rec_ns), "| PAYLOAD CODE:", hex(pkt_data[17]))
            if pkt_data[17] == 0xa3:
                str_source_addr = [0]*6
                for k in range(0, 6):
                    str_source_addr_ascii[k]=(pkt_data[18+k]>>1)
                
                str_source_addr_str = "".join([chr(c) for c in str_source_addr_ascii])
                for k in range(0, 37):
                    msg_str_ascii[k]=pkt_data[24+k]
            
                msg_str_str = "".join([chr(c) for c in msg_str_ascii])
                print("STRING MESSAGE FROM:", str_source_addr_str, "\nMESSAGE:", msg_str_str)
                return 1
        
        else:
            print("\nCONTROL BYTE ERROR")
            return 0
    
    elif frame_type == 2:
        print("\nMESSAGE RECEIVED (RESPONSE)")
        if (pkt_data[15] & 0x05) == 0x01 :
            print("REPLY OF SUPERVISORY FRAME FROM:", source_addr_str, "| N(R):",hex(rec_nr))
            
        elif (pkt_data[15] & 0x01) == 0:
            print("REPLY OF INFORMATION FRAME FROM:", source_addr_str, "| SEQUENCE N(S):", hex(rec_ns), "| PAYLOAD CODE:", hex(pkt_data[17]))
            if pkt_data[17] == 0xa3:
                ack_msg = [0]*3
                for k in range(0, 3):
                    ack_msg[k]=(pkt_data[18+k])
                    
                ack_str = "".join([chr(c) for c in ack_msg])
                print("STRING MESSAGE ACKNOWLEDGE REPLY:", ack_str)
                return 1
            
            elif pkt_data[17] == 0xa2:
                devices_info_str = [0]*43
                for k in range(0, 43):
                    devices_info_str[k]=(pkt_data[18+k])
                    
                devices_info = "".join([chr(c) for c in devices_info_str])
                print("DEVICES INFO REPLY:", devices_info)
                return 1
            
            elif pkt_data[17] == 0xa4:
                ack_msg = [0]*3
                for k in range(0, 3):
                    ack_msg[k]=(pkt_data[18+k])
                    
                ack_str = "".join([chr(c) for c in ack_msg])
                print("RESET RADIO ACKNOWLEDGE REPLY:", ack_str)
                return 1
            
            elif pkt_data[17] == 0xa5:
                ack_msg = [0]*3
                for k in range(0, 3):
                    ack_msg[k]=(pkt_data[18+k])
                    
                ack_str = "".join([chr(c) for c in ack_msg])
                print("SET SUPERVISION TIME ACKNOWLEDGE REPLY:", ack_str)
                return 1
            
            elif pkt_data[17] == 0xa6:
                ack_msg = [0]*3
                for k in range(0, 3):
                    ack_msg[k]=(pkt_data[18+k])
                    
                ack_str = "".join([chr(c) for c in ack_msg])
                print("TEST LED BLINK ACKNOWLEDGE REPLY:", ack_str)
                return 1
            
            elif pkt_data[17] == 0xa7:
                ack_msg = [0]*3
                for k in range(0, 3):
                    ack_msg[k]=(pkt_data[18+k])
                    
                ack_str = "".join([chr(c) for c in ack_msg])
                print("SET BIT RATE ACKNOWLEDGE REPLY:", ack_str)
                return 1
            
            elif pkt_data[17] == 0xa8:
                if pkt_data[18] == 1:
                    print("BATTERY STATUS REPLY: MAIN CIRCUIT ON")
                elif pkt_data[18] == 0:
                    print("BATTERY STATUS REPLY: BACKUP CIRCUIT ON")
                else:
                    print("BATTERY STATUS REPLY:", pkt_data[18])
                    
                return 1
            
             
            elif pkt_data[17] == 0xa1:
                ntc0_raw = (pkt_data[18]<<8) + pkt_data[19]
                ntc1_raw = (pkt_data[20]<<8) + pkt_data[21]
                ldr0_raw = (pkt_data[22]<<8) + pkt_data[23]
                ldr1_raw = (pkt_data[24]<<8) + pkt_data[25]
                ldr2_raw = (pkt_data[26]<<8) + pkt_data[27]
                ldr3_raw = (pkt_data[28]<<8) + pkt_data[29]
                ldr4_raw = (pkt_data[30]<<8) + pkt_data[31]
                ldr5_raw = (pkt_data[32]<<8) + pkt_data[33]
                vcc_raw  = (pkt_data[34]<<8) + pkt_data[35]
                amp_raw  = (pkt_data[36]<<8) + pkt_data[37]
                temp_xlsb = pkt_data[40]
                temp_lsb  = pkt_data[39]
                temp_msb  = pkt_data[38]
                pres_xlsb = pkt_data[43]
                pres_lsb  = pkt_data[42]
                pres_msb  = pkt_data[41]
                bmp280_temp_raw = (pkt_data[40]/128) + (pkt_data[39]*16) + (pkt_data[38]*4096)
                bmp280_pres_raw = (pkt_data[43]/128) + (pkt_data[42]*16) + (pkt_data[41]*4096)
                dig_T1 = 27504
                dig_T2 = 26435
                x1 = bmp280_temp_raw/8
                x2 = x1 - (dig_T1*2)
                var1 = (x2*dig_T2)/2048
                y1 = (bmp280_temp_raw/16 - dig_T1)*(bmp280_temp_raw/16 - dig_T1)
                var2 = ((y1/4096)*1000)/16384
                t_fine = var1 - var2
                T = ((t_fine*5) +128)/256
                temp_absolut_read = T/100
                
                dig_P1 = 36477
                dig_P2 = -10685
                dig_P3 = 3024
                dig_P4 = 2855
                dig_P5 = 140
                dig_P6 = -7
                dig_P7 = 15500
                dig_P8 = -14600
                dig_P9 = 6000
                var1 = (t_fine/2) - 64000
                var2 = var1*var1*(dig_P6)/32768
                var2 = var2 + (var1*dig_P5*2)
                var2 = (var2/4) + (dig_P4*65536)
                var1 = ((dig_P3*var1*var1/524288) + (dig_P2*var1))/524288
                var1 = (1 + var1/32768)*dig_P1
                p = 1048576 - bmp280_pres_raw
                p = (p -(var2/4096))*(6250/var1)
                var1 = dig_P9*p*p/2147483648
                var2 = p*dig_P8/32768
                pres_absolut_read = p + (var1+var2+dig_P7)/16 #Pascal
                pres_in_mmhg = 0.00750062*pres_absolut_read
                
                print("SENSORS READING REPLY: NTC0", hex(ntc0_raw), "| NTC1", hex(ntc1_raw), "| LDR0", hex(ldr0_raw), "| LDR1", hex(ldr1_raw), "| LDR2", hex(ldr2_raw), "| LDR3", hex(ldr3_raw), "| LDR4", hex(ldr4_raw), "| LDR5", hex(ldr5_raw), "| VCC", hex(vcc_raw), "| AMP", hex(amp_raw) )
                print("BMP280 Temp:", "%.2f" % temp_absolut_read,"C")
                print("BMP280 Pres:", "%.2f" % pres_absolut_read,"Pa", "|", "%.2f" % pres_in_mmhg, "mmHg")
                #print ("BMP280 TEMP =", hex(bmp280_temp_raw), "C | BMP280 PRES = ", hex(bmp280_pres_raw), "hPa") 
                
            else:
                print("UNIDENTIFIED LCP COMMAND")
            
        else:
            print("\nCONTROL BYTE ERROR")     
            return 0     
             
    return 0


#request the sensors readings of the LCP (raw values are sent) - 0xa1
def send_sensor_read ():
    sbuff=[0]*64
    sbuff[0] = 0x7e
    sbuff[1] = 0x8c
    sbuff[2] = 0x8a
    sbuff[3] = 0x92
    sbuff[4] = 0x98
    sbuff[5] = 0x86
    sbuff[6] = 0xa0
    sbuff[7] = 0xe0
    sbuff[8] = 0x8c
    sbuff[9] = 0x8a
    sbuff[10] = 0x92
    sbuff[11] = 0x84
    sbuff[12] = 0xa6
    sbuff[13] = 0x62
    sbuff[14] = 0x61
    ns = random.getrandbits(3) #radom integer of 3 bits
    nr = (ns+1)&0x07 
    sbuff[15] = (nr<<5) | (ns<<1) | 0x10
    sbuff[16] = 0xf0
    sbuff[17] = 0xa1
    
    checksum=0
    for i in range (1, 17):
        checksum+=sbuff[i]
        
    sbuff[61] = (~checksum)+1
    checksum=0
    for i in range (17, 61):
        checksum+=sbuff[i]
    
    sbuff[62] = (~checksum)+1
    sbuff[63] = 0x7e
    #send the packet to the FIFO
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    radio_tx_cfg()
    for i in range (0, 64):
        radio_write(0, sbuff[i])
    
    while (GPIO.input(11)==0):
        time.sleep(0.00001)
    
    print ("INFORMATION FRAME SENT: REQUEST SENSOR READINGS")
    print ("SEQUENCE NUMBER N(R):", hex(nr))
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    print ("WAITING FOR REPLY...")
    
    base(4) ## wait for message with a timeout
    base(0) ## this maintains the radio at the receiver routine after the end of the transmission
    return


#request infomation of the devices in the LCP - 0xa2
def send_devices_info ():
    sbuff=[0]*64
    sbuff[0] = 0x7e
    sbuff[1] = 0x8c
    sbuff[2] = 0x8a
    sbuff[3] = 0x92
    sbuff[4] = 0x98
    sbuff[5] = 0x86
    sbuff[6] = 0xa0
    sbuff[7] = 0xe0
    sbuff[8] = 0x8c
    sbuff[9] = 0x8a
    sbuff[10] = 0x92
    sbuff[11] = 0x84
    sbuff[12] = 0xa6
    sbuff[13] = 0x62
    sbuff[14] = 0x61
    ns = random.getrandbits(3) #radom integer of 3 bits
    nr = (ns+1)&0x07 
    sbuff[15] = (nr<<5) | (ns<<1) | 0x10
    sbuff[16] = 0xf0
    sbuff[17] = 0xa2
    
    checksum=0
    for i in range (1, 17):
        checksum+=sbuff[i]
        
    sbuff[61] = (~checksum)+1
    checksum=0
    for i in range (17, 61):
        checksum+=sbuff[i]
    
    sbuff[62] = (~checksum)+1
    sbuff[63] = 0x7e
    #send the packet to the FIFO
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    radio_tx_cfg()
    for i in range (0, 64):
        radio_write(0, sbuff[i])
    
    while (GPIO.input(11)==0):
        time.sleep(0.00001)
    
    print ("INFORMATION FRAME SENT: REQUEST DEVICE INFORMATION")
    print ("SEQUENCE NUMBER N(R):", hex(nr))
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    print ("WAITING FOR REPLY...")
    
    base(4) ## wait for message with a timeout
    base(0) ## this maintains the radio at the receiver routine after the end of the transmission
    return


#send a string message to another base station using LCP as a hub - 0xa3
def send_string_message (target, msg):
    sbuff=[0]*64
    sbuff[0] = 0x7e
    sbuff[1] = 0x8c
    sbuff[2] = 0x8a
    sbuff[3] = 0x92
    sbuff[4] = 0x98
    sbuff[5] = 0x86
    sbuff[6] = 0xa0
    sbuff[7] = 0xe0
    sbuff[8] = 0x8c
    sbuff[9] = 0x8a
    sbuff[10] = 0x92
    sbuff[11] = 0x84
    sbuff[12] = 0xa6
    sbuff[13] = 0x62
    sbuff[14] = 0x61
    ns = random.getrandbits(3) #radom integer of 3 bits
    nr = (ns+1)&0x07 
    sbuff[15] = (nr<<5) | (ns<<1) | 0x10
    sbuff[16] = 0xf0
    sbuff[17] = 0xa3
    target_break = [char for char in target]
    target_in_ascii_code = [ord(ele) for sub in target_break for ele in sub]
    msg_break = [char for char in msg]
    msg_in_ascii_code = [ord(ele) for sub in msg_break for ele in sub]
    for i in range (0, 6):
        sbuff[i+18] = (target_in_ascii_code[i]<<1)
    
    if (len(msg_in_ascii_code)) > 37:
        for i in range (0, 37):
            sbuff[i+24] = msg_in_ascii_code[i]
    else:
         for i in range (0, len(msg_in_ascii_code)):
             sbuff[i+24] = msg_in_ascii_code[i]
    
    checksum=0
    for i in range (1, 17):
        checksum+=sbuff[i]
        
    sbuff[61] = (~checksum)+1
    checksum=0
    for i in range (17, 61):
        checksum+=sbuff[i]
    
    sbuff[62] = (~checksum)+1
    sbuff[63] = 0x7e
    #send the packet to th FIFO
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    radio_tx_cfg()
    for i in range (0, 64):
        radio_write(0, sbuff[i])
    
    while (GPIO.input(11)==0):
        time.sleep(0.00001)
    
    print ("INFORMATION FRAME SENT: STRING MESSAGE")
    print ("STRING SENT TO:", target, "| SEQUENCE NUMBER N(R):", hex(nr))
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    print ("WAITING FOR REPLY...")
    
    base(4) ## wai for message with a timeout of 2 seconds
    #base(2) ## to prevent locking (bug)
    base(0) ## this maintains the radio at the receiver routine after the end of the transmission
    return

#send command to reset the radio device for X*10 miliseconds - 0xa4
def send_reset_radio (time_to_reset):
    sbuff=[0]*64
    sbuff[0] = 0x7e
    sbuff[1] = 0x8c
    sbuff[2] = 0x8a
    sbuff[3] = 0x92
    sbuff[4] = 0x98
    sbuff[5] = 0x86
    sbuff[6] = 0xa0
    sbuff[7] = 0xe0
    sbuff[8] = 0x8c
    sbuff[9] = 0x8a
    sbuff[10] = 0x92
    sbuff[11] = 0x84
    sbuff[12] = 0xa6
    sbuff[13] = 0x62
    sbuff[14] = 0x61
    ns = random.getrandbits(3) #radom integer of 3 bits
    nr = (ns+1)&0x07 
    sbuff[15] = (nr<<5) | (ns<<1) | 0x10
    sbuff[16] = 0xf0
    sbuff[17] = 0xa4
    if time_to_reset>0 and time_to_reset<256:
        sbuff[18] = time_to_reset
    elif time_to_reset == 0:
        sbuff[18] = 0
    else:
        print("WRONG PARAMETERS INSERTED")
        return
        
    checksum=0
    for i in range (1, 17):
        checksum+=sbuff[i]
        
    sbuff[61] = (~checksum)+1
    checksum=0
    for i in range (17, 61):
        checksum+=sbuff[i]
    
    sbuff[62] = (~checksum)+1
    sbuff[63] = 0x7e
    #send the packet to the FIFO
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    radio_tx_cfg()
    for i in range (0, 64):
        radio_write(0, sbuff[i])
    
    while (GPIO.input(11)==0):
        time.sleep(0.00001)
    
    print ("INFORMATION FRAME SENT: RESET RADIO FOR", (time_to_reset*10), "ms")
    print ("SEQUENCE NUMBER N(R):", hex(nr))
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    print ("WAITING FOR REPLY...")
    
    base(4) ## wait for message with a timeout
    base(0) ## this maintains the radio at the receiver routine after the end of the transmission
    return


#send command to set the interval of supervision - 0xa5
def send_set_sup_time (interval):
    sbuff=[0]*64
    sbuff[0] = 0x7e
    sbuff[1] = 0x8c
    sbuff[2] = 0x8a
    sbuff[3] = 0x92
    sbuff[4] = 0x98
    sbuff[5] = 0x86
    sbuff[6] = 0xa0
    sbuff[7] = 0xe0
    sbuff[8] = 0x8c
    sbuff[9] = 0x8a
    sbuff[10] = 0x92
    sbuff[11] = 0x84
    sbuff[12] = 0xa6
    sbuff[13] = 0x62
    sbuff[14] = 0x61
    ns = random.getrandbits(3) #radom integer of 3 bits
    nr = (ns+1)&0x07 
    sbuff[15] = (nr<<5) | (ns<<1) | 0x10
    sbuff[16] = 0xf0
    sbuff[17] = 0xa5
    if interval>1 and interval<255:
        sbuff[18] = interval
    else:
        print("WRONG PARAMETERS INSERTED")
        return
        
    checksum=0
    for i in range (1, 17):
        checksum+=sbuff[i]
        
    sbuff[61] = (~checksum)+1
    checksum=0
    for i in range (17, 61):
        checksum+=sbuff[i]
    
    sbuff[62] = (~checksum)+1
    sbuff[63] = 0x7e
    #send the packet to the FIFO
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    radio_tx_cfg()
    for i in range (0, 64):
        radio_write(0, sbuff[i])
    
    while (GPIO.input(11)==0):
        time.sleep(0.00001)
    
    print ("INFORMATION FRAME SENT: SET SUPERVISION TIME TO", interval, "SECONDS")
    print ("SEQUENCE NUMBER N(R):", hex(nr))
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    print ("WAITING FOR REPLY...")
    
    base(4) ## wait
    base(0) ## this maintains the radio at the receiver routine after the end of the transmission
    return


#send command to set blink the LED X times - 0xa6
def send_blink (n_blink):
    sbuff=[0]*64
    sbuff[0] = 0x7e
    sbuff[1] = 0x8c
    sbuff[2] = 0x8a
    sbuff[3] = 0x92
    sbuff[4] = 0x98
    sbuff[5] = 0x86
    sbuff[6] = 0xa0
    sbuff[7] = 0xe0
    sbuff[8] = 0x8c
    sbuff[9] = 0x8a
    sbuff[10] = 0x92
    sbuff[11] = 0x84
    sbuff[12] = 0xa6
    sbuff[13] = 0x62
    sbuff[14] = 0x61
    ns = random.getrandbits(3) #radom integer of 3 bits
    nr = (ns+1)&0x07 
    sbuff[15] = (nr<<5) | (ns<<1) | 0x10
    sbuff[16] = 0xf0
    sbuff[17] = 0xa6
    if n_blink>0 and n_blink<255:
        sbuff[18] = n_blink
    else:
        print("WRONG PARAMETERS INSERTED")
        return
        
    checksum=0
    for i in range (1, 17):
        checksum+=sbuff[i]
        
    sbuff[61] = (~checksum)+1
    checksum=0
    for i in range (17, 61):
        checksum+=sbuff[i]
    
    sbuff[62] = (~checksum)+1
    sbuff[63] = 0x7e
    #send the packet to the FIFO
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    radio_tx_cfg()
    for i in range (0, 64):
        radio_write(0, sbuff[i])
    
    while (GPIO.input(11)==0):
        time.sleep(0.00001)
    
    print ("INFORMATION FRAME SENT: DEBUG LED BLINK", n_blink, "TIMES")
    print ("SEQUENCE NUMBER N(R):", hex(nr))
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    print ("WAITING FOR REPLY...")
    
    base(4) ## wai for message with a timeout of 2 seconds
    base(0) ## this maintains the radio at the receiver routine after the end of the transmission
    return

def send_supervisory (nr):
    sbuff=[0]*64
    sbuff[0] = 0x7e
    sbuff[1] = 0x8c
    sbuff[2] = 0x8a
    sbuff[3] = 0x92
    sbuff[4] = 0x98
    sbuff[5] = 0x86
    sbuff[6] = 0xa0
    sbuff[7] = 0xe0
    sbuff[8] = 0x8c
    sbuff[9] = 0x8a
    sbuff[10] = 0x92
    sbuff[11] = 0x84
    sbuff[12] = 0xa6
    sbuff[13] = 0x62
    sbuff[14] = 0x61
    nr = nr&(0x07)
    sbuff[15] = (nr<<5)|0x1d
    sbuff[16] = 0xf0

    checksum=0
    for i in range (1, 17):
        checksum+=sbuff[i]
        
    sbuff[61] = (~checksum)+1
    checksum=0
    for i in range (17, 61):
        checksum+=sbuff[i]
    
    sbuff[62] = (~checksum)+1
    sbuff[63] = 0x7e
    
    #send the packet to th FIFO
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    radio_tx_cfg()
    for i in range (0, 64):
        radio_write(0, sbuff[i])
    
    while (GPIO.input(11)==0):
        time.sleep(0.00001)
    print ("SUPERVISORY COMMAND SENT | SEQUENCE NUMBER N(R)", hex(nr))
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    print ("WAITING FOR REPLY...")
    
    base(4) ## wai for message with a timeout of 2 seconds
    base(0) ## this maintains the radio at the receiver routine after the end of the transmission
    return
    
def set_bit_rate (msb_rate, lsb_rate, msb_fda, lsb_fda):
    sbuff=[0]*64
    sbuff[0] = 0x7e
    sbuff[1] = 0x8c
    sbuff[2] = 0x8a
    sbuff[3] = 0x92
    sbuff[4] = 0x98
    sbuff[5] = 0x86
    sbuff[6] = 0xa0
    sbuff[7] = 0xe0
    sbuff[8] = 0x8c
    sbuff[9] = 0x8a
    sbuff[10] = 0x92
    sbuff[11] = 0x84
    sbuff[12] = 0xa6
    sbuff[13] = 0x62
    sbuff[14] = 0x61
    ns = random.getrandbits(3) #radom integer of 3 bits
    nr = (ns+1)&0x07 
    sbuff[15] = (nr<<5) | (ns<<1) | 0x10
    sbuff[16] = 0xf0
    sbuff[17] = 0xa7
    
    sbuff[18] = msb_rate
    sbuff[19] = lsb_rate
    sbuff[20] = msb_fda
    sbuff[21] = lsb_fda
    
    checksum=0
    for i in range (1, 17):
        checksum+=sbuff[i]
        
    sbuff[61] = (~checksum)+1
    checksum=0
    for i in range (17, 61):
        checksum+=sbuff[i]
    
    sbuff[62] = (~checksum)+1
    sbuff[63] = 0x7e
    #send the packet to the FIFO
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    radio_tx_cfg()
    for i in range (0, 64):
        radio_write(0, sbuff[i])
    
    while (GPIO.input(11)==0):
        time.sleep(0.00001)
    
    bit_rate = 32000/((msb_rate)*255 + (lsb_rate))
    print ("INFORMATION FRAME SENT: SET BIT RATE TO:", "%.2f" % bit_rate, "kbps", "| FDA: ", 61*((msb_fda)*255 + (lsb_fda)), "Hz")
    print ("SEQUENCE NUMBER N(R):", hex(nr))
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    print ("WAITING FOR REPLY...")
    
    base(4) ## wait
    radio_write(0x0, 0x0)
    
    time.sleep(0.001)
    radio_write(0x02, msb_rate)
    radio_write(0x03, lsb_rate)
    radio_write(0x04, msb_fda)
    radio_write(0x05, lsb_fda)
    
    base(0) ## this maintains the radio at the receiver routine after the end of the transmission
    return


def change_battery (code):
    sbuff=[0]*64
    sbuff[0] = 0x7e
    sbuff[1] = 0x8c
    sbuff[2] = 0x8a
    sbuff[3] = 0x92
    sbuff[4] = 0x98
    sbuff[5] = 0x86
    sbuff[6] = 0xa0
    sbuff[7] = 0xe0
    sbuff[8] = 0x8c
    sbuff[9] = 0x8a
    sbuff[10] = 0x92
    sbuff[11] = 0x84
    sbuff[12] = 0xa6
    sbuff[13] = 0x62
    sbuff[14] = 0x61
    ns = random.getrandbits(3) #radom integer of 3 bits
    nr = (ns+1)&0x07 
    sbuff[15] = (nr<<5) | (ns<<1) | 0x10
    sbuff[16] = 0xf0
    sbuff[17] = 0xa8
    
    sbuff[18] = code #0x00 for only reading / 0xf0 to switch to main / 0xff to switch to backup
    
    checksum=0
    for i in range (1, 17):
        checksum+=sbuff[i]
        
    sbuff[61] = (~checksum)+1
    checksum=0
    for i in range (17, 61):
        checksum+=sbuff[i]
    
    sbuff[62] = (~checksum)+1
    sbuff[63] = 0x7e
    #send the packet to the FIFO
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    radio_tx_cfg()
    for i in range (0, 64):
        radio_write(0, sbuff[i])
    
    while (GPIO.input(11)==0):
        time.sleep(0.00001)
    
    print ("INFORMATION FRAME SENT: CHANGE BATTERY CIRCUIT")
    print ("SEQUENCE NUMBER N(R):", hex(nr))
    reg_value = radio_read(0x3f)
    radio_write(0x3f, reg_value | 0x10) #reset FIFO
    print ("WAITING FOR REPLY...")
    
    base(4) ## wait
    base(0) ## this maintains the radio at the receiver routine after the end of the transmission
    return























