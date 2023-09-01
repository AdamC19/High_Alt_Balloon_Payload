
#include "printf.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "ublox.h"


/* MESSAGE NAMES */
/* NAV messages, CLASS = 0x01 */
const char* ubx_msg_names_nav_class[] = {
    [0x60] = "AOPSTATUS",
    [0x05] = "ATT",
    [0x22] = "CLOCK",
    [0x36] = "COV",
    [0x31] = "DGPS",
    [0x04] = "DOP",
    [0x3d] = "EELL",
    [0x61] = "EOE",
    [0x39] = "GEOFENCE",
    [0x13] = "HPPOSECEF",
    [0x14] = "HPPOSLLH",
    [0x28] = "NMI",
    [0x09] = "ODO",
    [0x34] = "ORB",
    [0x01] = "POSCEF",
    [0x02] = "POSLLH",
    [0x07] = "PVT",
    [0x3C] = "RELPOSNED",
    [0x10] = "RESETODO",
    [0x35] = "SAT",
    [0x32] = "SBAS",
    [0x42] = "SLAS",
    [0x06] = "SOL",
    [0x03] = "STATUS",
    [0x30] = "SVINFO",
    [0x3B] = "SVIN",
    [0x24] = "TIMEBDS",
    [0x25] = "TIMEGAL",
    [0x23] = "TIMEGLO",
    [0x20] = "TIMEGPS",
    [0x26] = "TIMELS",
    [0x21] = "TIMEUTC",
    [0x11] = "VELECEF",
    [0x12] = "VELNED"
};

/* INF messages, CLASS = 0x04 */
const char* ubx_msg_names_inf_class[] = {
    [0x04] = "DEBUG",
    [0x00] = "ERROR",
    [0x02] = "NOTICE",
    [0x03] = "TEST",
    [0x01] = "WARNING"
};

/* CFG messages, class = 0x06 */
const char* ubx_msg_names_cfg_class[] = {
    [UBX_CFG_MSG_ID] = "MSG",
    [0x02] = "INF",
    [0x47] = "LOGFILTER"
};

/* LOG messages, class 0x21 */
const char* ubx_msg_names_log_class[] = {
    [0x01] = "MSG",
    [0x11] = "BATCH",
    [0x07] = "LOG"
};

/* ACK messages, class 0x05 */
const char* ubx_msg_names_ack_class[] = {
    [0x01] = "ACK",
    [0x00] = "NAK"
};

/* message name arrays indexed by class number */
const char** ubx_msg_names[] = {
    [UBX_NAV_CLASS] = ubx_msg_names_nav_class,
    [UBX_INF_CLASS] = ubx_msg_names_inf_class,
    [UBX_CFG_CLASS] = ubx_msg_names_cfg_class,
    [UBX_LOG_CLASS] = ubx_msg_names_log_class,
    [UBX_ACK_CLASS] = ubx_msg_names_ack_class
};

/* Names of classes by class number */
const char* ubx_msg_class_names[] = {
    [UBX_NAV_CLASS] = "NAV",
    [UBX_INF_CLASS] = "INF",
    [UBX_CFG_CLASS] = "CFG",
    [UBX_RXM_CLASS] = "RXM",
    [UBX_ACK_CLASS] = "ACK",
    [UBX_UPD_CLASS] = "UPD",
    [UBX_MON_CLASS] = "MON",
    [UBX_AID_CLASS] = "AID",
    [UBX_TIM_CLASS] = "TIM",
    [UBX_ESF_CLASS] = "ESF",
    [UBX_MGA_CLASS] = "MGA",
    [UBX_LOG_CLASS] = "LOG",
    [UBX_SEC_CLASS] = "SEC",
    [UBX_HNR_CLASS] = "HNR"
};

/**
 * takes a few parameters and stashes them in the gps struct
*/
void ublox_init(MaxM8C_t* gps, I2C_HandleTypeDef* hi2c, UART_HandleTypeDef* huart, GPIO_TypeDef* rst_port, uint16_t rst_pin){
    gps->hi2c = hi2c;
    gps->huart = huart;
    gps->reset_port = rst_port;
    gps->reset_pin = rst_pin;
    HAL_GPIO_WritePin(gps->reset_port, gps->reset_pin, 1); // ensure we're not holding GPS in reset
    gps->i2c_addr = MAX_M8C_DEFAULT_I2C_ADDR << 1;
}


/**
 * 
*/
void ublox_config_i2c(MaxM8C_t* gps){
    gps->tx_buf[0] = 0xB5;
    gps->tx_buf[1] = 0x62;
    gps->tx_buf[UBX_CLASS_IND] = 0x06;
    gps->tx_buf[UBX_ID_IND] = 0x00;
    gps->tx_buf[UBX_LEN_IND] = 20;
    gps->tx_buf[UBX_LEN_IND + 1] = 0;
    uint8_t* payload = gps->tx_buf + UBX_PAYLOAD_IND;
    memset(payload, 0x00, 20); // ensure payload starts as 0's
    payload[4] = gps->i2c_addr; // default i2c addr
    payload[12] = 1; // in = UBX
    payload[14] = 1; // out = UBX
    ublox_add_checksum(gps);
    ublox_send_packet_i2c(gps);
}


/**
 * 
*/
void ublox_config_gnss(MaxM8C_t* gps){
    gps->tx_buf[0] = 0xB5;
    gps->tx_buf[1] = 0x62;
    gps->tx_buf[UBX_CLASS_IND] = 0x06;
    gps->tx_buf[UBX_ID_IND] = 0x3E;
    uint8_t config_blocks = 6;
    uint16_t payload_len = 4 + (config_blocks*8);
    gps->tx_buf[UBX_LEN_IND] = payload_len & 0xFF;
    gps->tx_buf[UBX_LEN_IND + 1] = (payload_len >> 8) & 0xFF;
    uint8_t* payload = gps->tx_buf + UBX_PAYLOAD_IND;
    memset(payload, 0x00, payload_len); // ensure payload starts as 0's
    payload[1] = 32;
    payload[2] = 32;
    payload[3] = config_blocks; // number of configuration blocks

    int n = 0;

    // GPS
    payload[4 + n*8] = 0; // GNSS ID for GPS
    payload[5 + n*8] = 8; // default reserved number of tracking channels
    payload[6 + n*8] = 16; //default max tracking channels
    payload[8 + n*8] = 1; // enable
    payload[10 + n*8] = 0x01; // L1C/A frequency

    // SBAS
    n++;
    payload[4 + n*8] = 1; // GNSS ID for SBAS
    payload[5 + n*8] = 1; // default reserved number of tracking channels
    payload[6 + n*8] = 3; //default max tracking channels
    payload[8 + n*8] = 1; // enable
    payload[10 + n*8] = 0x01; // L1C/A frequency

    // GALILEO 
    n++;
    payload[4 + n*8] = 2; // GNSS ID for GALILEO
    payload[5 + n*8] = 4; // default reserved number of tracking channels
    payload[6 + n*8] = 8; //default max tracking channels
    payload[8 + n*8] = 1; // enable
    payload[10 + n*8] = 0x01; // E1 frequency

    // BEIDOU (disabled)
    n++;
    payload[4 + n*8] = 3; // GNSS ID for BEIDOU
    payload[5 + n*8] = 0; // default reserved number of tracking channels
    payload[6 + n*8] = 8; //default max tracking channels
    payload[8 + n*8] = 0; // disabled
    payload[10 + n*8] = 0x01; // L1 frequency

    // QZSS
    n++;
    payload[4 + n*8] = 5; // GNSS ID for QZSS
    payload[5 + n*8] = 1; // default reserved number of tracking channels
    payload[6 + n*8] = 3; //default max tracking channels
    payload[8 + n*8] = 1; // enable
    payload[10 + n*8] = 0x01; // L1C/A frequency

    // GLONASS (disabled)
    n++;
    payload[4 + n*8] = 6; // GNSS ID for GLONASS
    payload[5 + n*8] = 4; // default reserved number of tracking channels
    payload[6 + n*8] = 8; //default max tracking channels
    payload[8 + n*8] = 0; // disabled
    payload[10 + n*8] = 0x01; // L1 frequency

    ublox_add_checksum(gps);
    ublox_send_packet_i2c(gps);
}


/**
 * Pulses reset line LOW to perform a hardware reset
*/
void ublox_hw_reset(MaxM8C_t* gps){
    HAL_GPIO_WritePin(gps->reset_port, gps->reset_pin, 0);
    gps->delay_ms(1);
    HAL_GPIO_WritePin(gps->reset_port, gps->reset_pin, 1);
}


/**
 * Used to check if the data stream contains 0xFF, indicating
 * no more data to read
*/
bool ublox_contains_data_stop(uint8_t* data){
    for(int i = 0; i < UBLOX_I2C_CHUNK_SIZE; i++){
        if(data[i] == 0xFF){
            return true;
        }
    }
    return false;
}


/**
 * returns CK_A and CK_B as a uint16_t.
 * CK_A is the low-order address, CK_B is the high-order address
*/
uint16_t ublox_compute_checksum(uint8_t* buffer, int total_len){
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for(int i = 0; i < total_len; i++){
        ck_a = ck_a + buffer[i];
        ck_b = ck_b + ck_a;
    }
    return (ck_b << 8) | ck_a;
}


/**
 * Adds the 8-bit Fletcher Algorithm checksum at the end of the
 * gps->tx_buf packet. Packet must be otherwise constructed correctly.
*/
void ublox_add_checksum(MaxM8C_t* gps){
    int payload_len = gps->tx_buf[UBX_LEN_IND] | (gps->tx_buf[UBX_LEN_IND + 1] << 8);
    int total_len = payload_len + 4;
    uint8_t* buffer = gps->tx_buf + UBX_CLASS_IND;
    uint16_t cksum = ublox_compute_checksum(buffer, total_len);

    buffer[total_len] = cksum & 0xFF;
    buffer[total_len + 1] = (cksum >> 8) & 0xFF;
}


/**
 * Computes the checksum of the packet and compares it to the one included 
 * in the received packet. Returns TRUE if the two match, false otherwise.
 * packet should include SYNC characters and everything.
*/
bool ublox_verify_packet_checksum(uint8_t* packet){
    int check_len = (packet[UBX_LEN_IND] | (packet[UBX_LEN_IND + 1] << 8)) + 4;
    uint16_t rcvd_cksum = packet[check_len + 2] | (packet[check_len + 3] << 8);
    uint16_t computed_cksum = ublox_compute_checksum(packet + UBX_CLASS_IND, check_len);
    return (rcvd_cksum == computed_cksum);
}


/**
 * Reads from the GPS receiver how many data bytes are available to read
*/
int ublox_bytes_available_i2c(MaxM8C_t* gps){
    uint8_t tmp = 0xFD;
    uint8_t buf[2] = {0, 0};
    HAL_I2C_Mem_Read(gps->hi2c, gps->i2c_addr, 0xFD, 1, buf, 2, 25);
    // if(HAL_I2C_Master_Transmit(gps->hi2c, gps->i2c_addr, &tmp, 1, 10) != HAL_OK){
    //     return -1;
    // }
    // if(HAL_I2C_Master_Receive(gps->hi2c, gps->i2c_addr, buf, 2, 10) != HAL_OK){
    //     return -1;
    // }
    return (buf[0] << 8) | buf[1];
}


/**
 * Read all currently available data over the I2C interface,
 * storing the bytes in gps->rx_buf. gps->rx_buf_pos is updated
 * and will reflect the number of bytes read
*/
void ublox_read_all_data_i2c(MaxM8C_t* gps, int start_pos){
    gps->rx_buf_pos = start_pos;

    int readlen = ublox_bytes_available_i2c(gps);
    if(readlen > UBLOX_RX_BUF_SIZE - gps->rx_buf_pos){
        readlen = UBLOX_RX_BUF_SIZE - gps->rx_buf_pos;
    }

    // address the 0xFF position
    // first transaction sends register address to ensure we're pointing to 0xFF
    // S SLAVE_ADDR W | 0xFF | S SLAVE_ADDR R | DATA_0 | ... | DATA_N NAK STOP

    // send 0xFF to ensure we're reading the right stuff
    // likely unnecessary, but it's a good idea in general to be extra sure
    uint8_t tmp = 0xFF;
    // HAL_I2C_Master_Transmit(gps->hi2c, gps->i2c_addr, &tmp, 1, 10); 
    // HAL_I2C_Master_Receive(gps->hi2c, gps->i2c_addr, gps->rx_buf + gps->rx_buf_pos, readlen, 50);
    int readsize = 0;
    while(readlen > 0){
        if (readlen - UBLOX_I2C_CHUNK_SIZE < 0){
            readsize = readlen;
        }else{
            readsize = UBLOX_I2C_CHUNK_SIZE;
        }
        HAL_I2C_Master_Receive(gps->hi2c, gps->i2c_addr, gps->rx_buf + gps->rx_buf_pos, readsize, 50);
        gps->rx_buf_pos += readsize;
        readlen -= readsize;
    }

}


/**
 * Send whatever packet is in the gps->tx_buf
*/
void ublox_send_packet_i2c(MaxM8C_t* gps){
    int send_size = UBX_GET_MSG_SIZE(gps->tx_buf);
    HAL_I2C_Master_Transmit(gps->hi2c, gps->i2c_addr, gps->tx_buf, send_size, 50);
}


/**
 * 
*/
void ublox_send_poll_msg_i2c(MaxM8C_t* gps, ubx_msg_t msg){
    
}


/**
 * Parses a message stored in buffer and populates the fields of msg.
 * Notably, msg->payload is a malloc'd buffer that will hold the raw
 * bytes of the original message. ublox_cleanup_messages() frees this.
 * @returns the total number of bytes the message took up, or -1 if no 
 * message is found in buffer
*/
int ublox_get_message(uint8_t* buffer, ubx_msg_t* msg, int max_len){
    bool found_hddr = false;
    int i = 1;
    while(!found_hddr && i < max_len){
        found_hddr = (buffer[i - 1] == 0xB5) && (buffer[i] == 0x62);
        i++;
    }

    // we didn't find a packet before hitting the end of the line
    if(!found_hddr && i >= max_len){
        return -1;
    }
    // TODO: this length checking can be further improved upon

    // i now points to the class byte
    msg->checksum_ok = ublox_verify_packet_checksum(buffer + (i - 2));
    msg->class = buffer[i];
    i++;
    msg->id = buffer[i];
    i++;

    // msg->description = ubx_msg_names[msg->class][msg->id];

    msg->length = buffer[i];
    i++;
    msg->length |= (buffer[i] << 8);
    i++;
    if(msg->class == 0x04){
        // ensure this ASCII string is NULL terminated
        msg->payload = (uint8_t*)malloc(msg->length + 1);
        msg->payload[msg->length] = '\0';
    }else{
        msg->payload = (uint8_t*)malloc(msg->length);
    }
    
    memcpy(msg->payload, buffer + i, msg->length);
    i += msg->length + 2; // 2 includes the checksum

    return i;
}


/**
 * Parses data in gps->rx_buf, and mallocs message structs.
 * Places the pointers to the structs in the gps->msgs array.
 * Be sure to run ublox_cleanup_messages() at some point!
*/
void ublox_parse_messages(MaxM8C_t* gps){
    int offset = 0;
    while(gps->msg_count < UBX_MSGS_MAX && offset < UBLOX_RX_BUF_SIZE){
        gps->msgs[gps->msg_count] = (ubx_msg_t*)malloc(sizeof(ubx_msg_t));
        int inc = ublox_get_message(gps->rx_buf + offset, gps->msgs[gps->msg_count], UBLOX_RX_BUF_SIZE - offset);
        if(inc <= 0){
            // we didn't find a message in the last pass
            free(gps->msgs[gps->msg_count]);
            break;
        }
        offset += inc;
        // gps->msgs[gps->msg_count] = msg;
        gps->msg_count++;
    }

    // ensure even if we read beyond the end of the allotted buffer,
    // nothing will be broken (hopefully)
    if(offset > UBLOX_RX_BUF_SIZE){
        offset = UBLOX_RX_BUF_SIZE;
    }
    memset(gps->rx_buf, 0x00, offset);
    gps->rx_buf_pos = 0;
}


/**
 * free()'s memory that was malloc'd in ublox_parse_messages
*/
void ublox_cleanup_messages(MaxM8C_t* gps){
    for(int i = gps->msg_count - 1; i >= 0; i--){
        free(gps->msgs[i]->payload);
        // gps->msgs[i]->payload = NULL;
        free(gps->msgs[i]);
        // gps->msgs[i] = NULL;
    }
    gps->msg_count = 0;
}


/**
 * Parse the payload bytes and populate the ts object with usable numbers
*/
void ublox_get_gnss_timestamp(uint8_t* payload, gnss_timestamp_t* ts){
    ts->year   = payload[12] | (payload[13] << 8);
    ts->month   = payload[14];
    ts->day     = payload[15];
    ts->hour    = payload[16];
    ts->minute  = payload[17];
    ts->second  = payload[18];
    ts->valid   = payload[19];
}


/**
 * The message, if interesting, is parsed and snprintf'd into buf.
 * There's a lot of messages, so not all are covered here. If the message 
 * doesn't have special code written for it, the bytes are simply printed 
 * in hexadecimal format.
*/
void ublox_snprintf_msg(MaxM8C_t* gps, ubx_msg_t* msg, char* buf, int max_len){
    int offset = 0; //snprintf(buf, max_len, "TEST");

    // print the UBX-CLASS-ID string
    if(ubx_msg_class_names[msg->class] != NULL){
        if(ubx_msg_names[msg->class][msg->id] != NULL){
            offset = snprintf(buf, max_len, "(UBX-%s-%s) ", ubx_msg_class_names[msg->class], ubx_msg_names[msg->class][msg->id]);
        }else{
            offset = snprintf(buf, max_len, "(UBX-%s-0x%02X) ", ubx_msg_class_names[msg->class], msg->id);
        }
    }else{
        offset = snprintf(buf, max_len, "(UBX-0x%02X-0x%02X) ", msg->class, msg->id);
        // offset = snprintf(buf, max_len, "(%s) ", "UBX-UNKNOWN");
    }
    buf += offset;
    max_len -= offset;
    offset = 0;
    bool handled = false;

    if(msg->class == UBX_INF_CLASS){
        // these are the UBX-INF class messages
        // they are meant to be printf'd directly
        snprintf(buf, max_len, "%s", msg->payload);
        handled = true;
    }else if(msg->class == UBX_NAV_CLASS){
        // this is the UBX-NAV class

        if(msg->id == 0x21){
            // UBX-NAV-TIMEUTC
            // printf in ISO date time format
            // yyyy-MM-ddTHH:mm:ssZ
            gnss_timestamp_t* ts = &(gps->ts);
            ublox_get_gnss_timestamp(msg->payload, ts);
            snprintf(buf, max_len, "%4d-%02d-%02dT%02d:%02d:%02dZ", ts->year, ts->month, ts->day, ts->hour, ts->minute, ts->second);
            handled = true;
        }
        else if(msg->id == 0x07){
            // UBX-NAV-PVT
            // TODO do things here
            
            snprintf(buf, max_len, "PVT message. VALID=%02X, FLAGS=%02X, FLAGS2=%02X", msg->payload[11] & 0x0F, msg->payload[21], msg->payload[22] & 0xE0);
            handled = true;
        }
        else if(msg->id == 0x35){
            // UBX-NAV-SAT
            int num_svs = msg->payload[5];
            int buffer_offset = 0;
            for(int n = 0; n < num_svs; n++){
                buffer_offset += snprintf(  buf + buffer_offset, max_len - buffer_offset, 
                                            "%d, %d: CN0=%d Q=%d | ", 
                                            msg->payload[8 + 12*n],
                                             msg->payload[9 + 12*n],
                                             msg->payload[10 + 12*n],
                                             msg->payload[16 + 12*n] & 0x07);
            }
            handled = true;
        }
    }else if(msg->id == UBX_LOG_CLASS){
        
    }

    if(!handled){
        // Just print the bytes of the payload in hex
        for(int i = 0; i < msg->length; i++){
            int app_len = snprintf(buf, max_len, "%02X ", msg->payload[i]);
            max_len -= app_len;
            offset += app_len;
            if(max_len <= 0){
                break;
            }
        }

        // Print checksum status, because if the command was unrecognized,
        // we might surmise that is was due to a bad packet
        if(msg->checksum_ok){
            snprintf(buf + offset, max_len, "(%s)", "CKSUM OK");
        }else{
            snprintf(buf + offset, max_len, "(%s)", "CKSUM ERR");
        }
        
    }
}


/**
 * Iterate through all messages in gps->msgs and print each one using the provided
 * print_to_console function.
*/
void ublox_print_all_messages(MaxM8C_t* gps, void(*print_to_console)(char*)){
    char tmp[256];
    // snprintf(buf, 128, "GPS MSG COUNT: %d", gps->msg_count);
    for(int i = 0; i < gps->msg_count; i++){
        ublox_snprintf_msg(gps, gps->msgs[i], tmp, 256);
        print_to_console(tmp);
        // gps->console_print(tmp);
        gps->delay_ms(4); // some delay is necessary to let the UART send out data
    }
}


void ublox_parse_hpposllh(ubx_msg_t* msg, gnss_position_t* pos){
    gnss_position_t pos;
    memcpy(&(pos->lon_degrees), msg->payload + 8, 4);
    memcpy(&(pos->lat_degrees), msg->payload + 12, 4);
    memcpy(&(pos->altitude), msg->payload + 20, 4);
    memcpy(&(pos->lon_hp), msg->payload + 24, 1);
    memcpy(&(pos->lat_hp), msg->payload + 25, 1);
    memcpy(&(pos->alt_hp), msg->payload + 27, 1);

    pos->lon = (1.0*pos->lon_degrees)*1.0e-7 + (1.0*pos->lon_hp)*1.0e-9;
    pos->lat = (1.0*pos->lat_degrees)*1.0e-7 + (1.0*pos->lat_hp)*1.0e-9;
    pos->alt = (1.0*pos->altitude)*1.0e-3;
}


/**
 * Enable ASCII debugging messages to be output over I2C.
*/
void ublox_enable_inf_messages(MaxM8C_t* gps){
    gps->tx_buf[0] = 0xB5;
    gps->tx_buf[1] = 0x62;
    gps->tx_buf[UBX_CLASS_IND] = 0x06;
    gps->tx_buf[UBX_ID_IND] = 0x02;
    gps->tx_buf[UBX_LEN_IND] = 10;
    gps->tx_buf[UBX_LEN_IND + 1] = 0;
    uint8_t* payload = gps->tx_buf + UBX_PAYLOAD_IND;
    memset(payload, 0x00, 10); // ensure payload starts as 0's
    payload[4] = 0x1F; // all INF messages enabled on I2C (port 0)
    ublox_add_checksum(gps);
    ublox_send_packet_i2c(gps);
}


/**
 * Enable the periodic output of UTC time messages on the current port (I2C). 
*/
void ublox_enable_utc_time_messages(MaxM8C_t* gps){
    gps->tx_buf[0] = 0xB5;
    gps->tx_buf[1] = 0x62;
    gps->tx_buf[UBX_CLASS_IND] = UBX_CFG_CLASS;
    gps->tx_buf[UBX_ID_IND] = UBX_CFG_MSG_ID;
    gps->tx_buf[UBX_LEN_IND] = 3;
    gps->tx_buf[UBX_LEN_IND + 1] = 0;
    uint8_t* payload = gps->tx_buf + UBX_PAYLOAD_IND;
    memset(payload, 0x00, 3); // ensure payload starts as 0's
    payload[0] = UBX_NAV_CLASS;
    payload[1] = UBX_NAV_TIMEUTC_ID;
    payload[2] = 1; // on every time solution, send time message on I2C
    ublox_add_checksum(gps);
    ublox_send_packet_i2c(gps);
}


/**
 * 
*/
void ublox_enable_pvt_messages(MaxM8C_t* gps){
    gps->tx_buf[0] = 0xB5;
    gps->tx_buf[1] = 0x62;
    gps->tx_buf[UBX_CLASS_IND] = UBX_CFG_CLASS;
    gps->tx_buf[UBX_ID_IND] = UBX_CFG_MSG_ID;
    gps->tx_buf[UBX_LEN_IND] = 3;
    gps->tx_buf[UBX_LEN_IND + 1] = 0;
    uint8_t* payload = gps->tx_buf + UBX_PAYLOAD_IND;
    memset(payload, 0x00, 3); // ensure payload starts as 0's
    payload[0] = UBX_NAV_CLASS;
    payload[1] = 0x07; // UBX-NAV-PVT
    payload[2] = 1; // on every solution, send message on I2C
    ublox_add_checksum(gps);
    ublox_send_packet_i2c(gps);
}


void ublox_enable_nav_messages(MaxM8C_t* gps){
    gps->tx_buf[0] = 0xB5;
    gps->tx_buf[1] = 0x62;
    gps->tx_buf[UBX_CLASS_IND] = UBX_CFG_CLASS;
    gps->tx_buf[UBX_ID_IND] = UBX_CFG_MSG_ID;
    gps->tx_buf[UBX_LEN_IND] = 3;
    gps->tx_buf[UBX_LEN_IND + 1] = 0;
    uint8_t* payload = gps->tx_buf + UBX_PAYLOAD_IND;
    memset(payload, 0x00, 3); // ensure payload starts as 0's
    payload[0] = UBX_NAV_CLASS;
    payload[1] = 0x14; // UBX-NAV-HPPOSLLH
    payload[2] = 1; // on every time solution, send time message on I2C
    ublox_add_checksum(gps);
    ublox_send_packet_i2c(gps);
}


void ublox_poll_message_i2c(MaxM8C_t* gps, uint8_t class, uint8_t id){
    gps->tx_buf[0] = 0xB5;
    gps->tx_buf[1] = 0x62;
    gps->tx_buf[UBX_CLASS_IND] = class;
    gps->tx_buf[UBX_ID_IND] = id;
    gps->tx_buf[UBX_LEN_IND] = 0;
    gps->tx_buf[UBX_LEN_IND + 1] = 0;
    ublox_add_checksum(gps);
    ublox_send_packet_i2c(gps);
}