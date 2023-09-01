#ifndef __UBLOX_H__
#define __UBLOX_H__

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <string.h>

#define GPS_ERR_MSG_SIZE            64
#define MAX_M8C_DEFAULT_I2C_ADDR    0x42
#define UBLOX_RX_BUF_SIZE           512
#define UBLOX_TX_BUF_SIZE           64
#define UBLOX_I2C_CHUNK_SIZE        32
#define UBLOX_LOG_FILENAME          "gps_log.txt"
#define UBX_SYNC_CHAR_1             0xB5
#define UBX_SYNC_CHAR_2             0x62
#define UBX_CLASS_IND               2
#define UBX_ID_IND                  3
#define UBX_LEN_IND                 4
#define UBX_PAYLOAD_IND             6
#define UBX_GET_MSG_SIZE(msg)       ((msg[UBX_LEN_IND] | (msg[UBX_LEN_IND + 1] << 8)) + 8)

/* Message header is 0xB5 0x62. */
#define UBX_MSG_HEADER              ((uint16_t)0xB562)

/* Message class numbers */
#define UBX_NAV_CLASS               0x01
#define UBX_INF_CLASS               0x04
#define UBX_CFG_CLASS               0x06
#define UBX_ACK_CLASS               0x05
#define UBX_LOG_CLASS               0x21
#define UBX_RXM_CLASS               0x02
#define UBX_UPD_CLASS               0x09
#define UBX_MON_CLASS               0x0A
#define UBX_AID_CLASS               0x0B
#define UBX_TIM_CLASS               0x0D
#define UBX_ESF_CLASS               0x10
#define UBX_MGA_CLASS               0x13
#define UBX_SEC_CLASS               0x27
#define UBX_HNR_CLASS               0x28

/* NAV message IDs */
#define UBX_NAV_TIMEUTC_ID          0x21

/* CFG message IDs */
#define UBX_CFG_MSG_ID              0x01
#define UBX_CFG_RATE_ID             0x08

#define UBX_MSGS_MAX                32

typedef struct gnss_timestamp_struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t valid;
}gnss_timestamp_t;

typedef struct gnss_position_struct {
    int32_t lat_degrees;
    int8_t lat_hp;
    int32_t lon_degrees;
    int8_t lon_hp;
    int32_t altitude;
    int8_t alt_hp;
    float lat;
    float lon;
    float alt;
}gnss_position_t;

typedef struct ubx_msg_struct {
    uint8_t class;
    uint8_t id;
    uint16_t length;
    uint8_t* payload;
    bool checksum_ok;
}ubx_msg_t;


typedef struct max_m8c_struct {
    I2C_HandleTypeDef* hi2c;
    UART_HandleTypeDef* huart;
    GPIO_TypeDef * reset_port;
    uint16_t reset_pin;
    uint8_t i2c_addr;
    int rx_buf_pos;
    uint8_t rx_buf[UBLOX_RX_BUF_SIZE];
    uint8_t tx_buf[UBLOX_TX_BUF_SIZE];
    void (*delay_ms)(uint32_t);
    void (*console_print)(char*);
    bool overflow;
    ubx_msg_t* msgs[UBX_MSGS_MAX];
    uint8_t msg_count;
    gnss_timestamp_t ts;
}MaxM8C_t;



void ublox_init(MaxM8C_t* gps, I2C_HandleTypeDef* hi2c, UART_HandleTypeDef* huart, GPIO_TypeDef* rst_port, uint16_t rst_pin);
void ublox_config_i2c(MaxM8C_t* gps);
void ublox_config_gnss(MaxM8C_t* gps);
void ublox_hw_reset(MaxM8C_t* gps);
bool ublox_contains_data_stop(uint8_t* data);
void ublox_add_checksum(MaxM8C_t* gps);
int ublox_bytes_available_i2c(MaxM8C_t* gps);
void ublox_read_all_data_i2c(MaxM8C_t* gps, int start_pos);
void ublox_send_packet_i2c(MaxM8C_t* gps);
void ublox_send_poll_msg_i2c(MaxM8C_t* gps, ubx_msg_t msg);
int ublox_get_message(uint8_t* buffer, ubx_msg_t* msg, int max_len);
void ublox_parse_messages(MaxM8C_t* gps);
void ublox_cleanup_messages(MaxM8C_t* gps);
void ublox_get_gnss_timestamp(uint8_t* payload, gnss_timestamp_t* ts);
void ublox_snprintf_msg(MaxM8C_t* gps, ubx_msg_t* msg, char* buf, int max_len);
void ublox_print_all_messages(MaxM8C_t* gps, void(*print_to_console)(char*));

void ublox_parse_hpposllh(ubx_msg_t* msg, gnss_position_t* pos);

void ublox_enable_inf_messages(MaxM8C_t* gps);
void ublox_enable_utc_time_messages(MaxM8C_t* gps);
void ublox_enable_pvt_messages(MaxM8C_t* gps);
void ublox_enable_nav_messages(MaxM8C_t* gps);
void ublox_poll_message_i2c(MaxM8C_t* gps, uint8_t class, uint8_t id);

#endif