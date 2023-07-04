#ifndef __UBLOX_H__
#define __UBLOX_H__

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <string.h>

#define GPS_ERR_MSG_SIZE            64
#define MAX_M8C_DEFAULT_I2C_ADDR    0x42
#define UBLOX_RX_BUF_SIZE           256
#define UBLOX_TX_BUF_SIZE           64
#define UBLOX_I2C_CHUNK_SIZE        32
#define UBLOX_LOG_FILENAME          "gps_log.txt"
#define UBX_SYNC_CHAR_1             0xB5
#define UBX_SYNC_CHAR_2             0x62
#define UBX_CLASS_IND               2
#define UBX_ID_IND                  3
#define UBX_LEN_IND                 4
#define UBX_PAYLOAD_IND             6
#define UBX_GET_MSG_SIZE(msg)       (msg[UBX_LEN_IND] + 8)

/* Message header is 0xB5 0x62. */
#define UBX_MSG_HEADER              ((uint16)0xB562)


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

typedef struct ubx_msg_struct {
    char* description;
    uint8_t class;
    uint8_t id;
    uint8_t length;
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
    void (*delay_ms)(int);
    void (*console_print)(char*);
    bool overflow;
    ubx_msg_t* msgs[UBX_MSGS_MAX];
    uint8_t msg_count;
    gnss_timestamp_t ts;
}MaxM8C_t;


/* MESSAGE NAMES */
/* NAV messages, CLASS = 0x01 */
static char* ubx_msg_names_class_1[] = {
    [0x60] = "UBX-NAV-AOPSTATUS",
    [0x05] = "UBX-NAV-ATT",
    [0x22] = "UBX-NAV-CLOCK",
    [0x36] = "UBX-NAV-COV",
    [0x31] = "UBX-NAV-DGPS",
    [0x04] = "UBX-NAV-DOP",
    [0x3d] = "UBX-NAV-EELL",
    [0x61] = "UBX-NAV-EOE",
    [0x39] = "UBX-NAV-GEOFENCE",
    [0x13] = "UBX-NAV-HPPOSECEF",
    [0x14] = "UBX-NAV-HPPOSLLH",
    [0x28] = "UBX-NAV-NMI",
    [0x09] = "UBX-NAV-ODO",
    [0x34] = "UBX-NAV-ORB",
    [0x01] = "UBX-NAV-POSCEF",
    [0x02] = "UBX-NAV-POSLLH",
    [0x07] = "UBX-NAV-PVT",
    [0x3C] = "UBX-NAV-RELPOSNED",
    [0x10] = "UBX-NAV-RESETODO",
    [0x35] = "UBX-NAV-SAT",
    [0x32] = "UBX-NAV-SBAS",
    [0x42] = "UBX-NAV-SLAS",
    [0x06] = "UBX-NAV-SOL",
    [0x03] = "UBX-NAV-STATUS",
    [0x30] = "UBX-NAV-SVINFO",
    [0x3B] = "UBX-NAV-SVIN",
    [0x24] = "UBX-NAV-TIMEBDS",
    [0x25] = "UBX-NAV-TIMEGAL",
    [0x23] = "UBX-NAV-TIMEGLO",
    [0x20] = "UBX-NAV-TIMEGPS",
    [0x26] = "UBX-NAV-TIMELS",
    [0x21] = "UBX-NAV-TIMEUTC",
    [0x11] = "UBX-NAV-VELECEF",
    [0x12] = "UBX-NAV-VELNED"
};

/* INF messages, CLASS = 0x02 */
static char* ubx_msg_names_class_4[] = {
    [0x04] = "UBX-INF-DEBUG",
    [0x00] = "UBX-INF-ERROR",
    [0x02] = "UBX-INF-NOTICE",
    [0x03] = "UBX-INF-TEST",
    [0x01] = "UBX-INF-WARNING"
};

static char** ubx_msg_names[] = {
    [0x01] = ubx_msg_names_class_1,
    [0x04] = ubx_msg_names_class_4
};

void ublox_init(MaxM8C_t* gps, I2C_HandleTypeDef* hi2c, UART_HandleTypeDef* huart, GPIO_TypeDef* rst_port, uint16_t rst_pin);
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
void ublox_snprintf_msg(ubx_msg_t* msg, char* buf, int max_len);
void ublox_print_all_messages(MaxM8C_t* gps, void(*print_to_console)(char*));

#endif