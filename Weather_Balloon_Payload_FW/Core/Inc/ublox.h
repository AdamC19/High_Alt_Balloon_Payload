#ifndef __UBLOX_H__
#define __UBLOX_H__

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <string.h>

#define MAX_M8C_DEFAULT_I2C_ADDR    0x42
#define UBLOX_RX_BUF_SIZE           256
#define UBLOX_I2C_CHUNK_SIZE        4
#define UBLOX_LOG_FILENAME          "gps_log.txt"

typedef struct max_m8c_struct {
    I2C_HandleTypeDef* hi2c;
    UART_HandleTypeDef* huart;
    uint8_t i2c_addr;
    int rx_buf_pos;
    uint8_t rx_buf[UBLOX_RX_BUF_SIZE];
    void (*delay_ms)(int);
    bool overflow;
}MaxM8C_t;

void ublox_init(MaxM8C_t* gps, I2C_HandleTypeDef* hi2c, UART_HandleTypeDef* huart);
bool ublox_contains_data_stop(uint8_t* data);
void ublox_read_all_data_i2c(MaxM8C_t* gps, int start_pos);
bool ublox_get_next_byte_i2c(MaxM8C_t* gps);

#endif