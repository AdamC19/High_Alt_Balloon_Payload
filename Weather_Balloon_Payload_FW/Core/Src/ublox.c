#include "ublox.h"
#include <stdbool.h>
#include <string.h>

void ublox_init(MaxM8C_t* gps, I2C_HandleTypeDef* hi2c, UART_HandleTypeDef* huart){
    gps->hi2c = hi2c;
    gps->huart = huart;
    gps->i2c_addr = MAX_M8C_DEFAULT_I2C_ADDR;
}

bool ublox_contains_data_stop(uint8_t* data){
    for(int i = 0; i < UBLOX_I2C_CHUNK_SIZE; i++){
        if(data[i] == 0xFF){
            return true;
        }
    }
    return false;
}

void ublox_read_all_data_i2c(MaxM8C_t* gps, int start_pos){
    gps->rx_buf_pos = start_pos;

    // i2c_addr is already left shifted
    // address the 0xFF position
    // if dat comes back != 0xFF: this is real data, store and keep going
    // else: no more data, exit
    // first transaction sends register address to ensure we're pointing to 0xFF
    // S SLAVE_ADDR W | 0xFF | S SLAVE_ADDR R | DATA_0 | ... | DATA_N NAK STOP
    HAL_I2C_Mem_Read(gps->hi2c, gps->i2c_addr, 0xFF, 1, gps->rx_buf + gps->rx_buf_pos, UBLOX_I2C_CHUNK_SIZE, 50);

    while(!ublox_contains_data_stop(gps->rx_buf + gps->rx_buf_pos) &&
          gps->rx_buf_pos + UBLOX_I2C_CHUNK_SIZE < UBLOX_RX_BUF_SIZE)
    {
        gps->rx_buf_pos += UBLOX_I2C_CHUNK_SIZE; // jump over all the stuff we just read
        HAL_I2C_Master_Receive(gps->hi2c, gps->i2c_addr, gps->rx_buf + gps->rx_buf_pos, UBLOX_I2C_CHUNK_SIZE, 50);
    }

    if(ublox_contains_data_stop(gps->rx_buf + gps->rx_buf_pos)){

    }

}

bool ublox_get_next_byte_i2c(MaxM8C_t* gps){
    bool retval = false;

    uint8_t dat = 0;
    // i2c_addr is already left shifted
    HAL_I2C_Mem_Read(gps->hi2c, gps->i2c_addr, 0xFF, 1, &dat, 1, 50);

    return retval;
}