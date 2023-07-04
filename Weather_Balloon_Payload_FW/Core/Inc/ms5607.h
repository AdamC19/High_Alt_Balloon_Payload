#ifndef __MS5607_H__
#define __MS5607_H__


#include "stm32f3xx_hal.h"
#include <stdbool.h>

#define MS5607_RESET_CMD    0x1E


typedef struct ms5607_struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t i2c_addr;
}MS5607_t;


#endif