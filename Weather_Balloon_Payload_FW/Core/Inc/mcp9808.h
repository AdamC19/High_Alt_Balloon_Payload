#ifndef __MCP9808_H__
#define __MCP9808_H__


#include "stm32f3xx_hal.h"
#include <stdbool.h>

#define MCP9808_AMBIENT_T_REG   5

typedef struct mcp9808_struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t i2c_addr;
    bool online;
    uint64_t update_ts;
    float temp;
}MCP9808_t;

float mcp9808_get_temperature(MCP9808_t* mcp);

#endif