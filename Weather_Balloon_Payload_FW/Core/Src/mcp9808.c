#include "mcp9808.h"


/**
 * Reads the temperature from the MCP9808. Returns temperature in 
 * degrees Celcius
*/
float mcp9808_get_temperature(MCP9808_t* mcp){
    uint8_t data[2];
    if(HAL_I2C_Mem_Read(mcp->hi2c, mcp->i2c_addr, MCP9808_AMBIENT_T_REG, 1, data, 2, 10) == HAL_OK){
        mcp->online = true;
        
        int16_t upper_byte = data[0] & 0x1f;
        int16_t lower_byte = data[1];
        if((upper_byte & 0x10) == 0x10){
            // T_A < 0 deg C
            upper_byte = upper_byte & 0x0F; // clear sign bit
            mcp->temp = 256.0 - (16.0*upper_byte + lower_byte/16.0);
        }else{
            mcp->temp = (16.0*upper_byte + lower_byte/16.0);
        }
    }else{
        mcp->online = false;
        mcp->temp = 0.0;
    }
    return mcp->temp;
}