#include "adf4002.h"


void adf_init_mem(ADF4002_t* adf){
    adf->latches[R_COUNTER_LATCH_IND] = R_COUNTER_INIT_VAL;
    adf->latches[N_COUNTER_LATCH_IND] = N_COUNTER_INIT_VAL;
    adf->latches[FUNCTION_LATCH_IND] = FUNCTION_LATCH_INIT_VAL;
    adf->latches[INIT_LATCH_IND] = INIT_LATCH_INIT_VAL;
}

void adf_init_chip(ADF4002_t* adf){
    adf_disable_chip(adf);
    HAL_GPIO_WritePin(adf->le_port, adf->le_pin, 1);
    
    PAUSE_BRIEF();
    adf_write_latch(adf, FUNCTION_LATCH_IND);
    PAUSE_BRIEF();
    adf_write_latch(adf, R_COUNTER_LATCH_IND);
    PAUSE_BRIEF();
    adf_write_latch(adf, N_COUNTER_LATCH_IND);
    PAUSE_BRIEF();
    adf_enable_chip(adf);
}

void adf_enable_chip(ADF4002_t* adf){
    HAL_GPIO_WritePin(adf->ce_port, adf->ce_pin, 1);
}
void adf_disable_chip(ADF4002_t* adf){
    HAL_GPIO_WritePin(adf->ce_port, adf->ce_pin, 0);
}

void adf_write_latch(ADF4002_t* adf, int latch_ind){
    uint8_t data[3];
    data[0] = adf->latches[latch_ind] >> 16;
    data[1] = adf->latches[latch_ind] >> 8;
    data[2] = adf->latches[latch_ind] & 0xFF;

    HAL_GPIO_WritePin(adf->le_port, adf->le_pin, 0);
    HAL_SPI_Transmit(adf->hspi, data, 3, 10);
    HAL_GPIO_WritePin(adf->le_port, adf->le_pin, 1);
}

void adf_write_all_latches(ADF4002_t* adf){
    for (int i = 0; i < 3; i++){
        adf_write_latch(adf, i);
        PAUSE_BRIEF();
    }
}

void adf_write_all_latches_w_reset(ADF4002_t* adf){
    adf_write_latch(adf, 0);
    PAUSE_BRIEF();
    adf_write_latch(adf, 1);
    PAUSE_BRIEF();
    adf_write_latch(adf, 3);
    PAUSE_BRIEF();
}

void adf_set_n_counter(ADF4002_t* adf, uint16_t value, bool update){
    adf->latches[N_COUNTER_LATCH_IND] &= ~(8191 << 8);
    adf->latches[N_COUNTER_LATCH_IND] |= (value & 8191) << 8;
    if(update){
        adf_write_latch(adf, N_COUNTER_LATCH_IND);
    }
}

void adf_set_r_counter(ADF4002_t* adf, uint16_t value, bool update){
    adf->latches[R_COUNTER_LATCH_IND] &= ~(16383 << 2);
    adf->latches[R_COUNTER_LATCH_IND] |= (value & 16383) << 2;
    if(update){
        adf_write_latch(adf, R_COUNTER_LATCH_IND);
    }
}