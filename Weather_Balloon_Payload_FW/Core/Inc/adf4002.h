#ifndef _ADF4002_H_
#define _ADF4002_H_

#include "stm32f3xx_hal.h"
#include <stdbool.h>

#define R_COUNTER_LATCH_IND     0
#define N_COUNTER_LATCH_IND     1
#define FUNCTION_LATCH_IND      2
#define INIT_LATCH_IND          3

/* FUNCTION LATCH DEFS */
#define MUXOUT_LOCK_DET         (1 << 4)
#define MUXOUT_N_DIV_OUTPUT     (2 << 4)
#define MUXOUT_DVDD             (3 << 4)
#define MUXOUT_R_DIV_OUTPUT     (4 << 4)
#define MUXOUT_OPEN_DRAIN_LOCK_DET (5 << 4)
#define MUXOUT_SER_DATA_OUT     (6 << 4)
#define MUXOUT_DGND             (7 << 4)
#define PHASE_DET_POLARITY_BIT  (7)
#define PHASE_DET_POLARITY_POS  (1 << PHASE_DET_POLARITY_BIT)
#define CHG_PUMP_OUTPUT_BIT     (8)
#define CHG_PUMP_TRI_STATE      (1 << CHG_PUMP_OUTPUT_BIT)

#define R_COUNTER_INIT_VAL      (0 | (1 << 2) | (1 << 17))
#define N_COUNTER_INIT_VAL      (1 | (1 << 8))
#define FUNCTION_LATCH_INIT_VAL (2 | (1 << 7) | (4 << 4) | (7 << 15) | (7 << 18))
#define INIT_LATCH_INIT_VAL     (3 | (1 << 4))
#define PAUSE_BRIEF()           __ASM("NOP\n NOP\n NOP\n NOP\n")

typedef struct adf4002_struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef * le_port;
    uint16_t le_pin;
    GPIO_TypeDef * ce_port;
    uint16_t ce_pin;
    uint32_t latches[4];
}ADF4002_t;

void adf_init_mem(ADF4002_t* adf);
void adf_init_chip(ADF4002_t* adf);
void adf_enable_chip(ADF4002_t* adf);
void adf_disable_chip(ADF4002_t* adf);
void adf_write_latch(ADF4002_t* adf, int latch_ind);
void adf_write_all_latches(ADF4002_t* adf);
void adf_write_all_latches_w_reset(ADF4002_t* adf);
void adf_set_n_counter(ADF4002_t* adf, uint16_t value, bool update);
void adf_set_r_counter(ADF4002_t* adf, uint16_t value, bool update);

#endif