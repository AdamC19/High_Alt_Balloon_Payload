#ifndef __AX_25_H__
#define __AX_25_H__


#include "stm32f3xx_hal.h"
#include <stdbool.h>

#define AX25_FLAG           0x7E
#define AX25_UI_FRAME       0x03
#define AX25_FCS_POLYNOM    0x1021

typedef struct ax25_frame_struct {
    uint8_t dest_addr[7];
    uint8_t src_addr[7];
    uint8_t control_field;
    uint8_t protocol_id;
    uint8_t* payload;
    uint16_t payload_size;
    uint8_t fcs[2];
}Ax25Frame_t;

void ax25_alloc_frame(Ax25Frame_t* frame, uint16_t payload_size);
void ax25_free_frame(Ax25Frame_t* frame);
int ax25_serialize_frame(Ax25Frame_t* frame, uint8_t* packet);

#endif