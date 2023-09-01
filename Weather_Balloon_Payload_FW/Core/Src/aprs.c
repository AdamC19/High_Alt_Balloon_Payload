#include "aprs.h"

#include <string.h>
#include <stdlib.h>


void aprs_init_frame(Ax25Frame_t* frame){
    frame->control_field = AX25_UI_FRAME;
    frame->protocol_id = 0xF0;
}