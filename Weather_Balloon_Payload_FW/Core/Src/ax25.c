#include "ax25.h"

#include <string.h>
#include <stdlib.h>


void ax25_alloc_frame(Ax25Frame_t* frame, uint16_t payload_size){
    frame = (Ax25Frame_t*)malloc(sizeof(Ax25Frame_t));
    frame->payload = (uint8_t*)malloc(payload_size);
    frame->payload_size = payload_size;
}


void ax25_free_frame(Ax25Frame_t* frame){
    free(frame->payload);
    free(frame);
}

/**
 * Serailizes the frame struct into the packet array.
 * Returns length of the resulting array
*/
int ax25_serialize_frame(Ax25Frame_t* frame, uint8_t* packet){
    int index = 0;
    packet[index] = AX25_FLAG;
    index++;

    // Destination address (7 bytes)
    memcpy(packet + index, frame->dest_addr, 7);
    index += 7;

    // Source address (7 bytes)
    memcpy(packet + index, frame->src_addr, 7);
    index += 7;

    // digipeater address (0 to 56 bytes)
    // for now, do nothing with this

    // Control Field (UI) (1 byte) always 0x03
    packet[index] = frame->control_field;
    index++;

    // Protocol ID (1 byte)
    packet[index] = frame->protocol_id;
    index++;

    // Information Field (1 to 256 bytes)
    memcpy(packet + index, frame->payload, frame->payload_size);
    index += frame->payload_size;

    // FCS (2 bytes)
    // there's like no docs on this that I can find
    packet[index] = 0;
    index++;
    packet[index] = 0;
    index++;

    // End flag 0x7E
    packet[index] = AX25_FLAG;
    index++;
    return index;
}