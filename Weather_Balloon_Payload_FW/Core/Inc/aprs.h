#ifndef __APRS_H__
#define __APRS_H__


#include "ax25.h"
#include "ublox.h"
#include "stm32f3xx_hal.h"
#include <stdbool.h>

/* APRS DATA TYPE IDENTIFIERS */
#define APRS_POS_WOUT_TS_NO_MSG '!'
#define APRS_RAW_GPS_DATA       '$'
#define APRS_MESSAGE            ':'
#define APRS_OBJECT             ';'
#define APRS_POS_WITH_TS_NO_MSG '/'
#define APRS_POS_WOUT_TS_MSG    '='
#define APRS_POS_WITH_TS_MSG    '@'
#define APRS_STATUS             '>'
#define APRS_TELEMETRY          'T'
#define APRS_USER_DEFINED       '{'
#define APRS_QUERY              '?'

void aprs_init_frame(Ax25Frame_t* frame);

#endif