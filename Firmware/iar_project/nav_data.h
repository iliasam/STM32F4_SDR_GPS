#ifndef _GPS_NAV_DATA_H
#define _GPS_NAV_DATA_H

#include "stdint.h"
#include "gps_misc.h"

void gps_nav_data_analyse_new_code(gps_ch_t* channel, uint8_t index, int16_t new_i);

#endif



