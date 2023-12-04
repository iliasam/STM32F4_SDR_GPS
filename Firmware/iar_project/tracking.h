#ifndef _GPS_TRACKING_H
#define _GPS_TRACKING_H

#include "stdint.h"

void gps_tracking_process(gps_ch_t* channel, uint8_t* data, uint8_t index);

#endif



