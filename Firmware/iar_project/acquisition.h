#ifndef _ACQUISITION_H
#define _ACQUISITION_H

#include "stdint.h"

void acquisition_process(gps_ch_t* channel, uint8_t* data);
uint32_t* acquisition_get_hist(void);
uint8_t acquisition_need_new_data(gps_ch_t* channel);

#endif



