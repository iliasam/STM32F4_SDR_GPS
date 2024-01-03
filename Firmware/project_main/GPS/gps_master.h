#ifndef _GPS_MASTER_H
#define _GPS_MASTER_H

#include "stdint.h"
#include "gps_misc.h"

void gps_master_handling(gps_ch_t* channels, uint8_t index);
uint8_t gps_master_need_acq(void);
uint8_t gps_master_need_freq_search(gps_ch_t* channels);

uint8_t gps_master_is_code_search3(gps_ch_t* channels);

uint8_t gps_master_need_acq(void);

#endif



