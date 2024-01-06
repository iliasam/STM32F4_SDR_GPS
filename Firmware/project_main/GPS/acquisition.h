#ifndef _ACQUISITION_H
#define _ACQUISITION_H

#include "stdint.h"


void acquisition_process(gps_ch_t* channel, uint8_t* data);
uint32_t* acquisition_get_hist(void);

void acquisition_freq_test(gps_ch_t* channel, uint8_t* data);
uint8_t* acquisition_get_phase_hist(gps_ch_t* channel);

void acquisition_start_channel(gps_ch_t* channel);
void acquisition_start_code_search_channel(gps_ch_t* channel);
void acquisition_start_code_search3_channel(gps_ch_t* channel);
#endif



