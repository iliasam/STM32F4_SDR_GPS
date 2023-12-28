#ifndef _ACQUISITION_H
#define _ACQUISITION_H

#include "stdint.h"

//1 step is 0.5 of chip
#define ACQ_PHASE1_HIST_STEP		(64)
#define ACQ_PHASE1_HIST_SIZE		((PRN_LENGTH + 1) * 2 / ACQ_PHASE1_HIST_STEP) //32

#define ACQ_COUNT		        (ACQ_SEARCH_FREQ_HZ * 2 / ACQ_SEARCH_STEP_HZ + 1)


void acquisition_process(gps_ch_t* channel, uint8_t* data);
uint32_t* acquisition_get_hist(void);

void acquisition_freq_test(gps_ch_t* channel, uint8_t* data);
uint8_t* acquisition_get_phase_hist(void);

#endif



