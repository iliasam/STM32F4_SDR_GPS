#ifndef _COMMON_RAM_H
#define _COMMON_RAM_H

#include "stdint.h"
#include "config.h"

// Increase buffer size 1023+1=1024 to make space for shifting data
// So now it is possible to work 
// with buffers as (GPS_DATA_WORDS_CNT/2=512) 32-bit words
#define GPS_DATA_WORDS_CNT      (PRN_SPI_WORDS_CNT + 1)

extern uint16_t tmp_prn_data[GPS_DATA_WORDS_CNT];//data with no modulation
extern uint16_t tmp_data_i[GPS_DATA_WORDS_CNT];
extern uint16_t tmp_data_q[GPS_DATA_WORDS_CNT];

/*
//Tmp correlation results
extern  uint16_t tmp_corr_i[PRN_SPI_WORDS_CNT];
extern uint16_t tmp_corr_q[PRN_SPI_WORDS_CNT];
*/



#endif //_COMMON_RAM_H



