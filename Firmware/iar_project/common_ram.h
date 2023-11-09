#ifndef _COMMON_RAM_H
#define _COMMON_RAM_H

#include "stdint.h"
#include "config.h"

extern uint16_t tmp_sin_data[PRN_SPI_WORDS_CNT];
extern uint16_t tmp_cos_data[PRN_SPI_WORDS_CNT];

extern uint16_t tmp_prn_data[PRN_SPI_WORDS_CNT];//data with no modulation
extern uint16_t tmp_data_i[PRN_SPI_WORDS_CNT];
extern uint16_t tmp_data_q[PRN_SPI_WORDS_CNT];

//Tmp correlation results
extern  uint16_t tmp_corr_i[PRN_SPI_WORDS_CNT];
extern uint16_t tmp_corr_q[PRN_SPI_WORDS_CNT];



#endif //_COMMON_RAM_H



