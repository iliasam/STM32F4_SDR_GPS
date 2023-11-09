#include "common_ram.h"

uint16_t tmp_sin_data[PRN_SPI_WORDS_CNT];
uint16_t tmp_cos_data[PRN_SPI_WORDS_CNT];

//Tmp correlation results
uint16_t tmp_corr_i[PRN_SPI_WORDS_CNT];
uint16_t tmp_corr_q[PRN_SPI_WORDS_CNT];

uint16_t tmp_prn_data[PRN_SPI_WORDS_CNT];//data with no modulation
uint16_t tmp_data_i[PRN_SPI_WORDS_CNT];
uint16_t tmp_data_q[PRN_SPI_WORDS_CNT];