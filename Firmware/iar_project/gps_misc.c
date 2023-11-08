#include "gps_misc.h"
#include "config.h"
#include "stdint.h"
#include <math.h>
#include "delay_us_timer.h"
#include "intrinsics.h"

volatile static uint32_t diff;

uint8_t gps_summ_table[256];

uint8_t gps_summ_table16[65536];

uint16_t tmp_sin_data[PRN_SPI_WORDS_CNT];
uint16_t tmp_cos_data[PRN_SPI_WORDS_CNT];

//Tmp correlation results
uint16_t tmp_corr_i[PRN_SPI_WORDS_CNT];
uint16_t tmp_corr_q[PRN_SPI_WORDS_CNT];

uint8_t gps_summ16(uint16_t data);
uint16_t gps_buff_summ16(uint16_t* data, uint16_t length);

//888888888888888888888888888888888888888

void gps_fill_summ_table(void)
{
  for (uint16_t i = 0; i < 256; i++)
  {
    gps_summ_table[i] = gps_summ8((uint8_t)i);
  }
  
  for (uint32_t i = 0; i < 65536; i++)
  {
    gps_summ_table16[i] = gps_summ16(i);
  }
}

uint8_t gps_summ8(uint8_t data)
{
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (data & 1)
      cnt++;
    data = data >> 1;
  }
  return cnt;
}

uint8_t gps_summ16(uint16_t data)
{
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 16; i++)
  {
    if (data & 1)
      cnt++;
    data = data >> 1;
  }
  return cnt;
}


uint8_t gps_check_bit16(uint16_t* ptr, uint8_t pos)
{
	if (pos > 15)
		return 0;

	if (((*ptr) & (1 << pos)) != 0)
		return 1;
	else
		return 0;
}

void gps_generate_sin(uint16_t* ptr, uint16_t length, float freq_hz)
{
	//SPI is LSB first
	uint32_t total_bit_cnt = 0;
	uint16_t word_cnt = 0;

	/// Period in SPI bits
	double period = (double)SPI_BAUDRATE_HZ / freq_hz;
	double threshold = period / 2;
	if (period < 0.0f)
		return;

	for (word_cnt = 0; word_cnt < length; word_cnt++)
	{
		uint16_t word_value = 0;
		for (uint8_t bit_cnt = 0; bit_cnt < 16; bit_cnt++)
		{
			word_value = word_value >> 1;
			uint32_t div1 = (uint32_t)((double)total_bit_cnt / period);
			double zone = (double)total_bit_cnt - ((double)div1 * period); //=bit_cnt % period
			if (zone < threshold)
				word_value |= 0x8000;
			total_bit_cnt++;
		}
		ptr[word_cnt] = word_value;
	}
}

void gps_generate_cos(uint16_t* ptr, uint16_t length, float freq_hz)
{
	//SPI is LSB first
	uint32_t total_bit_cnt = 0;
	uint16_t word_cnt = 0;

	/// Period in SPI bits
	double period = (double)SPI_BAUDRATE_HZ / freq_hz;
	double threshold = period / 4;
	double threshold2 = period - threshold;
	if (period < 0.0f)
		return;

	for (word_cnt = 0; word_cnt < length; word_cnt++)
	{
		uint16_t word_value = 0;
		for (uint8_t bit_cnt = 0; bit_cnt < 16; bit_cnt++)
		{
			word_value = word_value >> 1;
			uint32_t div1 = (uint32_t)((double)total_bit_cnt / period);
			double zone = (double)total_bit_cnt - ((double)div1 * period); //=bit_cnt % period
			if ((zone < threshold) || (zone >= threshold2))
				word_value |= 0x8000;
			total_bit_cnt++;
		}
		ptr[word_cnt] = word_value;
	}
}

void gps_mult8(uint8_t* src1_p, uint8_t* src2_p, uint8_t* dst_p, uint16_t length, uint16_t offset)
{
  uint8_t tmp_var;
  uint16_t pos = 0;
  for (uint16_t i = 0; i < length; i++)
  {
    pos = offset + i;
    if (pos >= length)
      pos = pos - length;
    tmp_var = src1_p[pos] ^ src2_p[i];
    dst_p[i] = ~tmp_var;
  }
}

///no inversion
void gps_mult8_fast(uint8_t* src1_p, uint8_t* src2_p, uint8_t* dst_p, uint16_t length, uint16_t offset)
{
	uint16_t pos = 0;
	for (uint16_t i = 0; i < length; i++)
	{
		pos = offset + i;
		if (pos >= length)
			pos = pos - length;
		dst_p[i] = src1_p[pos] ^ src2_p[i];
	}
}

void gps_mult_iq8(uint8_t* src_i, uint8_t* src_q, uint8_t* src2, uint8_t* dst_i, uint8_t* dst_q, uint16_t length, uint16_t offset)
{
  uint16_t pos = 0;
  
  for (uint16_t i = 0; i < length; i++)
  {
    pos = i + offset;
    if (pos >= length)
      pos = pos - length;
    dst_i[i] = (src_i[pos] ^ src2[i]);
    dst_q[i] = (src_q[pos] ^ src2[i]);
  }
}

void gps_mult_iq32(uint8_t* src_i, uint8_t* src_q, uint8_t* src2, uint8_t* dst_i, uint8_t* dst_q, uint16_t length, uint16_t offset)
{
  uint32_t* src2_p32 = (uint32_t*)src2;
  
  uint32_t* src_i_p32 = (uint32_t*)(src_i + offset);
  uint32_t* dst_i_p32 = (uint32_t*)dst_i;
  
  uint32_t* src_q_p32 = (uint32_t*)(src_q + offset);
  uint32_t* dst_q_p32 = (uint32_t*)dst_q;
  
  uint16_t length_words_p1 = (length - offset) / 4;
  
  for (uint16_t i = 0; i < length_words_p1; i++)
  {
    dst_i_p32[i] = (*src_i_p32) ^ src2_p32[i];
    src_i_p32++;
    dst_q_p32[i] = (*src_q_p32) ^ src2_p32[i];
    src_q_p32++;
  }
  
  //reset to start of buffer
  src_i_p32 = (uint32_t*)(src_i);
  src_q_p32 = (uint32_t*)(src_q);
  
  for (uint16_t i = length_words_p1; i < (length / 4); i++)
  {
    dst_i_p32[i] = (*src_i_p32) ^ src2_p32[i];
    src_i_p32++;
    dst_q_p32[i] = (*src_q_p32) ^ src2_p32[i];
    src_q_p32++;
  }
}

uint16_t gps_summ0(uint16_t* data, uint16_t length)
{
  uint8_t* tmp_p = (uint8_t*)data;
  uint16_t cnt = 0;
  for (uint16_t byte_cnt = 0; byte_cnt < (length); byte_cnt++)
  {
    cnt += gps_summ_table[*tmp_p];
    tmp_p++;
  }
  
  return cnt;
}

uint16_t gps_buff_summ16(uint16_t* data, uint16_t length)
{
  uint16_t cnt = 0;
  
  uint16_t* end_p = data + (length / 2);
  while (data < end_p)
  {
    cnt += gps_summ_table16[*data];
    data++;
    cnt += gps_summ_table16[*data];
    data++;
    cnt += gps_summ_table16[*data];
    data++;
    cnt += gps_summ_table16[*data];
    data++;
    cnt += gps_summ_table16[*data];
    data++;
    cnt += gps_summ_table16[*data];
    data++;
    cnt += gps_summ_table16[*data];
    data++;
    cnt += gps_summ_table16[*data];
    data++;
  }
  
  return cnt;
}


int16_t gps_correlation_test(
  uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, uint16_t* corr1, uint16_t* corr2, uint16_t offset)
{
  uint32_t start_t = get_dwt_value();
  
  //Correllaton
  gps_mult_iq32((uint8_t*)data_i, (uint8_t*)data_q, (uint8_t*)prn_p, 
               (uint8_t*)corr1, (uint8_t*)corr2, PRN_SPI_WORDS_CNT*2, offset);
  
  //Integrate
  int16_t summ1 = gps_buff_summ16(corr1, PRN_SPI_WORDS_CNT*2) - BITS_IN_PRN / 2;
  int16_t summ2 = gps_buff_summ16(corr2, PRN_SPI_WORDS_CNT*2) - BITS_IN_PRN / 2;
  
  if (summ1 < 0)
    summ1 = 0;
  
  if (summ2 < 0)
    summ2 = 0;
  
  int32_t summ1S = summ1 * summ1;
  int32_t summ2S = summ2 * summ2;

  int16_t corr_res = (int16_t)sqrtf((float)summ1S + (float)summ2S);
  uint32_t stop_t = get_dwt_value();
  
  diff = stop_t - start_t;
  diff = diff / 168;
  
  return corr_res;
}

int16_t gps_correlation8(
  uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, uint16_t* corr1, uint16_t* corr2, uint16_t offset)
{
  //Correllaton
  gps_mult_iq32((uint8_t*)data_i, (uint8_t*)data_q, (uint8_t*)prn_p, 
               (uint8_t*)corr1, (uint8_t*)corr2, PRN_SPI_WORDS_CNT*2, offset);
  
  //Integrate
  int16_t summ1 = gps_buff_summ16(corr1, PRN_SPI_WORDS_CNT*2) - BITS_IN_PRN / 2;
  int16_t summ2 = gps_buff_summ16(corr2, PRN_SPI_WORDS_CNT*2) - BITS_IN_PRN / 2;

  int32_t summ1S = summ1 * summ1;
  int32_t summ2S = summ2 * summ2;

  int16_t corr_res = (int16_t)sqrtf((float)summ1S + (float)summ2S);
  return corr_res;
}

uint16_t correlation_search(uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, uint16_t* aver_val)
{
  
  uint32_t start_t = get_dwt_value();
  
  //uint16_t max_correl_pos = 0;
  uint16_t max_correl_val = 0;
  
  uint32_t total_summ = 0;
  
  for (uint16_t offset = 0; offset < (PRN_LENGTH * 2); offset++)
  {
    int16_t corr_res = gps_correlation8(
      prn_p, data_i, data_q, tmp_corr_i, tmp_corr_q, offset);
    
    if (corr_res > max_correl_val)
    {
      max_correl_val = corr_res;
      //max_correl_pos = offset;
    }
    
    total_summ += corr_res;
  }
  total_summ = total_summ / (PRN_LENGTH * 2);

  *aver_val = (uint16_t)total_summ;
  
  uint32_t stop_t = get_dwt_value();
  
  diff = stop_t - start_t;
  diff = diff / 168;
  
  return max_correl_val;
}

void gps_shift_to_zero_freq(
  uint8_t* signal_data, uint8_t* data_i, uint8_t* data_q, uint32_t freq)
{
  //Generate frequency
  gps_generate_sin((uint16_t*)tmp_sin_data, PRN_SPI_WORDS_CNT, freq);
  gps_generate_cos((uint16_t*)tmp_cos_data, PRN_SPI_WORDS_CNT, freq);
  
  //Shift to 0 Hz - with inversion
  gps_mult_iq32((uint8_t*)tmp_sin_data, (uint8_t*)tmp_cos_data, (uint8_t*)signal_data,
		data_i, data_q, PRN_SPI_WORDS_CNT * 2, 0);
}