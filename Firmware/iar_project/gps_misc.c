#include "gps_misc.h"
#include "config.h"
#include "stdint.h"
#include <math.h>
#include "delay_us_timer.h"
#include "intrinsics.h"
#include "string.h"
#include "common_ram.h"

volatile static uint32_t diff;

uint8_t gps_summ_table[256];

uint8_t gps_summ_table16[65536];

uint8_t gps_summ16(uint16_t data);
uint16_t gps_buff_summ16(uint16_t* data, uint16_t length);


void gps_generate_prn(uint8_t* dest, int prn);

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

//length - data in bytes 
void gps_mult_and_summ(
  uint8_t* src_i, uint8_t* src_q, uint8_t* src2, 
  uint16_t* summ_i, uint16_t* summ_q, uint16_t length, uint16_t offset)
{
  uint16_t* src2_p16 = (uint16_t*)src2;
  
  uint16_t* src_i_p16 = (uint16_t*)(src_i + offset);
  uint16_t* src_q_p16 = (uint16_t*)(src_q + offset);
  
  uint8_t small_offset = (uint8_t)(offset & 1);//=1 if offset is odd number
  
  uint16_t length_words_p1 = (length - offset) / 2; //divide -> bytes to words
  
  uint16_t tmp_i, tmp_q;
  
  uint16_t cnt_i = 0;
  uint16_t cnt_q = 0;
  
  for (uint16_t i = 0; i < length_words_p1; i++)
  {
    tmp_i = (*src_i_p16) ^ src2_p16[i];
    src_i_p16++;
    tmp_q = (*src_q_p16) ^ src2_p16[i];
    src_q_p16++;
    cnt_i += gps_summ_table16[tmp_i];
    cnt_q += gps_summ_table16[tmp_q];
  }
  
  //reset to start of buffer
  src_i_p16 = (uint16_t*)(src_i + small_offset);
  src_q_p16 = (uint16_t*)(src_q + small_offset);
  
  //If offset is odd, we process 1 word less
  for (uint16_t i = length_words_p1 + small_offset; i < ((length / 2) - small_offset); i++)
  {
    tmp_i = (*src_i_p16) ^ src2_p16[i];
    src_i_p16++;
    tmp_q = (*src_q_p16) ^ src2_p16[i];
    src_q_p16++;
    cnt_i += gps_summ_table16[tmp_i];
    cnt_q += gps_summ_table16[tmp_q];
  }
  
  *summ_i = cnt_i;
  *summ_q = cnt_q;
}

int16_t gps_correlation8(
  uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, uint16_t offset)
{ 
  int16_t summ1;
  int16_t summ2;
  
  gps_mult_and_summ(
                    (uint8_t*)data_i, (uint8_t*)data_q, (uint8_t*)prn_p, 
                    (uint16_t*)&summ1, (uint16_t*)&summ2, 
                    PRN_SPI_WORDS_CNT * 2, offset);
  summ1 = summ1 - BITS_IN_PRN / 2;
  summ2 = summ2 - BITS_IN_PRN / 2;
  
  /*
  if (summ1 < 0)
    summ1 = 0;
  if (summ2 < 0)
    summ2 = 0;

  int32_t summ1S = summ1 * summ1;
  int32_t summ2S = summ2 * summ2;
  int16_t corr_res = (int16_t)sqrtf((float)summ1S + (float)summ2S);
  */
  int16_t corr_res = summ1 + summ2;
  return corr_res;
}

void gps_correlation_iq(
  uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, uint16_t offset, 
  int16_t* res_i, int16_t* res_q)
{
  int16_t summ_i;
  int16_t summ_q;
  
  gps_mult_and_summ(
    (uint8_t*)data_i, (uint8_t*)data_q, 
    (uint8_t*)prn_p, 
    (uint16_t*)&summ_i, (uint16_t*)&summ_q, 
    PRN_SPI_WORDS_CNT * 2, offset);
  summ_i = summ_i - BITS_IN_PRN / 2;
  summ_q = summ_q - BITS_IN_PRN / 2;
  
  *res_i = summ_i;
  *res_q = summ_q;
}

uint16_t correlation_search(
	uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, 
	uint16_t start_shift, uint16_t stop_shift,  uint16_t* aver_val, uint16_t* phase)
{
  //uint32_t start_t = get_dwt_value();
  
  uint16_t max_correl_pos = 0;
  int16_t max_correl_val = 0;
  
  int32_t total_summ = 0;
  
  for (uint16_t offset = start_shift; offset < stop_shift; offset++)
  {
    int16_t corr_res = gps_correlation8(prn_p, data_i, data_q, offset);
    
    if (corr_res > max_correl_val)
    {
      max_correl_val = corr_res;
      max_correl_pos = offset;
    }
    
    total_summ += corr_res;
  }
  total_summ = total_summ / (PRN_LENGTH * 2);
  
  if (total_summ < 0)
    total_summ = 0;
  
  *aver_val = (uint16_t)total_summ;
  *phase = max_correl_pos;
  
  //uint32_t stop_t = get_dwt_value();
 // diff = stop_t - start_t;
  //diff = diff / 168;
  
  return max_correl_val;
}



void gps_shift_to_zero_freq(uint8_t* signal_data, uint8_t* data_i, uint8_t* data_q, float freq_hz)
{
  const uint32_t sin_buf32[4] = { 0x33333333, 0x9999999, 0xCCCCCCCC, 0x66666666 };
  const uint32_t cos_buf32[4] = { 0x9999999, 0xCCCCCCCC, 0x66666666, 0x33333333 };
  
  uint32_t acc_step = (uint32_t)(freq_hz / (IF_NCO_STEP_HZ));
  uint64_t acc_step64 = (uint64_t)acc_step * 32;//we do 32steps per 1 word
  acc_step = (uint32_t)acc_step64;
  
  uint32_t accum = 0;
  uint32_t* ptr_i32 = (uint32_t*)data_i;
  uint32_t* ptr_q32 = (uint32_t*)data_q;
  uint32_t* signal_p32 = (uint32_t*)signal_data;
  
  uint16_t word_cnt = 0;
  for (word_cnt = 0; word_cnt < (PRN_SPI_WORDS_CNT / 2); word_cnt++)//to get 32bit words
  {
    uint32_t phase = (accum >> 30);//upper 2 bits
    *ptr_i32 = sin_buf32[phase] ^ *signal_p32;
    *ptr_q32 = cos_buf32[phase] ^ *signal_p32;
    accum += acc_step;
    
    ptr_i32++;
    ptr_q32++;
    signal_p32++;
  }
}

//With keeping phase
void gps_shift_to_zero_freq_track(
  gps_tracking_t* trk_channel, uint8_t* signal_data, uint8_t* data_i, uint8_t* data_q)
{
  const uint32_t sin_buf32[4] = { 0x33333333, 0x9999999, 0xCCCCCCCC, 0x66666666 };
  const uint32_t cos_buf32[4] = { 0x9999999, 0xCCCCCCCC, 0x66666666, 0x33333333 };
  
  uint32_t acc_step = (uint32_t)(
    ((float)IF_FREQ_HZ + trk_channel->if_freq_offset_hz) / (IF_NCO_STEP_HZ));
  uint64_t acc_step64 = (uint64_t)acc_step * 32;
  acc_step = (uint32_t)acc_step64;
  
  uint32_t accum = trk_channel->if_freq_accum;
  uint32_t* ptr_i32 = (uint32_t*)data_i;
  uint32_t* ptr_q32 = (uint32_t*)data_q;
  uint32_t* signal_p32 = (uint32_t*)signal_data;
  
  uint16_t word_cnt = 0;
  for (word_cnt = 0; word_cnt < (PRN_SPI_WORDS_CNT / 2); word_cnt++)//to get 32bit words
  {
    uint32_t phase = (accum >> 30);//upper 2 bits
    *ptr_i32 = sin_buf32[phase] ^ *signal_p32;
    *ptr_q32 = cos_buf32[phase] ^ *signal_p32;
    accum += acc_step;
    
    ptr_i32++;
    ptr_q32++;
    signal_p32++;
  }
  
  trk_channel->if_freq_accum = accum;
}


void gps_generate_prn_data2(
  gps_ch_t* channel, uint16_t* data, uint16_t offset_bits)
{
  memset(data, 0, PRN_SPI_WORDS_CNT * 2);
  uint32_t* tmp_p32;
  
  uint32_t wr_word = 0x0000FFFF << (offset_bits & 15);
  uint8_t* prn_data_p = channel->prn_code;//array with 0/1 values
  
  for (uint16_t word_cnt = 0; word_cnt < PRN_SPI_WORDS_CNT; word_cnt++)
  {
    tmp_p32 = (uint32_t*)data;
    if (*prn_data_p)
      *tmp_p32 |= wr_word;
    data++;
    prn_data_p++;
  }
}

//*********************************************************************

//Fill PRN table (1023 bits)
void gps_channell_prepare(gps_ch_t* channel)
{
  if (channel->prn < 1)
    return;
  gps_generate_prn(channel->prn_code, channel->prn);
}

//Code is taken from https://github.com/taroz/GNSS-SDRLIB
/* C/A code (IS-GPS-200) -----------------------------------------------------*/
void gps_generate_prn(uint8_t* dest, int prn)
{
  const static int16_t delay[] = { /* G2 delay (chips) */
    5,   6,   7,   8,  17,  18, 139, 140, 141, 251,   /*   1- 10 */
    252, 254, 255, 256, 257, 258, 469, 470, 471, 472,   /*  11- 20 */
    473, 474, 509, 512, 513, 514, 515, 516, 859, 860,   /*  21- 30 */
    861, 862, 863, 950, 947, 948, 950,  67, 103,  91,   /*  31- 40 */
    19, 679, 225, 625, 946, 638, 161,1001, 554, 280,   /*  41- 50 */
    710, 709, 775, 864, 558, 220, 397,  55, 898, 759,   /*  51- 60 */
    367, 299,1018, 729, 695, 780, 801, 788, 732,  34,   /*  61- 70 */
    320, 327, 389, 407, 525, 405, 221, 761, 260, 326,   /*  71- 80 */
    955, 653, 699, 422, 188, 438, 959, 539, 879, 677,   /*  81- 90 */
    586, 153, 792, 814, 446, 264,1015, 278, 536, 819,   /*  91-100 */
    156, 957, 159, 712, 885, 461, 248, 713, 126, 807,   /* 101-110 */
    279, 122, 197, 693, 632, 771, 467, 647, 203, 145,   /* 111-120 */
    175,  52,  21, 237, 235, 886, 657, 634, 762, 355,   /* 121-130 */
    1012, 176, 603, 130, 359, 595,  68, 386, 797, 456,   /* 131-140 */
    499, 883, 307, 127, 211, 121, 118, 163, 628, 853,   /* 141-150 */
    484, 289, 811, 202,1021, 463, 568, 904, 670, 230,   /* 151-160 */
    911, 684, 309, 644, 932,  12, 314, 891, 212, 185,   /* 161-170 */
    675, 503, 150, 395, 345, 846, 798, 992, 357, 995,   /* 171-180 */
    877, 112, 144, 476, 193, 109, 445, 291,  87, 399,   /* 181-190 */
    292, 901, 339, 208, 711, 189, 263, 537, 663, 942,   /* 191-200 */
    173, 900,  30, 500, 935, 556, 373,  85, 652, 310    /* 201-210 */
  };
  int8_t G1[PRN_LENGTH], G2[PRN_LENGTH], R1[10], R2[10], C1, C2;
  int i, j;
  
  if (prn < 1)
    return;
  
  for (i = 0;i < 10;i++)
    R1[i] = R2[i] = -1;
  
  for (i = 0; i < PRN_LENGTH; i++)
  {
    G1[i] = R1[9];
    G2[i] = R2[9];
    C1 = R1[2] * R1[9];
    C2 = R2[1] * R2[2] * R2[5] * R2[7] * R2[8] * R2[9];
    for (j = 9; j > 0; j--)
    {
      R1[j] = R1[j - 1];
      R2[j] = R2[j - 1];
    }
    R1[0] = C1;
    R2[0] = C2;
  }
  for (i = 0, j = PRN_LENGTH - delay[prn - 1]; i < PRN_LENGTH; i++, j++)
  {
    short tmp_val = -G1[i] * G2[j % PRN_LENGTH];
    dest[i] = (tmp_val < 0) ? 0 : 1;
  }
  
  return;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/*
//working

//length - in 16-bit words
uint32_t gps_generate_sin_cos(
  uint16_t* ptr_i, uint16_t* ptr_q, uint16_t length, float freq_hz, uint32_t start_accum)
{
  const uint32_t sin_buf32[4] = { 0x33333333, 0x9999999, 0xCCCCCCCC, 0x66666666};
  const uint32_t cos_buf32[4] = { 0x9999999, 0xCCCCCCCC, 0x66666666, 0x33333333};
  
  uint32_t acc_step = (uint32_t)(freq_hz / (IF_NCO_STEP_HZ));
  uint64_t acc_step64 = (uint64_t)acc_step * 32;
  acc_step = (uint32_t)acc_step64;
  
  uint32_t accum = start_accum;
  uint32_t* ptr_i32 = (uint32_t*)ptr_i;
  uint32_t* ptr_q32 = (uint32_t*)ptr_q;
  
  uint16_t word_cnt = 0;
  for (word_cnt = 0; word_cnt < (length / 2); word_cnt++)//to get 32bit words
  {
    uint32_t phase = (accum >> 30);//upper 2 bits
    *ptr_i32 = sin_buf32[phase];
    *ptr_q32 = cos_buf32[phase];
    accum += acc_step;
    
    ptr_i32++;
    ptr_q32++;
  }
  
  return accum;
}

void gps_shift_to_zero_freq(
  uint8_t* signal_data, uint8_t* data_i, uint8_t* data_q, float freq_hz)
{
  //Generate frequency
  gps_generate_sin_cos(
    tmp_sin_data, tmp_cos_data, PRN_SPI_WORDS_CNT, freq_hz, 0);

  //Shift to 0 Hz - with inversion
  gps_mult_iq32((uint8_t*)tmp_sin_data, (uint8_t*)tmp_cos_data, (uint8_t*)signal_data,
		data_i, data_q, PRN_SPI_WORDS_CNT * 2, 0);
}
*/
