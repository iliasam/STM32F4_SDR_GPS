#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "nav_data.h"
#include "tracking.h"
#include "signal_capture.h"
#include "common_ram.h"
#include "config.h"
#include <math.h>

#define PLL_BAD_STATE_DETECTION_THRESHOLD       (80)

//In code steps
#define GPS_PRE_TRACK_ZONE      (30)
#define GPS_PRE_TRACK_STEP      (GPS_PRE_TRACK_ZONE / TRACKING_CH_LENGTH)

//1 chip is 16 bit, 0.5 chis is 8bit = 1 byte
#define GPS_FINE_RATIO	        8

// Number of tracking cycles to calculate SNR
#define GPS_SNR_CALC_LENGTH     200



//******************************************************************

uint16_t pre_track_best_corr_value = 0;
uint16_t pre_track_best_corr_phase = 0;

void gps_pre_track_process(gps_ch_t* channel, uint8_t* data, uint8_t index);
void gps_pre_tracking_process_data(gps_ch_t* channel, uint8_t points_cnt);
void gps_tracking_data_process(gps_ch_t* channel, uint8_t* data, uint8_t index);
void gps_tracking_dll(gps_ch_t* channel, uint8_t index, int16_t IE, int16_t QE, int16_t IL, int16_t QL);
void gps_tracking_pll(gps_ch_t* channel, uint8_t index, int16_t IP, int16_t QP);
void gps_tracking_pll_check(gps_ch_t* channel, uint8_t index, int16_t new_ip);
void gps_tracking_fll(gps_ch_t* channel, uint8_t index, int16_t IP, int16_t QP);

//****************************************************

void gps_tracking_process(gps_ch_t* channel, uint8_t* data, uint8_t index)
{
  if (channel->tracking_data.state == GPS_NEED_PRE_TRACK)
  {
    channel->tracking_data.code_search_start = channel->acq_data.found_code_phase - (GPS_PRE_TRACK_ZONE / 2);
    channel->tracking_data.code_search_stop = channel->acq_data.found_code_phase + (GPS_PRE_TRACK_ZONE / 2);
    if (channel->tracking_data.code_search_start > (2 * PRN_LENGTH))
      channel->tracking_data.code_search_start = 0;
    if (channel->tracking_data.code_search_stop > (2 * PRN_LENGTH))
      channel->tracking_data.code_search_stop = 2 * PRN_LENGTH;
    
    channel->tracking_data.if_freq_offset_hz = (float)channel->acq_data.found_freq_offset_hz;
    channel->tracking_data.pre_track_count = 0;
    memset(channel->tracking_data.pre_track_phases, 0, PRE_TRACK_POINTS_MAX_CNT * 2);
    channel->tracking_data.state = GPS_PRE_TRACK_RUN;
  }
  
  if (channel->tracking_data.state == GPS_PRE_TRACK_RUN)
  {
    gps_pre_track_process(channel, data, index);
  }
  else if (channel->tracking_data.state == GPS_PRE_TRACK_DONE)
  {
    channel->tracking_data.state = GPS_TRACKING_RUN;
  }
  
  if (channel->tracking_data.state == GPS_TRACKING_RUN)
  {
    gps_tracking_data_process(channel, data, index);
  }
}

void gps_tracking_data_process(gps_ch_t* channel, uint8_t* data, uint8_t index)
{
  uint32_t curr_tick_time = signal_capture_get_packet_cnt();
  
  if (index >= TRACKING_CH_LENGTH)
  {
    return;
  }
  
  uint32_t diff_ticks = curr_tick_time - channel->tracking_data.prev_track_timestamp; //1 normally, 14 at jump
  channel->tracking_data.prev_track_timestamp = curr_tick_time;
  
  if (diff_ticks > 50)//startup check
    diff_ticks = 1;
  
  if (diff_ticks != 1)
  {
    //Simulate skipped tracking steps
    gps_rewind_if_phase(&(channel->tracking_data), (uint8_t)(diff_ticks - 1));
  }
  
  int16_t code_phase_fine = (int16_t)channel->tracking_data.code_phase_fine;
  uint8_t offset_bits = code_phase_fine & (GPS_FINE_RATIO - 1);
  gps_generate_prn_data2(channel, tmp_prn_data, offset_bits);
  
  gps_shift_to_zero_freq_track(
    &(channel->tracking_data), data, (uint8_t*)tmp_data_i, (uint8_t*)tmp_data_q);
  
  uint16_t offset_p = code_phase_fine / GPS_FINE_RATIO;//present
  uint16_t offset_e = offset_p - 1;//early
  uint16_t offset_l = offset_p + 1;//late
  
  if (offset_e >= 2 * PRN_LENGTH)
    offset_e = 2 * PRN_LENGTH - 1;
  if (offset_l >= 2 * PRN_LENGTH)
    offset_l = 0;
  
  int16_t IE, QE;//early
  int16_t IP, QP;//present
  int16_t IL, QL;//late
  
  gps_correlation_iq(tmp_prn_data, tmp_data_i, tmp_data_q, offset_e, &IE, &QE);
  gps_correlation_iq(tmp_prn_data, tmp_data_i, tmp_data_q, offset_p, &IP, &QP);
  gps_correlation_iq(tmp_prn_data, tmp_data_i, tmp_data_q, offset_l, &IL, &QL);
  
  gps_tracking_dll(channel, index, IE, QE, IL, QL);
  gps_tracking_pll(channel, index, IP, QP);
  gps_tracking_fll(channel, index, IP, QP);
  
  gps_nav_data_analyse_new_code(channel, index, IP);
  
  //Integrating values for SNR calculation
  channel->tracking_data.i_part_summ += abs(IP);
  channel->tracking_data.q_part_summ += abs(QP);
  channel->tracking_data.snr_summ_cnt++;
  
  if (channel->tracking_data.snr_summ_cnt > GPS_SNR_CALC_LENGTH)
  {
    if (channel->tracking_data.q_part_summ == 0)
    {
      channel->tracking_data.snr_value = 1.0f;
      return;
    }
    else
    {
      float ratio = (float)channel->tracking_data.i_part_summ / 
        (float)channel->tracking_data.q_part_summ;
      channel->tracking_data.snr_value = 10.0f * log10f(ratio);
    }
    
    channel->tracking_data.snr_summ_cnt = 0;
    channel->tracking_data.i_part_summ = 0;
    channel->tracking_data.q_part_summ = 0;
  }
}

void gps_tracking_pll(gps_ch_t* channel, uint8_t index, int16_t IP, int16_t QP)
{
  float carr_phase_err_rad;
  
  /* PLL discriminator */
  if (IP > 0)
    carr_phase_err_rad = atan2f((float)QP, (float)IP) / M_PI;
  else
    carr_phase_err_rad = atan2((float)-QP, (float)-IP) / M_PI;
  
  if (index != 0)
    return;
  
  float phase_diff_old = carr_phase_err_rad - channel->tracking_data.pll_code_err;
  if (phase_diff_old > M_PI / 2)
    phase_diff_old = M_PI - phase_diff_old;
  if (phase_diff_old < -M_PI / 2)
    phase_diff_old = -M_PI - phase_diff_old;
  
  float dt_s = 0.001f;
  if (channel->nav_data.period_sync_ok_flag)
  {
    channel->tracking_data.if_freq_offset_hz +=
      TRACKING_PLL2_C1 * phase_diff_old +
        (TRACKING_PLL2_C2 * dt_s * carr_phase_err_rad);
  }
  else
  {
    channel->tracking_data.if_freq_offset_hz +=
      TRACKING_PLL1_C1 * phase_diff_old +
        (TRACKING_PLL1_C2 * dt_s * carr_phase_err_rad);
  }
  
  channel->tracking_data.pll_code_err = carr_phase_err_rad;
}

void gps_tracking_fll(gps_ch_t* channel, uint8_t index, int16_t IP, int16_t QP)
{
  gps_tracking_pll_check(channel, index, IP);
  
  if (index == 0) //first index is skipped
  {
    channel->tracking_data.fll_old_i = IP;
    channel->tracking_data.fll_old_q = QP;
    return;
  }
  
  int16_t oldIP = channel->tracking_data.fll_old_i;
  int16_t oldQP = channel->tracking_data.fll_old_q;
  
  /* FLL discriminator */
  float f1 = (IP == 0) ? (M_PI / 2) : atanf((float)QP / (float)IP);
  float f2 = (oldIP == 0) ? (M_PI / 2) : atanf((float)oldQP / (float)oldIP);
  float freq_diff_rad = f1 - f2;
  
  if (freq_diff_rad > M_PI / 2)
    freq_diff_rad = M_PI - freq_diff_rad;
  if (freq_diff_rad < -M_PI / 2)
    freq_diff_rad = -M_PI - freq_diff_rad;
  
  float old_diff_rad = freq_diff_rad - channel->tracking_data.fll_err;
  if (old_diff_rad > M_PI / 2)
    old_diff_rad = M_PI - old_diff_rad;
  if (old_diff_rad < -M_PI / 2)
    old_diff_rad = -M_PI - old_diff_rad;
  
  
  float dt_s = 0.001f;
  float diff_f_hz = TRACKING_FLL1_C1 * dt_s * old_diff_rad + (TRACKING_FLL1_C2 * dt_s * freq_diff_rad);
  channel->tracking_data.if_freq_offset_hz += diff_f_hz;
  
  channel->tracking_data.fll_old_i = IP;
  channel->tracking_data.fll_old_q = QP;
  channel->tracking_data.fll_err = freq_diff_rad;
  
  //char tmp_txt[100];
  //sprintf(tmp_txt, "%.02f\n", freq_diff_rad);
  //sprintf(tmp_txt, "%.02f diff=%.02f \n", channel->tracking_data.if_freq_offset_hz, diff_f_hz);
  //OutputDebugString((LPCSTR)tmp_txt);
}

// Check that PLL is stable and restart it, if not
void gps_tracking_pll_check(gps_ch_t* channel, uint8_t index, int16_t new_ip)
{
  if (index >= TRACKING_CH_LENGTH)
    return;
  
  channel->tracking_data.pll_check_buf[index] = new_ip;
  if (index < (TRACKING_CH_LENGTH - 1))
    return;
  
  //come here then "index" is at end (index == TRACKING_CH_LENGTH - 1)
  //Count number of sign switching
  uint8_t swith_counter = 0;//sign switch
  uint8_t pol_old = (channel->tracking_data.pll_check_buf[0] > 0) ? 1 : 0;
  uint8_t pol;
  for (uint8_t i = 1; i < TRACKING_CH_LENGTH; i++)
  {
    pol = (channel->tracking_data.pll_check_buf[i] > 0) ? 1 : 0;
    if (pol != pol_old)
      swith_counter++;
    pol_old = pol;
  }
  
  //A kind of filtering
  //Good data have only 1 or 0 transitions (one bit is 20ms long,  and TRACKING_CH_LENGTH < 20)
  if (swith_counter > 1) 
  {
    channel->tracking_data.pll_bad_state_cnt++;
    if (channel->tracking_data.pll_bad_state_cnt > 10)
      channel->tracking_data.pll_bad_state_cnt = 10;
  }
  else
  {
    if (channel->tracking_data.pll_bad_state_cnt > 0)
      channel->tracking_data.pll_bad_state_cnt--;
  }
  
  if (channel->tracking_data.pll_bad_state_cnt > 9)
  {
    channel->tracking_data.pll_bad_state_master_cnt++;
  }
  else if (channel->tracking_data.pll_bad_state_cnt == 0)
  {
    channel->tracking_data.pll_bad_state_master_cnt = 0;
  }
  
  if (channel->tracking_data.pll_bad_state_master_cnt > PLL_BAD_STATE_DETECTION_THRESHOLD)
  {
    //Too many bad data, looks like "false lock"
    //Try to change frequency
    channel->tracking_data.pll_bad_state_master_cnt = 0;
    channel->tracking_data.pll_bad_state_cnt = 0;
    
    //Change carrier freqency to a random
    
    int16_t diff_hz = 0;
    int16_t new_offset_hz;
    do
    {
      uint16_t rand_offset = (uint16_t)(rand() % ACQ_SEARCH_STEP_HZ);
      new_offset_hz = channel->acq_data.found_freq_offset_hz - rand_offset + (ACQ_SEARCH_STEP_HZ / 2);
      diff_hz = (int16_t)channel->tracking_data.if_freq_offset_hz - new_offset_hz;
    } while (abs(diff_hz) < 200);//until new value will  differ significantly
    
    channel->tracking_data.if_freq_offset_hz = (float)new_offset_hz;
    
    //printf("BAD LOCK\n");
  }
}

void gps_tracking_dll(gps_ch_t* channel, uint8_t index, int16_t IE, int16_t QE, int16_t IL, int16_t QL)
{
  //static float avr_summ = 0.0f;
  //static uint8_t summ_cnt = 0;
  //static int packet_cnt = 0;
  
  int32_t IE2 = (int32_t)IE * (int32_t)IE;
  int32_t QE2 = (int32_t)QE * (int32_t)QE;
  
  int32_t IL2 = (int32_t)IL * (int32_t)IL;
  int32_t QL2 = (int32_t)QL * (int32_t)QL;
  
  int32_t part1 = (IE2 + QE2) - (IL2 + QL2);
  int32_t part2 = (IE2 + QE2) + (IL2 + QL2);
  float code_err = (float)part1 / (float)part2;
  
  //float code_err = (sqrtf(IE2 + QE2) - sqrtf(IL2 + QL2)) / 
  //	(sqrtf(IE2 + QE2) + sqrtf(IL2 + QL2));
  
  code_err = -code_err;
  
  float dt_s = 0.001f;
  /* 2nd order DLL */
  channel->tracking_data.code_phase_fine += (TRACKING_DLL1_C1 * (code_err - channel->tracking_data.dll_code_err) +
                                             TRACKING_DLL1_C2 * dt_s * code_err);
  
  if (channel->tracking_data.code_phase_fine < 0.0f)
  {
    channel->tracking_data.code_phase_fine = 
      (float)(PRN_LENGTH * 2 * GPS_FINE_RATIO) - channel->tracking_data.code_phase_fine;
  }
  else if (channel->tracking_data.code_phase_fine > (float)(PRN_LENGTH * 2 * GPS_FINE_RATIO))
  {
    channel->tracking_data.code_phase_fine =
      channel->tracking_data.code_phase_fine - (float)(PRN_LENGTH * 2 * GPS_FINE_RATIO);
  }
  
  //packet_cnt++;
  //channel->tracking_data.code_phase_fine2 = (2039.8f - (float)packet_cnt * 0.001407) * 8.0f;
  //channel->tracking_data.code_phase_fine2 = (64.5f + (float)packet_cnt * (1.0f / 30.0f / 8.0f)) * 8.0f;
  //channel->tracking_data.code_phase_fine2 = (60.0f) * 8.0f;
  
  channel->tracking_data.dll_code_err = code_err;
  
  //float tmp_code = channel->tracking_data.code_phase_fine2 / 8.0f;
  //char tmp_txt[100];
  //sprintf(tmp_txt, "error=%.02f fine code=%.02f\n", code_err, tmp_code);
  //sprintf(tmp_txt, "%.02f\n", code_err);
  //sprintf(tmp_txt, "%.02f\n", tmp_code);
  //OutputDebugString((LPCSTR)tmp_txt);
}


void gps_pre_track_process(gps_ch_t* channel, uint8_t* data, uint8_t index)
{
  if (index >= TRACKING_CH_LENGTH)
    return;
  
  gps_generate_prn_data2(channel, tmp_prn_data, 0);
  gps_shift_to_zero_freq(
                         data,
                         (uint8_t*)tmp_data_i, (uint8_t*)tmp_data_q,
                         (float)IF_FREQ_HZ + channel->tracking_data.if_freq_offset_hz);
  
  uint16_t start_idx = channel->tracking_data.code_search_start + index * GPS_PRE_TRACK_STEP;
  uint16_t stop_idx = start_idx + GPS_PRE_TRACK_STEP;
  if (stop_idx > 2 * PRN_LENGTH)
    stop_idx = 2 * PRN_LENGTH;
  
  for (uint16_t code_idx = start_idx; code_idx < stop_idx; code_idx++)
  {
    int16_t corr_res = gps_correlation8(tmp_prn_data, tmp_data_i, tmp_data_q, code_idx);
    if (corr_res > pre_track_best_corr_value)
    {
      pre_track_best_corr_value = corr_res;
      pre_track_best_corr_phase = code_idx;
    }
  }
  
  if (index == (TRACKING_CH_LENGTH - 1))
  {
    channel->tracking_data.pre_track_phases[channel->tracking_data.pre_track_count] = 
      pre_track_best_corr_phase;
    channel->tracking_data.pre_track_count++;
    
    if (channel->tracking_data.pre_track_count > (PRE_TRACK_POINTS_MAX_CNT - 10))
    {
      gps_pre_tracking_process_data(channel, channel->tracking_data.pre_track_count);
    }
    
    if (channel->tracking_data.pre_track_count >= PRE_TRACK_POINTS_MAX_CNT)
    {
      channel->tracking_data.pre_track_count = 0;
      memset(channel->tracking_data.pre_track_phases, 0, PRE_TRACK_POINTS_MAX_CNT * 2);
    }
    
    //sprintf(tmp_txt, "%d\n", pre_track_best_corr_phase);
    
    pre_track_best_corr_value = 0;
  }
}

int compare_pre_tracking(const void * a, const void * b)
{
  return (*(uint16_t*)a - *(uint16_t*)b);
}

void gps_pre_tracking_process_data(gps_ch_t* channel, uint8_t points_cnt)
{
  qsort(channel->tracking_data.pre_track_phases, points_cnt, sizeof(uint16_t), compare_pre_tracking);
  uint8_t chain_items = 0;
  uint16_t max_chain_length = 0;
  uint16_t found_code_phase = 0;
  
  for (uint8_t i = 1; i < points_cnt; i++)
  {
    uint16_t diff = channel->tracking_data.pre_track_phases[i] - 
      channel->tracking_data.pre_track_phases[i - 1];
    
    if (abs(diff) < 1)
      chain_items++;
    else
    {
      if (chain_items > max_chain_length)
      {
        max_chain_length = chain_items;
        found_code_phase = channel->tracking_data.pre_track_phases[i-1];
      }
      chain_items = 0;
    }
  }
  if (chain_items > max_chain_length)
  {
    max_chain_length = chain_items;
    found_code_phase = channel->tracking_data.pre_track_phases[points_cnt - 1];
  }
  
  if (found_code_phase)
  {
    //sprintf(tmp_txt, "FOUND TRACK PHASE=%d\n", found_code_phase);
    //convert to fine points
    channel->tracking_data.code_phase_fine = (float)((found_code_phase) * GPS_FINE_RATIO);
    channel->tracking_data.state = GPS_PRE_TRACK_DONE;
  }
}


