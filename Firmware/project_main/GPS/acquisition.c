#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "acquisition.h"
#include "common_ram.h"
#include "signal_capture.h"
#include "config.h"

#define FREQ_SEARCH_POINTS_MAX_CNT		25
#define ACQ_CODE_SEARCH_TIMEOUT_MS              120000


// Histogram, one item is for one freq. step, used for freq. search
uint32_t acq_histogram[ACQ_COUNT];
uint16_t acq_histogram_cnt = 0;

//This is not a histogram, sorted when being processed, ussed for freq. search only
uint16_t acq_single_freq_phases[FREQ_SEARCH_POINTS_MAX_CNT];
uint8_t acq_single_freq_count = 0;

void acquisition_process_channel(gps_ch_t* channel, uint8_t* data);
void acquisition_freq_search(gps_ch_t* channel, uint8_t* data);


void acquisition_code_phase_search(gps_ch_t* channel, uint8_t* data);
void acquisition_add_to_phase_hist(gps_ch_t* channel, uint8_t idx);
float acquisition_phase_hist_average(gps_ch_t* channel);

void acquisition_single_freq_check_reset(void);
void acquisition_process_single_freq_data(gps_ch_t* channel, uint8_t points_cnt);
void acquisition_process_single_freq_histogram(gps_ch_t* channel);

//****************************************************

void acquisition_process(gps_ch_t* channel, uint8_t* data)
{
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    acquisition_process_channel(&channel[i], data);
  }
}

//Reset "acq_single_freq_phases"
void acquisition_single_freq_check_reset(void)
{
  memset(acq_histogram, 0, sizeof(acq_histogram));
  memset(acq_single_freq_phases, 0, sizeof(acq_single_freq_phases));
  acq_single_freq_count = 0;
  acq_histogram_cnt = 0;
}


void acquisition_start_channel(gps_ch_t* channel)
{
  if (channel->acq_data.state == GPS_ACQ_NEED_FREQ_SEARCH)
  {
    if (channel->acq_data.given_freq_offset_hz != 0)
    {
      channel->acq_data.found_freq_offset_hz = channel->acq_data.given_freq_offset_hz;
      channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_DONE;
      return;
    }
    
    acquisition_single_freq_check_reset();
    channel->acq_data.freq_index = 0;
    channel->acq_data.single_freq_length = 10;
    channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_RUN;
  }
}

void acquisition_start_code_search_channel(gps_ch_t* channel)
{
  if (channel->acq_data.state == GPS_ACQ_FREQ_SEARCH_DONE)
  {
    memset(channel->acq_data.code_phase_histogram, 0, ACQ_PHASE1_HIST_SIZE);
    channel->acq_data.code_search_start = 0;
    channel->acq_data.code_search_stop = 2 * PRN_LENGTH;
    //Histogram resolution is 64 steps
    channel->acq_data.code_hist_step = ACQ_PHASE1_HIST_STEP;
    channel->acq_data.start_timestamp = signal_capture_get_packet_cnt();
    
    channel->acq_data.state = GPS_ACQ_CODE_PHASE_SEARCH1;
  }
}

void acquisition_start_code_search3_channel(gps_ch_t* channel)
{
  if (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH2_DONE)
  {
    memset(channel->acq_data.code_phase_histogram, 0, ACQ_PHASE1_HIST_SIZE);
    
    channel->acq_data.code_search_start = channel->acq_data.found_code_phase - 30;
    channel->acq_data.code_search_stop = channel->acq_data.found_code_phase + 30;
    //check overflow
    if (channel->acq_data.code_search_start > (2 * PRN_LENGTH))
      channel->acq_data.code_search_start = 0;
    if (channel->acq_data.code_search_stop > (2 * PRN_LENGTH))
      channel->acq_data.code_search_stop = 2 * PRN_LENGTH;
    
    //Histogram resollution is near 3 steps
    channel->acq_data.code_hist_step = (60 / ACQ_PHASE1_HIST_SIZE) + 1;
    acquisition_single_freq_check_reset();
    channel->acq_data.start_timestamp = signal_capture_get_packet_cnt();
    channel->acq_data.state = GPS_ACQ_CODE_PHASE_SEARCH3;
    return;
  }
}

void acquisition_process_channel(gps_ch_t* channel, uint8_t* data)
{
  if (channel->prn < 1)
    return;
  
  if (channel->acq_data.state == GPS_ACQ_DONE)
    return;
  
  if (channel->acq_data.state == GPS_ACQ_FREQ_SEARCH_RUN)
  {
    acquisition_freq_search(channel, data);
    return;
  }
  
  //CODE search
  
  if (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH1_DONE)
  {
    memset(channel->acq_data.code_phase_histogram, 0, ACQ_PHASE1_HIST_SIZE);
    
    channel->acq_data.code_search_start = channel->acq_data.found_code_phase - 250;
    channel->acq_data.code_search_stop = channel->acq_data.found_code_phase + 250;
    //check overflow
    if (channel->acq_data.code_search_start > (2 * PRN_LENGTH))
      channel->acq_data.code_search_start = 0;
    if (channel->acq_data.code_search_stop > (2 * PRN_LENGTH))
      channel->acq_data.code_search_stop = 2 * PRN_LENGTH;
    
    //Histogram resollution is near 16 steps
    channel->acq_data.code_hist_step = (500 / ACQ_PHASE1_HIST_SIZE) + 1;
    acquisition_single_freq_check_reset();
    channel->acq_data.start_timestamp = signal_capture_get_packet_cnt();
    channel->acq_data.state = GPS_ACQ_CODE_PHASE_SEARCH2;
    return;
  }
  
  
  
  if (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH3_DONE)
  {
    //printf("FINAL ACQ (SRCH_3) CODE=%d\n", channel->acq_data.found_code_phase);
    channel->acq_data.state = GPS_ACQ_DONE;
  }
  
  if ((channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH1) || 
      (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH2) ||
        (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH3) )
  {
    acquisition_code_phase_search(channel, data);
    return;
  }
  
  return;
}

/// Check collected data using histoogram analyse(no sorting)
/// Can be long!
void acquisition_code_phase_search(gps_ch_t* channel, uint8_t* data)
{
  gps_generate_prn_data2(channel, tmp_prn_data, 0);
  gps_shift_to_zero_freq(
                         data, 
                         (uint8_t*)tmp_data_i, (uint8_t*)tmp_data_q, 
                         IF_FREQ_HZ + channel->acq_data.found_freq_offset_hz);
  uint16_t avr_val;
  uint16_t best_code_phase = 0;
  
  uint16_t res_val = correlation_search(
                                        tmp_prn_data, tmp_data_i, tmp_data_q, 
                                        channel->acq_data.code_search_start, channel->acq_data.code_search_stop, 
                                        &avr_val, &best_code_phase);
  
  if ((best_code_phase < channel->acq_data.code_search_start) ||
      (best_code_phase >= channel->acq_data.code_search_stop))
  {
    return;
  }
  
  uint32_t time_diff_ms = signal_capture_get_packet_cnt() - channel->acq_data.start_timestamp;
  if (time_diff_ms > ACQ_CODE_SEARCH_TIMEOUT_MS)
  {
    memset(channel->acq_data.code_phase_histogram, 0, ACQ_PHASE1_HIST_SIZE);
    channel->acq_data.start_timestamp = signal_capture_get_packet_cnt();
  }
  
  //Index in histogram
  uint8_t index = ((best_code_phase - channel->acq_data.code_search_start) / channel->acq_data.code_hist_step);
  acquisition_add_to_phase_hist(channel, index);
  
  uint8_t max_val = 0;
  uint8_t max_pos_idx = 0;
  //add +2 too remove problem with rounding
  uint16_t hist_length = (channel->acq_data.code_search_stop + 2 - channel->acq_data.code_search_start) / 
    channel->acq_data.code_hist_step;
  uint8_t unique_elements = 0;
  for (uint8_t idx = 0; idx < hist_length; idx++)
  {
    if (channel->acq_data.code_phase_histogram[idx] > max_val)
    {
      max_val = channel->acq_data.code_phase_histogram[idx];
      max_pos_idx = idx;
    }
    if (channel->acq_data.code_phase_histogram[idx] > 0)
      unique_elements++;
  }
  if (max_val < 2)
    return;
  
  float hist_avr_val = acquisition_phase_hist_average(channel);
  if (hist_avr_val < 0.01f)
    return;
  float ratio = (float)max_val / hist_avr_val;
  
  if ((unique_elements == 1) && (max_val > 3))
    ratio = 10.0f;
  
  if (ratio > 3.2f)
  {
    channel->acq_data.found_code_phase = channel->acq_data.code_search_start + max_pos_idx * channel->acq_data.code_hist_step;
    
    if (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH1)
      channel->acq_data.state = GPS_ACQ_CODE_PHASE_SEARCH1_DONE;
    
    if (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH2)
      channel->acq_data.state = GPS_ACQ_CODE_PHASE_SEARCH2_DONE;
    
    if (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH3)
      channel->acq_data.state = GPS_ACQ_CODE_PHASE_SEARCH3_DONE;
  }
}

void acquisition_freq_test(gps_ch_t* channel, uint8_t* data)
{
  char tmp_txt[100];
  gps_generate_prn_data2(channel, tmp_prn_data, 0);
  int16_t freq_offset_hz = channel->acq_data.given_freq_offset_hz;
  
  gps_shift_to_zero_freq(data, (uint8_t*)tmp_data_i, (uint8_t*)tmp_data_q, IF_FREQ_HZ + freq_offset_hz);
  uint16_t avr_val;
  uint16_t best_phase1 = 0;
  uint16_t res_val = correlation_search(tmp_prn_data, tmp_data_i, tmp_data_q, 0, (PRN_LENGTH * 2), &avr_val, &best_phase1);
}

void acquisition_freq_search(gps_ch_t* channel, uint8_t* data)
{
  gps_generate_prn_data2(channel, tmp_prn_data, 0);
  
  uint8_t idx = channel->acq_data.freq_index;
  int16_t freq_offset_hz = -ACQ_SEARCH_FREQ_HZ + idx * ACQ_SEARCH_STEP_HZ;
  //int16_t freq_offset_hz = 2000;
  
  gps_shift_to_zero_freq(data, (uint8_t*)tmp_data_i, (uint8_t*)tmp_data_q, IF_FREQ_HZ + freq_offset_hz);
  uint16_t avr_val;
  uint16_t best_phase = 0;
  uint16_t res_val = correlation_search(tmp_prn_data, tmp_data_i, tmp_data_q, 0, (PRN_LENGTH * 2), &avr_val, &best_phase);
  
  acq_single_freq_phases[acq_single_freq_count] = best_phase;
  acq_single_freq_count++;
  
  if (acq_single_freq_count >= channel->acq_data.single_freq_length)
  {
    acquisition_process_single_freq_data(channel, acq_single_freq_count);
    acquisition_process_single_freq_histogram(channel);
    acquisition_single_freq_check_reset();
    
    channel->acq_data.freq_index++;
    if (channel->acq_data.freq_index >= ACQ_COUNT)
    {
      channel->acq_data.freq_index = 0;
    }
  }
}

int compare_freq1_function(const void * a, const void * b)
{
  return (*(uint16_t*)a - *(uint16_t*)b);
}

//Used for frequency search
//Fill histogram "acq_histogram"
void acquisition_process_single_freq_data(gps_ch_t* channel, uint8_t points_cnt)
{
  qsort(acq_single_freq_phases, points_cnt, sizeof(uint16_t), compare_freq1_function);
  uint8_t chain_items = 0;
  uint16_t max_chain_length = 0;
  uint8_t same_flag = 0;
  
  for (uint8_t i = 1; i < points_cnt; i++)
  {
    uint16_t diff = acq_single_freq_phases[i] - acq_single_freq_phases[i - 1];
    if (abs(diff) < 3)
      same_flag = 1;
    
    if (diff < 15)
      chain_items++;
    else
    {
      if ((chain_items > max_chain_length) && (same_flag))
        max_chain_length = chain_items;
      chain_items = 0;
      same_flag = 0;
    }
  }
  if (chain_items > max_chain_length)
    max_chain_length = chain_items;
  
  if (max_chain_length >= 2)
  {
    acq_histogram[channel->acq_data.freq_index] += max_chain_length;
  }
  
  //printf("PRN=%d FREQ IDX=%d, CHAIN LNG=%d\n", channel->prn, channel->acq_data.freq_index, max_chain_length);
}

void acquisition_process_single_freq_histogram(gps_ch_t* channel)
{
  uint8_t non_zero_cnt = 0;
  uint8_t max_pos = 0;
  uint8_t max_val = 0;
  for (uint8_t i = 0; i < ACQ_COUNT; i++)
  {
    if (acq_histogram[i] > 0)
      non_zero_cnt++;
    
    if (acq_histogram[i] > max_val)
    {
      max_val = acq_histogram[i];
      max_pos = i;
    }
  }
  
  if ((non_zero_cnt == 1) && (max_val >= 3))
  {
    channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_DONE;
    channel->acq_data.found_freq_offset_hz = -ACQ_SEARCH_FREQ_HZ + max_pos * ACQ_SEARCH_STEP_HZ;
    channel->acq_data.hist_ratio = 10.0f;
  }
  else if (non_zero_cnt > 1)
  {
    float min_ratio = 10.0;
    for (uint8_t i = 0; i < ACQ_COUNT; i++)
    {
      if ((acq_histogram[i] > 0) && (i != max_pos))
      {
        float ratio = (float)max_val / (float)acq_histogram[i];
        if (ratio < min_ratio)
          min_ratio = ratio;
      }
    }
    if (min_ratio > 1.7f)
    {
      channel->acq_data.hist_ratio = min_ratio;
      channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_DONE;
      channel->acq_data.found_freq_offset_hz = -ACQ_SEARCH_FREQ_HZ + max_pos * ACQ_SEARCH_STEP_HZ;
    }
  }
  
  if (channel->acq_data.state == GPS_ACQ_FREQ_SEARCH_DONE)
  {
    printf("PRN=%d FINAL FREQ=%df\n", channel->prn, channel->acq_data.found_freq_offset_hz);
  }
}


uint32_t* acquisition_get_hist(void)
{
  return acq_histogram;
}

void acquisition_add_to_phase_hist(gps_ch_t* channel, uint8_t idx)
{
  if (idx < ACQ_PHASE1_HIST_SIZE)
    channel->acq_data.code_phase_histogram[idx]++;
}

uint8_t* acquisition_get_phase_hist(gps_ch_t* channel)
{
  return channel->acq_data.code_phase_histogram;
}

float acquisition_phase_hist_average(gps_ch_t* channel)
{
  uint32_t summ = 0;
  uint8_t count = 0;
  for (uint8_t idx = 0; idx < ACQ_PHASE1_HIST_SIZE; idx++)
  {
    if (channel->acq_data.code_phase_histogram[idx] > 0)
    {
      summ += channel->acq_data.code_phase_histogram[idx];
      count++;
    }
  }
  
  return (float)summ / (float)count;
}

