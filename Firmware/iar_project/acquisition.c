#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "acquisition.h"
#include "simulator.h"
#include "common_ram.h"
#include "config.h"

#define FREQ_SEARCH_POINTS_MAX_CNT		25

uint32_t acq_histogram[ACQ_COUNT];
uint16_t acq_histogram_cnt = 0;

uint16_t acq_single_freq_phases[FREQ_SEARCH_POINTS_MAX_CNT];
uint8_t acq_single_freq_count = 0;


uint8_t acq_code_phase_histogram[ACQ_PHASE1_HIST_SIZE];

int16_t acquisition_process_channel(gps_ch_t* channel, uint8_t* data, uint8_t offset);
void acquisition_freq_search(gps_ch_t* channel, uint8_t* data, uint8_t reset_flag);


void acquisition_code_phase_search(gps_ch_t* channel, uint8_t* data);
void acquisition_code_phase_search2(gps_ch_t* channel, uint8_t* data);
void acquisition_add_to_phase_hist(uint8_t idx);
float acquisition_phase_hist_average(void);

void acquisition_single_freq_check_reset(void);
void acquisition_process_single_freq_data(gps_ch_t* channel, uint8_t points_cnt);
void acquisition_process_code_search2_data(gps_ch_t* channel, uint8_t points_cnt);

//****************************************************

void acquisition_process(gps_ch_t* channel, uint8_t* data)
{
  acquisition_process_channel(channel, data, 0);
}

void acquisition_single_freq_check_reset(void)
{
  memset(acq_single_freq_phases, 0, sizeof(acq_single_freq_phases));
  acq_single_freq_count = 0;
}

int16_t acquisition_process_channel(gps_ch_t* channel, uint8_t* data, uint8_t offset)
{
  //uint8_t prn = channel->prn;
  
  if (channel->acq_data.state == GPS_ACQ_DONE)
    return 0;
  
  if (channel->acq_data.state == GPS_ACQ_NEED_FREQ_SEARCH)
  {
    if (channel->acq_data.given_freq_offset_hz != 0)
    {
      channel->acq_data.found_freq_offset_hz = channel->acq_data.given_freq_offset_hz;
      channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_DONE;
      return 0;
    }
    
    acquisition_single_freq_check_reset();
    channel->acq_data.freq_index = 0;
    channel->acq_data.single_freq_length = 10;
    channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_RUN;
  }
  
  if (channel->acq_data.state == GPS_ACQ_FREQ_SEARCH_RUN)
  {
    acquisition_freq_search(channel, data, 0);
  }
  
  //CODE search
  
  if (channel->acq_data.state == GPS_ACQ_FREQ_SEARCH_DONE)
  {
    memset(acq_code_phase_histogram, 0, sizeof(acq_code_phase_histogram));
    channel->acq_data.code_search_start = 0;
    channel->acq_data.code_search_stop = 2 * PRN_LENGTH;
    //Histogram resolution is 64 steps
    channel->acq_data.code_hist_step = ACQ_PHASE1_HIST_STEP;
    
    channel->acq_data.state = GPS_ACQ_CODE_PHASE_SEARCH1;
  }
  
  if (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH1_DONE)
  {
    memset(acq_code_phase_histogram, 0, sizeof(acq_code_phase_histogram));
    
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
    channel->acq_data.state = GPS_ACQ_CODE_PHASE_SEARCH2;
    return 0;
  }
  
  if (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH2_DONE)
  {
    memset(acq_code_phase_histogram, 0, sizeof(acq_code_phase_histogram));
    
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
    
    channel->acq_data.state = GPS_ACQ_CODE_PHASE_SEARCH3;
    return 0;
  }
  
  if ((channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH1) || 
      (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH2) ||
        (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH3) )
  {
    acquisition_code_phase_search(channel, data);
    return 0;
  }
  
  return 0;
}

uint8_t acquisition_need_new_data(gps_ch_t* channel)
{
  if (channel->acq_data.state == GPS_ACQ_FREQ_SEARCH_RUN)
  {
    return 1;
  }
  else if (channel->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH1)
    return 1;
  else
    return 1;
}

void acquisition_code_phase_search2(gps_ch_t* channel, uint8_t* data)
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
  

  acq_single_freq_phases[acq_single_freq_count] = best_code_phase;
  acq_single_freq_count++;
  
  if (acq_single_freq_count > 10)
  {
    if (acq_single_freq_count > (FREQ_SEARCH_POINTS_MAX_CNT - 2))
      acquisition_single_freq_check_reset();
  }
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
 
  //Index in histogram
  uint8_t index = ((best_code_phase - channel->acq_data.code_search_start) / channel->acq_data.code_hist_step);
  acquisition_add_to_phase_hist(index);
  
  uint8_t max_val = 0;
  uint8_t max_pos_idx = 0;
  //add +2 too remove problem with rounding
  uint16_t hist_length = (channel->acq_data.code_search_stop + 2 - channel->acq_data.code_search_start) / 
    channel->acq_data.code_hist_step;
  uint8_t unique_elements = 0;
  for (uint8_t idx = 0; idx < hist_length; idx++)
  {
    if (acq_code_phase_histogram[idx] > max_val)
    {
      max_val = acq_code_phase_histogram[idx];
      max_pos_idx = idx;
    }
    if (acq_code_phase_histogram[idx] > 0)
      unique_elements++;
  }
  if (max_val < 2)
    return;
  
  float hist_avr_val = acquisition_phase_hist_average();
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
  gps_generate_prn_data2(channel, tmp_prn_data, 0);
  int16_t freq_offset_hz = channel->acq_data.given_freq_offset_hz;
  
  gps_shift_to_zero_freq(data, (uint8_t*)tmp_data_i, (uint8_t*)tmp_data_q, IF_FREQ_HZ + freq_offset_hz);
  uint16_t avr_val;
  uint16_t best_phase1 = 0;
  uint16_t res_val = correlation_search(tmp_prn_data, tmp_data_i, tmp_data_q, 0, (PRN_LENGTH * 2), &avr_val, &best_phase1);
}

void acquisition_freq_search(gps_ch_t* channel, uint8_t* data, uint8_t reset_flag)
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
    
    acquisition_single_freq_check_reset();
    
    channel->acq_data.freq_index++;
    if (channel->acq_data.freq_index >= ACQ_COUNT)
    {
      channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_DONE;
    }
  }
}

int compare_freq1_function(const void * a, const void * b)
{
  return (*(uint16_t*)a - *(uint16_t*)b);
}

//Used for frequency search
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
  
  if (max_chain_length >= 3)
  {
    acq_histogram[channel->acq_data.freq_index] = max_chain_length;
  }
}


uint32_t* acquisition_get_hist(void)
{
  return acq_histogram;
}

void acquisition_add_to_phase_hist(uint8_t idx)
{
  if (idx < ACQ_PHASE1_HIST_SIZE)
    acq_code_phase_histogram[idx]++;
}

uint8_t* acquisition_get_phase_hist(void)
{
  return acq_code_phase_histogram;
}

float acquisition_phase_hist_average(void)
{
  uint32_t summ = 0;
  uint8_t count = 0;
  for (uint8_t idx = 0; idx < ACQ_PHASE1_HIST_SIZE; idx++)
  {
    if (acq_code_phase_histogram[idx] > 0)
    {
      summ += acq_code_phase_histogram[idx];
      count++;
    }
  }
  
  return (float)summ / (float)count;
}

