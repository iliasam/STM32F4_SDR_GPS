#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "acquisition.h"
#include "simulator.h"
#include "common_ram.h"
#include "config.h"

#define ACQ_SEARCH_FREQ_HZ		(7000)
#define ACQ_SEARCH_STEP_HZ		(500)

#define ACQ_COUNT		        (ACQ_SEARCH_FREQ_HZ * 2 / ACQ_SEARCH_STEP_HZ)



uint32_t acq_histogram[ACQ_COUNT];
uint16_t acq_histogram_cnt = 0;

int16_t acquisition_process_channel(gps_ch_t* channel, uint8_t* data, uint8_t offset);
void acquisition_add_to_hist(uint16_t value, uint8_t idx);
void acquisition_process_hist(gps_ch_t* channel);
uint32_t acquisition_hist_find_second(uint32_t max_val);
uint32_t acquisition_hist_average(uint8_t unique_cnt);
void acquisition_freq_search(gps_ch_t* channel, uint8_t* data, uint8_t reset_flag);
uint32_t acquisition_hist_median(uint8_t unique_cnt);

//****************************************************

void acquisition_process(gps_ch_t* channel, uint8_t* data)
{
  acquisition_process_channel(channel, data, 0);
  acquisition_process_hist(channel);
}

int16_t acquisition_process_channel(gps_ch_t* channel, uint8_t* data, uint8_t offset)
{
  //uint8_t prn = channel->prn;
  
  if (channel->acq_data.state == GPS_ACQ_NEED_FREQ_SEARCH)
  {
    channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_RUN;
  }
  
  if (channel->acq_data.state == GPS_ACQ_FREQ_SEARCH_RUN)
  {
    if (channel->acq_data.freq_switch_flag == 0)
      acquisition_freq_search(channel, data, 1);//reset
    else
      acquisition_freq_search(channel, data, 0);
  }
  
  return 0;
}

uint8_t acquisition_need_new_data(gps_ch_t* channel)
{
	if (channel->acq_data.freq_switch_flag == 0)
		return 1;
	else
		return 0;
}

void acquisition_freq_search(gps_ch_t* channel, uint8_t* data, uint8_t reset_flag)
{
  static int16_t best_offset_hz = 0;//to delete
  static uint16_t best_value = 0;
  static uint16_t best_diff = 0;
  static uint8_t best_index = 0;
  
  char tmp_txt[100];
  
  uint8_t prn = channel->prn;
  
  if (reset_flag)
  {
    channel->acq_data.freq_switch_flag = 1;
    channel->acq_data.freq_index = 0;
    
    best_offset_hz = 0;//to delete
    best_value = 0;
    best_diff = 0;
    best_index = 0;
  }
  
  if (channel->acq_data.freq_switch_flag == 0)
    return;
  
  gps_generate_prn_data(tmp_prn_data, prn);
  
  uint8_t idx = channel->acq_data.freq_index;
  int16_t freq_offset_hz = -ACQ_SEARCH_FREQ_HZ + idx * ACQ_SEARCH_STEP_HZ;
  
  gps_shift_to_zero_freq(data, (uint8_t*)tmp_data_i, (uint8_t*)tmp_data_q, IF_FREQ_HZ + freq_offset_hz);
  uint16_t avr_val;
  uint16_t res_val = correlation_search(tmp_prn_data, tmp_data_i, tmp_data_q, &avr_val);
  
  if (avr_val == 0)
    avr_val = 1;
  
  if (res_val > best_value)
  {
    best_diff = res_val - avr_val;
    best_value = res_val;
    best_offset_hz = freq_offset_hz;
    best_index = idx;
  }
  
  channel->acq_data.freq_index++;
  if (channel->acq_data.freq_index >= ACQ_COUNT)
  {
    //All frequencies were analysed
    channel->acq_data.freq_switch_flag = 0;
    
    if (best_value > 400)
    {
      acquisition_add_to_hist(best_diff, best_index);
    }
  }
}



void acquisition_process_hist(gps_ch_t* channel)
{
  if (channel->acq_data.state != GPS_ACQ_FREQ_SEARCH_RUN)
    return;
  
  if (channel->acq_data.freq_switch_flag != 0)
    return;
  
  channel->acq_data.hist_ratio = 0.0f;
  
  if (acq_histogram_cnt < 2)
  {
    return;
  }
  
  //Count non-zero elements + find maximum
  uint8_t unique_elements = 0;
  uint32_t max_hist_val = 0;
  uint8_t max_freq_idx = 0;
  for (uint8_t idx = 0; idx < ACQ_COUNT; idx++)
  {
    if (acq_histogram[idx] > 1)
      unique_elements++;
    
    if (acq_histogram[idx] > max_hist_val)
    {
      max_hist_val = acq_histogram[idx];
      max_freq_idx = idx;
    }
  }
  
  if (unique_elements == 1)//single maximum, super good signal
  {
    if (max_hist_val > 800)
      channel->acq_data.hist_ratio = 10.0f;
    else
      channel->acq_data.hist_ratio = 1.0f;
  }
  else if (unique_elements == 2)
  {
    if (max_hist_val > 2000)
    {
      channel->acq_data.hist_ratio = 3.0;
    }
    else
    {
        uint32_t second_val = acquisition_hist_find_second(max_hist_val);
        float ratio = (float)max_hist_val / (float)second_val;
        channel->acq_data.hist_ratio = ratio;
    }
  }
  else
  {
    //unique_elements > 2
    uint32_t avr_val = acquisition_hist_average(unique_elements);
    //uint32_t avr_val = acquisition_hist_median(unique_elements);
    float ratio = (float)max_hist_val / (float)avr_val;
    channel->acq_data.hist_ratio = ratio;
  }
  
  if (channel->acq_data.hist_ratio > 2.4)
  {
    //channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_DONE;
    channel->acq_data.found_freq_offset_hz = -ACQ_SEARCH_FREQ_HZ + max_freq_idx * ACQ_SEARCH_STEP_HZ;
  }
}

uint32_t acquisition_hist_average(uint8_t unique_cnt)
{
	uint32_t summ = 0;
	for (uint8_t idx = 0; idx < ACQ_COUNT; idx++)
		summ += acq_histogram[idx];
	summ = summ / unique_cnt;
	return summ;
}

//Find secod (non-maximum) value in histogram
uint32_t acquisition_hist_find_second(uint32_t max_val)
{
	for (uint8_t idx = 0; idx < ACQ_COUNT; idx++)
	{
		if ((acq_histogram[idx] > 1) && (acq_histogram[idx] != max_val))
			return acq_histogram[idx];
	}
	return 1;//not found
}

void acquisition_add_to_hist(uint16_t value, uint8_t idx)
{
	acq_histogram[idx] += value;
	acq_histogram_cnt++;
}

uint32_t* acquisition_get_hist(void)
{
	return acq_histogram;
}

int compare(const void * a, const void * b)
{
	//return (*(uint32_t*)a - *(uint32_t*)b);
	return (*(uint32_t*)b - *(uint32_t*)a);
}

uint32_t acquisition_hist_median(uint8_t unique_cnt)
{
  uint32_t tmp_histogram[ACQ_COUNT];
  memcpy(tmp_histogram, acq_histogram, sizeof(acq_histogram));
  
  qsort(tmp_histogram, ACQ_COUNT, sizeof(uint32_t), compare);
  return tmp_histogram[unique_cnt / 2];
}



	

	
