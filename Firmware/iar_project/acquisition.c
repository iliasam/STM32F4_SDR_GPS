#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "acquisition.h"
#include "simulator.h"
#include "common_ram.h"
#include "config.h"

#define FREQ_SEARCH_POINTS_MAX_CNT		40

#define ACQ_SEARCH_FREQ_HZ		(7000)
#define ACQ_SEARCH_STEP_HZ		(500)

#define ACQ_COUNT		        (ACQ_SEARCH_FREQ_HZ * 2 / ACQ_SEARCH_STEP_HZ)

uint16_t acq_single_freq_phases[FREQ_SEARCH_POINTS_MAX_CNT];
uint8_t acq_single_freq_count = 0;

volatile uint8_t tmp_found_cnt = 0;

int16_t acquisition_process_channel(gps_ch_t* channel, uint8_t* data, uint8_t offset);
void acquisition_add_to_hist(uint16_t value, uint8_t idx);
void acquisition_process_hist(gps_ch_t* channel);
uint32_t acquisition_hist_find_second(uint32_t max_val);
uint32_t acquisition_hist_average(uint8_t unique_cnt);
void acquisition_freq_search(gps_ch_t* channel, uint8_t* data, uint8_t reset_flag);
uint32_t acquisition_hist_median(uint8_t unique_cnt);

void acquisition_single_freq_check_reset(void);
void acquisition_process_single_freq_data(gps_ch_t* channel, uint8_t points_cnt);
//****************************************************

void acquisition_process(gps_ch_t* channel, uint8_t* data)
{
  acquisition_process_channel(channel, data, 0);
}

int16_t acquisition_process_channel(gps_ch_t* channel, uint8_t* data, uint8_t offset)
{
  if (channel->acq_data.state == GPS_ACQ_NEED_FREQ_SEARCH)
  {
    acquisition_single_freq_check_reset();
    channel->acq_data.freq_index = 0;
    channel->acq_data.single_freq_length = 30;
    channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_RUN;
  }
  
  if (channel->acq_data.state == GPS_ACQ_FREQ_SEARCH_RUN)
  {
    acquisition_freq_search(channel, data, 0);
  }
  
  return 0;
}

void acquisition_single_freq_check_reset(void)
{
  memset(acq_single_freq_phases, 0, sizeof(acq_single_freq_phases));
  acq_single_freq_count = 0;
}

uint8_t acquisition_need_new_data(gps_ch_t* channel)
{
  return 1;
}

void acquisition_freq_search(gps_ch_t* channel, uint8_t* data, uint8_t reset_flag)
{
	uint8_t prn = channel->prn;
	gps_generate_prn_data(tmp_prn_data, prn);

	uint8_t idx = channel->acq_data.freq_index;
	//int16_t freq_offset_hz = -ACQ_SEARCH_FREQ_HZ + idx * ACQ_SEARCH_STEP_HZ;
	int16_t freq_offset_hz = 2000;

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
                  channel->acq_data.freq_index = 0;
                  //channel->acq_data.state = GPS_ACQ_FREQ_SEARCH_DONE;
		}
	}
}

int compare_freq1(const void * a, const void * b)
{
	return (*(uint16_t*)a - *(uint16_t*)b);
}

void acquisition_process_single_freq_data(gps_ch_t* channel, uint8_t points_cnt)
{
  qsort(acq_single_freq_phases, points_cnt, sizeof(uint16_t), compare_freq1);
  uint8_t chain_items = 0;
  uint16_t max_chain_length = 0;
  uint8_t same_flag = 0;
  
  for (uint8_t i = 1; i < points_cnt; i++)
  {
    uint16_t diff = acq_single_freq_phases[i] - acq_single_freq_phases[i - 1];
    if (diff < 2)
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
    asm("nop");
    tmp_found_cnt++;
    //acq_histogram[channel->acq_data.freq_index] = max_chain_length;
  }
}




	
