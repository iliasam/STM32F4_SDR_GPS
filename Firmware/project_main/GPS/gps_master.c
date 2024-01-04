#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
//#include "delay_us_timer.h"
#include "signal_capture.h"
#include "uart_comm.h"
#include "gps_master.h"
#include "config.h"
#include "acquisition.h"
#include "math.h"
#include "time.h"

#if (ENABLE_RTCM_SEND)
  #include "obs_publish.h"
#endif


#define GPS_OFFSET_TIME_MS      (68.802)
#define CLIGHT		        299792458.0      /* speed of light (m/s) */
#define CLIGHT_NORM	        (299792458.0 / PRN_SPEED_HZ)
#define SUBFRAME_LENGTH_MS	(6000)


#if (ENABLE_RTCM_SEND)
  obsd_t obsd[GPS_SAT_CNT];
#endif

//******************************************************************
//Flag that at least one sat. need acquisition
uint8_t gps_common_need_acq = 1;

uint8_t gps_start_flag = 1;

volatile static uint32_t diff;

void gps_master_nav_handling(gps_ch_t* channels);
void gps_master_transmit_obs(gps_ch_t* channels);

//****************************************************
//****************************************************

void gps_master_handling(gps_ch_t* channels, uint8_t index)
{
  gps_ch_t* curr_ch = channels;
  
  if (gps_start_flag)
  {
    gps_start_flag = 0;
    acquisition_start_channel(&channels[0]);
  }
  
  gps_common_need_acq = 0;
  uint8_t need_f_search = 0;
  uint8_t code_search3_cnt = 0;
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (curr_ch->acq_data.state != GPS_ACQ_DONE)
      gps_common_need_acq = 1;
    if (curr_ch->acq_data.state < GPS_ACQ_FREQ_SEARCH_DONE)
      need_f_search = 1;
    if (curr_ch->acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH2_DONE)
      code_search3_cnt++;
    curr_ch++;
  }
  
  //Starting code search - one by one
  curr_ch = channels;
  if (gps_common_need_acq == 1)
  {
    for (uint8_t i = 0; i < (GPS_SAT_CNT - 1); i++)
    {
      if ((channels[i].acq_data.state == GPS_ACQ_FREQ_SEARCH_DONE) &&
          (channels[i + 1].acq_data.state == GPS_ACQ_NEED_FREQ_SEARCH))
      {
        //Start freq. search at the new channel
        acquisition_start_channel(&channels[i+1]);
        return;
      }
    }
  }
  
  //Start acq. code search for all possible
  if ((need_f_search == 0) && (gps_common_need_acq == 1))
  {
    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
      if (channels[i].acq_data.state == GPS_ACQ_FREQ_SEARCH_DONE)
        acquisition_start_code_search_channel(&channels[i]);
      
      // Start simultaneously code search 3
      if (code_search3_cnt == GPS_SAT_CNT)
      {
        acquisition_start_code_search3_channel(&channels[i]);
      }
    }
  }
  
  curr_ch = channels;
  if (gps_common_need_acq == 0)
  {
    //acq is done at all sats. -> start tracking!
    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
      if (curr_ch->tracking_data.state == GPS_TRACKNG_IDLE)
        curr_ch->tracking_data.state = GPS_NEED_PRE_TRACK; //start tracking for this sat.
      curr_ch++;
    }
  }
  
  if (index == 0xFF) //dummy tracking
    gps_master_nav_handling(channels);
}



void gps_master_nav_handling(gps_ch_t* channels)
{
  //Set first subframe detection time for all channels
  uint8_t has_subframe_time_cnt = 0;
  uint8_t first_time_not_set_cnt = 0;
  
  uint8_t ref_idx = 0;
  uint32_t min_subframe_time = 0xFFFFFFFF;
  uint32_t max_subframe_time = 0;
  
  uint16_t min_subframe_cnt = 0xFFFF;
  uint16_t max_subframe_cnt = 0;
  
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].nav_data.last_subframe_time != 0)
      has_subframe_time_cnt++;
    
    if (channels[i].nav_data.first_subframe_time == 0)
      first_time_not_set_cnt++;
    
    if (channels[i].nav_data.last_subframe_time < min_subframe_time)
    {
      min_subframe_time = channels[i].nav_data.last_subframe_time;
      ref_idx = i;
    }
    
    if (channels[i].nav_data.last_subframe_time > max_subframe_time)
    {
      max_subframe_time = channels[i].nav_data.last_subframe_time;
    }
    
    if (channels[i].nav_data.subframe_cnt < min_subframe_cnt)
      min_subframe_cnt = channels[i].nav_data.subframe_cnt;
    
    if (channels[i].nav_data.subframe_cnt > max_subframe_cnt)
      max_subframe_cnt = channels[i].nav_data.subframe_cnt;
  }
  
  if (min_subframe_time == 0)
    return;
  
  uint32_t diff_ms = max_subframe_time - min_subframe_time;
  if (diff_ms > 100)//wait untill all subframes of this epoch get received (they will have similiar times)
    return;
  
  if ((has_subframe_time_cnt == GPS_SAT_CNT) && (first_time_not_set_cnt == GPS_SAT_CNT))
  {
    //This works once!
    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
      channels[i].nav_data.first_subframe_time = channels[i].nav_data.last_subframe_time;
      channels[i].nav_data.subframe_cnt = 0;
    }
  }
  
  //************************************************
  //Pseudorange calculation
  
  if (channels[0].nav_data.first_subframe_time == 0)
    return;
  
  uint32_t ref_time_ms = channels[ref_idx].nav_data.first_subframe_time + max_subframe_cnt * SUBFRAME_LENGTH_MS;
  
  //Detecting code phase swap - may happen more longer after new subframe event
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].tracking_data.code_phase_swap_flag && channels[i].nav_data.new_subframe_flag)
    {
      channels[i].nav_data.new_subframe_flag = 0;
      channels[i].tracking_data.code_phase_swap_flag = 0;
    }
    
    float diff_f = fabs(channels[i].tracking_data.old_code_phase_fine - channels[i].tracking_data.code_phase_fine);
    if (diff_f > ((float)PRN_LENGTH * 16.0f / 2.0f))//half of range
    {
      //code phase swap detected
      channels[i].tracking_data.code_phase_swap_flag = 1;
    }
    
    channels[i].tracking_data.old_code_phase_fine = channels[i].tracking_data.code_phase_fine;
  }
  
  uint32_t curr_tick_time = signal_capture_get_packet_cnt();
  //Time from last subframe of REF sat.
  int32_t time_diff_ms = (int32_t)curr_tick_time - (int32_t)channels[ref_idx].nav_data.last_subframe_time;
  if ((time_diff_ms < 0))
    time_diff_ms = time_diff_ms % SUBFRAME_LENGTH_MS;
  
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    int32_t diff_prn = channels[i].nav_data.last_subframe_time - ref_time_ms;
    double diff_time_ms = (double)diff_prn + channels[i].tracking_data.code_phase_fine / ((double)PRN_LENGTH * 16.0f);
    //Comepensating state when code phase swapped, but new subframe has not come yet
    if (channels[i].tracking_data.code_phase_swap_flag == 1)
    {
      double corr_ms = 1.0f;
      if (channels[i].tracking_data.if_freq_offset_hz < 0.0f)
        corr_ms = -1.0f;
      //Add correction
      diff_time_ms = diff_time_ms - corr_ms;
    }
    double pseudo_range_m = (GPS_OFFSET_TIME_MS + diff_time_ms) * CLIGHT_NORM;
    channels[i].obs_data.pseudorange_m = pseudo_range_m;
    channels[i].obs_data.tow_s = channels[ref_idx].eph_data.tow_gpst + ((float)time_diff_ms / PRN_SPEED_HZ);
  }
  
  gps_master_transmit_obs(channels);
}


void gps_master_transmit_obs(gps_ch_t* channels)
{
  static uint32_t prev_obs_send_time_ms = 0;
  
  if (uart_prim_is_busy())
    return;
  
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].eph_data.received_mask & 0x7 == 0x7) //subrame 1,2,3
    {
      channels[i].eph_data.received_mask &= ~0x7;//clear mask
      sendrtcmnav(&channels[i]);
      return;//wait for UART TX
    }
  }
  
  uint32_t curr_time_ms = signal_capture_get_packet_cnt();
  if ((curr_time_ms - prev_obs_send_time_ms) > 200)
  {
    prev_obs_send_time_ms = curr_time_ms;
    sdrobs2obsd(channels, GPS_SAT_CNT, obsd);
    sendrtcmobs(obsd, GPS_SAT_CNT);
  }
}

uint8_t gps_master_need_freq_search(gps_ch_t* channels)
{
  uint8_t need_search = 0;
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels->acq_data.state < GPS_ACQ_FREQ_SEARCH_DONE)
      need_search = 1;
    channels++;
  }
  return need_search;
}

// All in mode > GPS_ACQ_CODE_PHASE_SEARCH2
uint8_t gps_master_is_code_search3(gps_ch_t* channels)
{
  uint8_t test_cnt = 0;
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels->acq_data.state > GPS_ACQ_CODE_PHASE_SEARCH2)
      test_cnt++;
    channels++;
  }
  return (test_cnt == GPS_SAT_CNT);
}

uint8_t gps_master_need_acq(void)
{
  return gps_common_need_acq;
}

void gps_master_test(gps_ch_t* channels)
{
  static uint8_t cnt = 0;
  if (uart_prim_is_busy())
    return;

  cnt++;
  if (cnt >= 4)
    cnt = 0;
  
  sendrtcmnav(&channels[cnt]);
}

