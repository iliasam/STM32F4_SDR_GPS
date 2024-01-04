#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "nav_data.h"
#include "nav_data_decode.h"
#include "signal_capture.h"
#include "config.h"
#include <math.h>

// Number of PRN codes in 1 navigation bit
#define CODES_IN_BIT	        20

#define GPS_WORDS_IN_SUBFRAME	10


//******************************************************************

const uint8_t gps_preamble[8] = { 1, 0, 0, 0, 1, 0, 1, 1}; // L1CA preamble

/// Polarity of single PRNs are saved here during one analyse
/// This buffer if common, becase there is only one real channel is processed
uint8_t gps_channel_tmp_nav_data[TRACKING_CH_LENGTH];

/// Timestamp of PRN counter in channel zero index, common
uint32_t gps_channel_tmp_start_time_ticks = 0;

void gps_nav_data_bits_extraction(gps_ch_t* channel, uint8_t new_short_bit, uint32_t meas_time);
void gps_nav_data_words_detection(gps_ch_t* channel, uint8_t new_bit);
uint8_t gps_nav_data_check_preamble(gps_ch_t* channel);
uint8_t gps_nav_data_check_preamble_inv(gps_ch_t* channel);
void gps_nav_data_save_word_data(gps_ch_t* channel);
uint8_t gps_nav_data_word_check_parity(gps_ch_t* channel);
void gps_nav_data_accurate_sync_detection(gps_ch_t* channel, int16_t* raw_values);
void gps_nav_data_update_subframe_time(gps_ch_t* channel);

//****************************************************
//****************************************************

/// Called at each PRN (when certain channel is active)
void gps_nav_data_analyse_new_code(gps_ch_t* channel, uint8_t index, int16_t new_i)
{
  static int16_t raw_ip_values[TRACKING_CH_LENGTH];
  
  if (index >= TRACKING_CH_LENGTH)
    return;
  
  uint8_t new_short_bit;
  if (new_i > 0)
    new_short_bit = 1;
  else
    new_short_bit = 0;
  
  if (channel->nav_data.inv_polarity_flag)
    new_short_bit = new_short_bit ^ 1;
  
  //Collect tmp. data for future processing
  gps_channel_tmp_nav_data[index] = new_short_bit;
  raw_ip_values[index] = new_i;
  
  uint32_t curr_tick_time = signal_capture_get_packet_cnt();
  if (index == 0)
  {
    gps_channel_tmp_start_time_ticks = curr_tick_time;
  }
  
  //Exatracting navigation data here
  if (channel->nav_data.period_sync_ok_flag == 1)
  {
    gps_nav_data_bits_extraction(channel, new_short_bit, curr_tick_time);
  }
  
  
  if (index < (TRACKING_CH_LENGTH - 1))
    return;
  
  // Now (index == (TRACKING_CH_LENGTH - 1)) - last PRN
  // Detecting sign switching
  
  // Count number of sign switching
  uint8_t swith_counter = 0;//sign switch
  uint8_t pol_old = gps_channel_tmp_nav_data[0];
  uint8_t pol;
  uint8_t pol_change_pos = 0;
  for (uint8_t i = 1; i < TRACKING_CH_LENGTH; i++)
  {
    pol = gps_channel_tmp_nav_data[i];
    if (pol != pol_old)
    {
      swith_counter++;
      pol_change_pos = i;
    }
    pol_old = pol;
  }
  
  if (swith_counter == 1) //only one bit switch
  {
    //Bit changing detected
    uint32_t swap_timestamp = gps_channel_tmp_start_time_ticks + pol_change_pos;
    uint32_t diff = swap_timestamp - channel->nav_data.old_swap_time;
    uint8_t reminder = diff % CODES_IN_BIT;
    if ((reminder < 2) || (reminder == (CODES_IN_BIT - 1))) //diff=0,1,19
    {
      //right period detected
      if (channel->nav_data.right_period_cnt < 10)
        channel->nav_data.right_period_cnt++;
      if (channel->nav_data.right_period_cnt > 8)
        channel->nav_data.period_sync_ok_flag = 1;
    }
    else
    {
      if (channel->nav_data.right_period_cnt > 0)
        channel->nav_data.right_period_cnt--;
      
      if (channel->nav_data.right_period_cnt < 3)
        channel->nav_data.period_sync_ok_flag = 0;
    }
    
    //sprintf(tmp_txt, "SWAP=%ld\n", swap_timestamp);
    //OutputDebugString((LPCSTR)tmp_txt);
    
    channel->nav_data.old_swap_time = swap_timestamp;
    
    //Two values has one sign, and another two - another sign
    if (channel->nav_data.period_sync_ok_flag && (pol_change_pos == 2))
    {
      gps_nav_data_accurate_sync_detection(channel, raw_ip_values);
    }
  }
}

// Called at the end of channel processing
// Used to calculate navv data bit swaps with more precision
// "raw_values" - 4 last IP values of this channel
void gps_nav_data_accurate_sync_detection(gps_ch_t* channel, int16_t* raw_values)
{
  //index in "raw_values" - we need to find is it 1 or 2 
  uint8_t swap_pos = 0;
  
  if (abs(raw_values[1]) > abs(raw_values[0]))
  {
    //Val1 is bigger than Val0, so Val1 can't be swap, looks like that swap is at val2
    //It is hard to use such data;
    return;
  }
  
  if (raw_values[3] == 0)//protect from divide to zero
    return;
  
  float whole_ratio = (float)abs(raw_values[0]) / (float)abs(raw_values[3]);
  if ((whole_ratio > 1.5f) || (whole_ratio < 0.7f))
    return;//signal differ significantly from start to end
  
  int16_t code_phase_prn = (int16_t)channel->tracking_data.code_phase_fine / 16;//0-1023
  if ((code_phase_prn < 0) | (code_phase_prn > PRN_LENGTH))
    return;
  
  uint16_t diff1 = 0;
  uint16_t diff2 = 0;
  if ((code_phase_prn < (PRN_LENGTH / 4)) || (code_phase_prn > (PRN_LENGTH * 3 / 4)))
  {
    if (raw_values[1] == 0)//protect from divide to zero
      return;
    
    float ratio_jump = (float)abs(raw_values[0]) / (float)abs(raw_values[1]);
    if ((ratio_jump > 1.5f) || (ratio_jump < 0.7f))
      return;//too big defference
    
    if (code_phase_prn < (PRN_LENGTH / 4))
      swap_pos = 2;
    else
      swap_pos = 1;
  }
  else
  {
    diff1 = abs(raw_values[0] - raw_values[1]);
    diff2 = abs(raw_values[2] - raw_values[3]);
    
    float ratio;
    if (diff1 > diff2)
    {
      if (diff2 == 0)
        return;
      ratio = (float)diff1 / (float)diff2;
      if (ratio < 2.5f)
        return;//no strong signal
      swap_pos = 1;
    }
    else
    {
      if (diff1 == 0)
        return;
      ratio = (float)diff2 / (float)diff1;
      if (ratio < 2.5f)
        return;//no strong signal
      swap_pos = 2;
    }
  }
  
  if (swap_pos == 0)
    return;
  
  uint32_t swap_timestamp = gps_channel_tmp_start_time_ticks + swap_pos;
  channel->nav_data.accurate_swap_time = swap_timestamp % CODES_IN_BIT;
  channel->nav_data.accurate_swap_ok = 1;
}

//new_short_bit - one PRN code bit (1ms), 0/1
//meas_time - main PRN counter
// Called at each PRN (when certain channel is active)
void gps_nav_data_bits_extraction(gps_ch_t* channel, uint8_t new_short_bit, uint32_t meas_time)
{
  uint32_t diff = meas_time - channel->nav_data.old_swap_time;
  uint8_t reminder = diff % CODES_IN_BIT;
  
  if (reminder < channel->nav_data.old_reminder)//"reminder" overflow, new nav. bit time zone
  {
    //end of data bit happend earier, find sign of ended bit
    
    uint8_t nav_data_bit;
    if (channel->nav_data.last_bit_pos_cnt > channel->nav_data.last_bit_neg_cnt)
      nav_data_bit = 1;
    else
      nav_data_bit = 0;
    
    
    gps_nav_data_words_detection(channel, nav_data_bit);
    
    //prepare for next nav. data bit
    channel->nav_data.last_bit_pos_cnt = 0;
    channel->nav_data.last_bit_neg_cnt = 0;
  }
  
  //Counting pos. and neg. bits in one nav. data period - 20ms
  if (new_short_bit)
    channel->nav_data.last_bit_pos_cnt++;
  else
    channel->nav_data.last_bit_neg_cnt++;
  
  channel->nav_data.old_reminder = reminder;
}

// Try to collect gps "word" from received bits
// new_bit - bit on nav data (20ms long)
void gps_nav_data_words_detection(gps_ch_t* channel, uint8_t new_bit)
{
  if (channel->nav_data.word_cnt == 0) //no preamble sync
  {
    //Shift bits
    for (uint8_t i = 1; i < GPS_NAV_WORD_LENGTH; i++)
    {
      channel->nav_data.word_buf[i - 1] = channel->nav_data.word_buf[i];
    }
    //Write new to the end of the array
    channel->nav_data.word_buf[GPS_NAV_WORD_LENGTH - 1] = new_bit;
    
    if (gps_nav_data_check_preamble(channel))
    {
      gps_nav_data_save_word_data(channel);
      channel->nav_data.word_cnt = 1;
      channel->nav_data.word_bit_cnt = 0;
      channel->nav_data.inv_preabmle_cnt = 0;
      
      //char tmp_txt[100];
      //sprintf(tmp_txt, "PREAMBLE!\n");
      //OutputDebugString((LPCSTR)tmp_txt);
    }
    
    if ((channel->nav_data.polarity_found == 0) && 
        (channel->nav_data.word_cnt == 0))
    {
      if (gps_nav_data_check_preamble_inv(channel))
        channel->nav_data.inv_preabmle_cnt++;
      if (channel->nav_data.inv_preabmle_cnt >= 2)
      {
        channel->nav_data.inv_polarity_flag = 1;
      }
    }
    
    //Try to redetect polarity if phase 180deg rotation happened
    if (channel->nav_data.polarity_found)
    {
      //Time from last correct word received
      uint32_t word_diff_ms = signal_capture_get_packet_cnt() - channel->nav_data.word_detection_timestamp;
      if (word_diff_ms > (1000 * 6 * 2))//2 subframes
      {
        channel->nav_data.word_detection_timestamp = signal_capture_get_packet_cnt();//reset diff
        channel->nav_data.polarity_found = 0;
        channel->nav_data.inv_polarity_flag = 0;
      }
    }
  }
  else
  {
    //Have preamble sync
    channel->nav_data.word_buf[channel->nav_data.word_bit_cnt] = new_bit;
    channel->nav_data.word_bit_cnt++;
    if (channel->nav_data.word_bit_cnt >= GPS_NAV_WORD_LENGTH)
    {
      //Word is collected
      if (gps_nav_data_word_check_parity(channel))
      {
        channel->nav_data.word_cnt_test++;
        
        gps_nav_data_save_word_data(channel);
        channel->nav_data.word_cnt++;
        channel->nav_data.word_bit_cnt = 0;
        channel->nav_data.word_detection_timestamp = signal_capture_get_packet_cnt();
        if (channel->nav_data.polarity_found == 0)
        {
          channel->nav_data.polarity_found = 1;
          printf("PRN=%d POLAR. FOUND\n", channel->prn);
        }
        
        //Full subframe collected
        if (channel->nav_data.word_cnt == GPS_WORDS_IN_SUBFRAME)
        {
          uint8_t subframe_id = gps_nav_data_decode_subframe(channel);
          gps_nav_data_update_subframe_time(channel);
          channel->nav_data.word_cnt = 0;
          channel->nav_data.new_subframe_flag = 1;
          //Clear word buffer to protect from false preamble detection from old data
          memset(channel->nav_data.word_buf, 0, GPS_NAV_WORD_LENGTH);
        }
      }
      else
      {
        channel->nav_data.word_cnt = 0;
        memset(channel->nav_data.word_buf, 0, GPS_NAV_WORD_LENGTH);
      }
    }
  }
}

//Called when full subframe is collected
// This function is usally called after bit swap happened
// and a new subfrme begins
// Timestamp is always set AFTER the bit swap (subframes border)
void gps_nav_data_update_subframe_time(gps_ch_t* channel)
{
  if (channel->nav_data.accurate_swap_ok == 0)
    return;
  
  uint32_t curr_tick_time = signal_capture_get_packet_cnt();
  
  //Last full accurate nav. bit swap time (in common PRN steps) (round + add accur. part)
  uint32_t accur_swap_time = (curr_tick_time / CODES_IN_BIT) * CODES_IN_BIT + channel->nav_data.accurate_swap_time;
  int32_t diff_accur = curr_tick_time - accur_swap_time;
  
  if (diff_accur < 0)
  {
    //Looks like that swap detection was not accurate and we entered here
    //before current nav. bit really ended
    accur_swap_time = accur_swap_time - CODES_IN_BIT;
    diff_accur = curr_tick_time - accur_swap_time;
  }
  
  channel->nav_data.subframe_cnt++;
  channel->nav_data.last_subframe_time = accur_swap_time;
}

//Check that preamble is at the start of channel->nav_data.word_buf[]
uint8_t gps_nav_data_check_preamble(gps_ch_t* channel)
{
  uint8_t summ = 0;
  for (uint8_t i = 0; i < sizeof(gps_preamble); i++)
  {
    if (channel->nav_data.word_buf[i] == gps_preamble[i])
      summ++;
    else
      break;
  }
  return (summ == sizeof(gps_preamble));
}

//Check that invertedd preamble is at the start of channel->nav_data.word_buf[]
uint8_t gps_nav_data_check_preamble_inv(gps_ch_t* channel)
{
  uint8_t summ = 0;
  for (uint8_t i = 0; i < sizeof(gps_preamble); i++)
  {
    if (channel->nav_data.word_buf[i] == (gps_preamble[i] ^ 1))
      summ++;
    else
      break;
  }
  return (summ == sizeof(gps_preamble));
}

//Save last word to the subframee data array
void gps_nav_data_save_word_data(gps_ch_t* channel)
{
  uint16_t curr_bit = channel->nav_data.word_cnt * GPS_NAV_WORD_LENGTH;
  
  uint8_t cur_byte;
  uint8_t bit_in_byte;
  for (uint8_t i = 0; i < GPS_NAV_WORD_LENGTH; i++)
  {
    cur_byte = curr_bit / 8;
    bit_in_byte = curr_bit & 7;
    
    if (channel->nav_data.word_buf[i] == 1)
      channel->nav_data.subframe_data[cur_byte] |= (1 << bit_in_byte);
    else
      channel->nav_data.subframe_data[cur_byte] &= ~(1 << bit_in_byte);
    
    curr_bit++;
  }
  channel->nav_data.old_D29 = channel->nav_data.word_buf[29 - 1];
  channel->nav_data.old_D30 = channel->nav_data.word_buf[30 - 1];
}

// Taken from http://www.aholme.co.uk/GPS/SRC/2013/C++/channel.cpp
// Check word parity
uint8_t gps_nav_data_word_check_parity(gps_ch_t* channel)
{
  uint8_t parity[6];
  uint8_t D29 = channel->nav_data.old_D29;
  uint8_t D30 = channel->nav_data.old_D30;
  
  //uint8_t *d = channel->nav_data.word_buf - 1;
  uint8_t *d = &channel->nav_data.word_buf[0] - 1;
  for (uint8_t i = 1; i < 25; i++) //invert data
    d[i] ^= D30;
  
  parity[0] = D29 ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23];
  parity[1] = D30 ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21] ^ d[24];
  parity[2] = D29 ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22];
  parity[3] = D30 ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23];
  parity[4] = D30 ^ d[1] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[21] ^ d[22] ^ d[24];
  parity[5] = D29 ^ d[3] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[11] ^ d[13] ^ d[15] ^ d[19] ^ d[22] ^ d[23] ^ d[24];
  if (memcmp(d + 25, parity, 6) == 0)
    return 1;
  else
    return 0;
}
