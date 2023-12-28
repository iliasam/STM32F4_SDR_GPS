#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "nav_data.h"
#include "signal_capture.h"
#include "nav_data_decode.h"
#include "config.h"
#include <math.h>

// Number of PRN codes in 1 navigation bit
#define CODES_IN_BIT	20

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

//****************************************************
//****************************************************

/// Called at each PRN (when certain channel is active)
void gps_nav_data_analyse_new_code(gps_ch_t* channel, uint8_t index, int16_t new_i)
{
  if (index >= TRACKING_CH_LENGTH)
    return;
  
  uint8_t new_short_bit;
  if (new_i > 0)
    new_short_bit = 1;
  else
    new_short_bit = 0;
  
  if (channel->nav_data.inv_polarity_flag)
    new_short_bit = new_short_bit ^ 1;
  
  gps_channel_tmp_nav_data[index] = new_short_bit;
  
  uint32_t curr_tick_time = signal_capture_get_packet_cnt();
  if (index == 0)
  {
    gps_channel_tmp_start_time_ticks = curr_tick_time;
  }
  
  if (channel->nav_data.period_sync_ok_flag == 1)
  {
    gps_nav_data_bits_extraction(channel, new_short_bit, curr_tick_time);
  }
  
  
  if (index < (TRACKING_CH_LENGTH - 1))
    return;
  
  // Now (index == (TRACKING_CH_LENGTH - 1)) - last PRN
  
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
    
    channel->nav_data.old_swap_time = swap_timestamp;
  }
}

//new_short_bit - one code bit (1ms)
void gps_nav_data_bits_extraction(gps_ch_t* channel, uint8_t new_short_bit, uint32_t meas_time)
{
  uint32_t diff = meas_time - channel->nav_data.old_swap_time;
  uint8_t reminder = diff % CODES_IN_BIT;
  
  if (reminder < channel->nav_data.old_reminder)//"reminder" overflow
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
      
      printf("PREAMBLE!\n");
    }
    
    if ((channel->nav_data.polarity_found == 0) && (channel->nav_data.word_cnt == 0))
    {
      if (gps_nav_data_check_preamble_inv(channel))
        channel->nav_data.inv_preabmle_cnt++;
      if (channel->nav_data.inv_preabmle_cnt >= 2)
      {
        channel->nav_data.inv_polarity_flag = 1;
        //OutputDebugString((LPCSTR)"SET INV POLOAR\n");
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
      if (gps_nav_data_word_check_parity(channel))
      {
        gps_nav_data_save_word_data(channel);
        channel->nav_data.word_cnt++;
        channel->nav_data.word_bit_cnt = 0;
        if (channel->nav_data.polarity_found == 0)
        {
          channel->nav_data.polarity_found = 1;
          printf("POLAR. FOUND\n");
        }
        
        //Full subframe collected
        if (channel->nav_data.word_cnt == GPS_WORDS_IN_SUBFRAME)
        {
          uint8_t subframe_id = gps_nav_data_decode_subframe(channel);
          channel->nav_data.word_cnt = 0;
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
  
  uint8_t *d = &channel->nav_data.word_buf[0] - 1; //values below starts from 1, not from 0
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
