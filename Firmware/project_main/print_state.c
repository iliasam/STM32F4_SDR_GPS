//Printring state of GPS receiving

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "uart_comm.h"
#include "print_state.h"
#include "stdio.h"
#include "string.h"
#include <stdarg.h>


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PRINT_STATE_BUF_LENGTH          1024
#define PRINT_STATE_SEND_PERIOD_MS      150
#define PRINT_STATE_TRACKING_UPDATE_MS  300
#define ESC "\033"

#define clr_screen()        debug_print(ESC"[2J") //lear the screen, move to (1,1)
#define gotoxy(x,y)	    debug_print(ESC"[%d;%dH", y, x);
#define clr_line()	    debug_print(ESC"[2K");

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char print_state_tmp_buf[PRINT_STATE_BUF_LENGTH];
char print_state_tx_buf[PRINT_STATE_BUF_LENGTH];

char *print_state_wr_p = print_state_tmp_buf;
uint16_t print_state_cnt = 0;

/* Private function prototypes -----------------------------------------------*/
void print_state_channel(gps_ch_t* channel, uint8_t ch_idx);
void print_state_tracking_channel(gps_ch_t* channel, uint8_t ch_idx);
void print_state_acquisition_channel(gps_ch_t* channel, uint8_t ch_idx);
void print_state_update_acquisition_line(gps_ch_t* channel, uint8_t line_idx);

/* Private functions ---------------------------------------------------------*/

void debug_print(const char *str_frmt, ...)
{
  //No protections here!
  va_list args;
  va_start(args, str_frmt);
  uint16_t new_cnt = vsprintf(print_state_wr_p, str_frmt, args);
  print_state_wr_p += new_cnt;
  print_state_cnt += new_cnt;
  va_end(args);
}

//Need to be called periodically
//time_ms - current system time
void print_state_handling(uint32_t time_ms)
{
  static uint32_t prev_proc_time_ms = 0;
  if (print_state_cnt > 0)
  {
    if (uart_second_is_busy())
      return;
    
    uint32_t diff_ms = time_ms - prev_proc_time_ms;
    if (diff_ms < PRINT_STATE_SEND_PERIOD_MS)
      return;
    prev_proc_time_ms = time_ms;
    
    //Send data to the DMA
    memcpy(print_state_tx_buf, print_state_tmp_buf, print_state_cnt);
    uart_second_dma_send_data((uint8_t*)print_state_tx_buf, print_state_cnt);
    //TMP buffer is ready for a new data
    print_state_cnt = 0;
    print_state_wr_p = print_state_tmp_buf;
  }
}
//Called at start
void print_state_prepare(gps_ch_t* channels)
{
  clr_screen();
  gotoxy(1,1);
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    print_state_channel(&channels[i], i);
  }
  
  gotoxy(1,6);
  clr_line();
  debug_print("SYS RUNTIME: %.1fs\n", 0.0f);
}

//Called from gps_master when acq. is running
void print_state_update_acquisition(gps_ch_t* channels, uint32_t time_ms)
{
  static gps_acq_state_t prev_state[GPS_SAT_CNT];
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].acq_data.state != prev_state[i])//check if state get changed
    {
      print_state_update_acquisition_line(&channels[i], i);
      prev_state[i] = channels[i].acq_data.state;
      return;
    }
  }
  
  gotoxy(1,6);
  clr_line();
  float time = (float)time_ms / 1000.0f;
  debug_print("SYS RUNTIME: %.1fs\n", time);
}

//Called from gps_master when tracking is running
void print_state_update_tracking(gps_ch_t* channels, uint32_t time_ms)
{
  static uint32_t prev_proc_time_ms = 0;
  uint32_t diff_ms = time_ms - prev_proc_time_ms;
  if (diff_ms < PRINT_STATE_TRACKING_UPDATE_MS)
    return;
  prev_proc_time_ms = time_ms;
  
  time_t cur_time = 0;
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    gotoxy(1, i + 1);
    clr_line();
    print_state_channel(&channels[i], i);
    
    if (channels[i].eph_data.eph.ttr.time > 0)
      cur_time = channels[i].eph_data.eph.ttr.time;
  }
  
  gotoxy(1,6);
  clr_line();
  float time = (float)time_ms / 1000.0f;
  debug_print("SYS RUNTIME: %.1fs\n", time);
  
  if (cur_time > 0)
  {
    gotoxy(1,7);
    clr_line();
    cur_time = cur_time - 18;//PS time offset
    debug_print("UTC TIME: %s", ctime(&cur_time));
  }
}

//line_idx = 0..3
void print_state_update_acquisition_line(gps_ch_t* channel, uint8_t line_idx)
{
  gotoxy(1, line_idx + 1);
  clr_line();
  print_state_channel(channel, line_idx);
}

void print_state_channel(gps_ch_t* channel, uint8_t ch_idx)
{
  debug_print("PRN=%2d ", channel->prn);
  if (channel->tracking_data.state > GPS_TRACKNG_IDLE)
  {
    print_state_tracking_channel(channel, ch_idx);
  }
  else
  {
    print_state_acquisition_channel(channel, ch_idx);
  }
  debug_print("\n", channel->prn);
}

void print_state_tracking_channel(gps_ch_t* channel, uint8_t ch_idx)
{
  switch (channel->tracking_data.state)
  {
  case GPS_NEED_PRE_TRACK:
    debug_print("NEED PRE_TRACK");
    break;
  case GPS_PRE_TRACK_RUN:
    debug_print("PRE_TRACK RUN");
    break;
  case GPS_PRE_TRACK_DONE:
    debug_print("PRE_TRACK DONE");
    break;
  case GPS_TRACKING_RUN:
    debug_print("*TRACK* | ");
    break;
  default:
    break;
  }
  
  uint16_t code = (uint16_t)channel->tracking_data.code_phase_fine / 16;
  debug_print("SNR=%2.0f dB Freq=%5d Hz | Code=%4d chips | wrd=%3d ", 
    channel->tracking_data.snr_value,
    (int16_t)channel->tracking_data.if_freq_offset_hz, 
    code, 
    channel->nav_data.word_cnt_test);
  
  if (channel->nav_data.polarity_found)
  {
    debug_print("| SUBF_CNT=%d ", channel->eph_data.sub_cnt);
  }
}

void print_state_acquisition_channel(gps_ch_t* channel, uint8_t ch_idx)
{
  switch (channel->acq_data.state)
  {
  case GPS_ACQ_NEED_FREQ_SEARCH:
    debug_print("ACQ NEED FREQ_SEARCH ");
    break;
  case GPS_ACQ_FREQ_SEARCH_RUN:
    debug_print("ACQ FREQ_SEARCH ");
    break;
    
  case GPS_ACQ_FREQ_SEARCH_DONE:
    debug_print("ACQ FREQ_SEARCH_DONE ");
    break;
    
  case GPS_ACQ_CODE_PHASE_SEARCH1:
    debug_print("ACQ CODE_SEARCH_1 ");
    break;
  case GPS_ACQ_CODE_PHASE_SEARCH2:
    debug_print("ACQ CODE_SEARCH_2 ");
    break;
  case GPS_ACQ_CODE_PHASE_SEARCH3:
    debug_print("ACQ CODE_SEARCH_3 ");
    break;
    
  case GPS_ACQ_CODE_PHASE_SEARCH1_DONE:
    debug_print("ACQ CODE_SEARCH_1_DONE ");
    break;
  case GPS_ACQ_CODE_PHASE_SEARCH2_DONE:
    debug_print("ACQ CODE_SEARCH_2_DONE ");
    break;
  case GPS_ACQ_CODE_PHASE_SEARCH3_DONE:
    debug_print("ACQ CODE_SEARCH_3_DONE ");
    break;
  default:
    break;
  }
  
  if (channel->acq_data.state >= GPS_ACQ_CODE_PHASE_SEARCH1)
  {
    debug_print("FREQ=%d Hz", channel->acq_data.found_freq_offset_hz);
  }
}


