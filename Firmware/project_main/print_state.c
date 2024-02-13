//Printing state of GPS receiving

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "uart_comm.h"
#include "print_state.h"
#include "stdio.h"
#include "string.h"
#include <stdarg.h>
#include <math.h>
#if (ENABLE_CALC_POSITION)
  #include "solving.h"
#endif


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PRINT_STATE_BUF_LENGTH          3000
#define PRINT_STATE_SEND_PERIOD_MS      150
#define PRINT_STATE_TRACKING_UPDATE_MS  300
#define ESC "\033"

#define LAT_DEG_TO_M    (1852.0f * 60.0f)

#define clr_screen()        debug_print(ESC"[2J") //lear the screen, move to (1,1)
#define gotoxy(x,y)	    debug_print(ESC"[%d;%dH", y, x);
#define clr_line()	    debug_print(ESC"[2K");
#define set_blue_color()    debug_print(ESC"[34m");
#define set_yellow_color()  debug_print(ESC"[33m");
#define set_red_color()  debug_print(ESC"[31m");
#define set_white_color()   debug_print(ESC"[37m");
#define set_gray_color()    debug_print(ESC"[2;37m");
#define reset_color()       debug_print(ESC"[0m");

#define PLOT_START_Y            11
#define PLOT_GRID_CELLS_Y       6 //count
#define PLOT_GRID_CELLS_X       8 //count
#define PLOT_WIDTH              (PLOT_GRID_STEP_X * 8)
#define PLOT_GRID_STEP_Y        3 //in chars
#define PLOT_GRID_STEP_X        7 //in chars
#define PLOT_HEIGHT             (PLOT_GRID_STEP_Y * PLOT_GRID_CELLS_Y)

#define PLOT_GRID_STEP_Y_M      77 //in m
#define PLOT_GRID_STEP_X_M      88 //in m

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char print_state_tmp_buf[PRINT_STATE_BUF_LENGTH];
char print_state_tx_buf[PRINT_STATE_BUF_LENGTH];

char *print_state_wr_p = print_state_tmp_buf;
uint16_t print_state_cnt = 0;

#if (ENABLE_CALC_POSITION)
  extern sol_t gps_sol;
  extern double final_pos[3];//geodetic position {lat,lon,h} (deg,m)
#endif

/* Private function prototypes -----------------------------------------------*/
void print_state_channel(gps_ch_t* channel, uint8_t ch_idx);
void print_state_tracking_channel(gps_ch_t* channel, uint8_t ch_idx);
void print_state_acquisition_channel(gps_ch_t* channel, uint8_t ch_idx);
void print_state_update_acquisition_line(gps_ch_t* channel, uint8_t line_idx);

void print_state_draw_point_terminal(int16_t x, int16_t y);
void print_state_conv_pos(float lat, float lon, float *x, float *y);
void print_state_draw_point(float lat, float lon);

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
  debug_print("SYS RUNTIME: %.1f s\n", 0.0f);
}

//Called from gps_master when acq. is running
void print_state_update_acquisition(gps_ch_t* channels, uint32_t time_ms)
{
  static gps_acq_state_t prev_state[GPS_SAT_CNT];
  uint8_t changed_flag = 0;
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    if (channels[i].acq_data.state != prev_state[i])//check if state get changed
    {
      print_state_update_acquisition_line(&channels[i], i);
      prev_state[i] = channels[i].acq_data.state;
      changed_flag = 1;
      break;
    }
  }
  
  //Update the channel that is scanned now
  if (changed_flag == 0)
  {
    for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
    {
      if (channels[i].acq_data.state == GPS_ACQ_FREQ_SEARCH_RUN)
      {
        print_state_update_acquisition_line(&channels[i], i);
        break;
      }
    }
  }
  
  gotoxy(1,6);
  clr_line();
  float time = (float)time_ms / 1000.0f;
  debug_print("SYS RUNTIME: %.1f s\n", time);
}

//Called from gps_master when tracking is running
void print_state_update_tracking(gps_ch_t* channels, uint32_t time_ms)
{
  static uint32_t prev_proc_time_ms = 0;
  static uint8_t have_position = 0;
  
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
  debug_print("SYS RUNTIME: %.1f s\n", time);
  
  if (cur_time > 0)
  {
    gotoxy(1,7);
    clr_line();
    cur_time = cur_time - GPS_UTC_TIME_OFFSET_S;
    debug_print("EPH UTC TIME: %s", ctime(&cur_time));
  }
  
#if (ENABLE_CALC_POSITION)
  if (gps_sol.stat != SOLQ_NONE)
  {
    gotoxy(1,8);
    clr_line();
    cur_time = gps_sol.time.time - GPS_UTC_TIME_OFFSET_S;
    debug_print("SOLUTION UTC TIME: %s", ctime(&cur_time));
    
    gotoxy(1,9);
    clr_line();
    debug_print("POSITION: %2.5f %2.5f", final_pos[0], final_pos[1]);
    
    if (have_position == 0)
      print_state_draw_plot_grid();
    have_position = 1;
    
    print_state_draw_point((float)final_pos[0], (float)final_pos[1]);
  }
#endif

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
    debug_print("ACQ FREQ_SEARCH_RUN << ");
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
  
  if (channel->acq_data.state == GPS_ACQ_FREQ_SEARCH_RUN)
  {
    uint16_t curr_f_idx = channel->acq_data.freq_index;
    uint16_t acq_percent = curr_f_idx * 100 / ACQ_COUNT;
    debug_print("SCAN: %d%%", acq_percent);
  }
  
  if (channel->acq_data.state >= GPS_ACQ_FREQ_SEARCH_DONE)
  {
    debug_print("FREQ=%5d Hz", channel->acq_data.found_freq_offset_hz);
  }
}

//*******************************************************
//POSITION PLOT

void print_state_draw_plot_grid(void)
{
  uint16_t x, y;
  set_gray_color();
  
  //Draw X lines
  for (y = PLOT_START_Y; y < (PLOT_START_Y + PLOT_HEIGHT + PLOT_GRID_STEP_Y); y+=PLOT_GRID_STEP_Y)
  {
      gotoxy(1, y);
      clr_line();
      
      uint8_t step_y = (y - PLOT_START_Y) / PLOT_GRID_STEP_Y;
      
      if ((step_y == 0) || (step_y == PLOT_GRID_CELLS_Y))
      {
        for (x = 0; x < PLOT_WIDTH; x++)
          debug_print("=");
      }
      else if (step_y == (PLOT_GRID_CELLS_Y / 2))
      {
        for (x = 0; x < PLOT_WIDTH; x++)
          debug_print("-");
      }
      else
      {
        for (x = 0; x < PLOT_WIDTH; x++)
          debug_print(".");
      }


      debug_print("\n");
  }

  
  //Draw Y lines
  for (x = 1; x < (PLOT_WIDTH + 2); x+=PLOT_GRID_STEP_X)
  {
    uint8_t step_x = (x - 1) / PLOT_GRID_STEP_X;
    
    for (y = PLOT_START_Y; y <= (PLOT_START_Y + PLOT_HEIGHT); y++)
    {
      gotoxy(x, y);
      if (step_x == (PLOT_GRID_CELLS_X / 2))
        debug_print("|");
      else
        debug_print(":");
    }
  }
  
  reset_color();
  gotoxy(1, PLOT_START_Y + PLOT_HEIGHT + 1);
  debug_print("HORIZ STEP: %d m", PLOT_GRID_STEP_X_M);
}

//Draw point with latitude/longitude at the grid
void print_state_draw_point(float lat, float lon)
{
  static uint16_t counter = 0;
  static float x_zero, y_zero;
  counter++;
  
  if (counter == 1)
  {
    return;//Skip first point
  }
  
  float x, y;//Pos. Value in m
  print_state_conv_pos(lat, lon, &x, &y);//Conv. to meters
  
  if (counter == 2)
  {
    //Latch zeo point
    x_zero = x;
    y_zero = y;
  }
  //Remove offset
  x = x - x_zero;
  y = y - y_zero;
  
  float grid_cells_x = x / (float)PLOT_GRID_STEP_X_M;
  float grid_cells_y = y / (float)PLOT_GRID_STEP_Y_M;

  int16_t chars_x = (int16_t)roundf(grid_cells_x * PLOT_GRID_STEP_X);
  int16_t chars_y = (int16_t)roundf(grid_cells_y * PLOT_GRID_STEP_Y);
  
  print_state_draw_point_terminal(chars_x, chars_y);
}

//Convert Lat/Long coordinates to X/Y in meters - inaccurate
void print_state_conv_pos(float lat, float lon, float *x, float *y)
{
  *y = lat * LAT_DEG_TO_M;
  float long_deg_to_m = LAT_DEG_TO_M * cosf(lat / 180.0f * 3.1416f);
  *x = long_deg_to_m * lon;
}

//Draw poinnt at the terminal
//x and y is char position, can be positive and negative
void print_state_draw_point_terminal(int16_t x, int16_t y)
{
  y = -y;//invert
  
  if (x < (-PLOT_WIDTH / 2))
    x = (-PLOT_WIDTH / 2);
  else if (x > (PLOT_WIDTH / 2))
    x = (PLOT_WIDTH / 2);
  
  if (y < (-PLOT_HEIGHT / 2))
    x = (-PLOT_HEIGHT / 2);
  else if (x > (PLOT_HEIGHT / 2))
    y = (PLOT_HEIGHT / 2);
  
  x = x + PLOT_WIDTH / 2 + 1;
  y = y + PLOT_START_Y + PLOT_HEIGHT / 2;
  
  //set_red_color();
  gotoxy(x, y);
  debug_print("*");
  //reset_color();
}

