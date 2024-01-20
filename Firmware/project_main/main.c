//Single satellite acquisition and tracking. 
//Printf is using semihosting via SWO (configured in )

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_rcc.h"
#include "delay_us_timer.h"
#include "gps_misc.h"
#include "config.h"
#include "signal_capture.h"
#include "common_ram.h"
#include "acquisition.h"
#include "tracking.h"
#include "gps_master.h"
#include "uart_comm.h"
#include "print_state.h"
#include "keys_controlling.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#if (ENABLE_RTCM_SEND)
  #include "obs_publish.h"
#endif

#if (ENABLE_CALC_POSITION)
  #include "solving.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
gps_ch_t gps_channels[GPS_SAT_CNT];

/* Private function prototypes -----------------------------------------------*/
void main_process_acq_data(void);

uint8_t need_slow_data_proc(void);
void main_slow_data_proc(void);
void main_fast_data_proc(void);

/* Private functions ---------------------------------------------------------*/

int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); //4 bits for subpriority
  dwt_init();
  
  gps_fill_summ_table();
  
  memset(&gps_channels[0], 0, sizeof(gps_channels));
  
  //solve_test();
  
  // user can enter known doppler frequency to make acquisition much faster
  gps_channels[0].prn = 5;
  gps_channels[0].acq_data.given_freq_offset_hz = 900;
  gps_channell_prepare(&gps_channels[0]);
  
  gps_channels[1].prn = 14;
  gps_channels[1].acq_data.given_freq_offset_hz = 4000;
  gps_channell_prepare(&gps_channels[1]);
  
  gps_channels[2].prn = 20;
  gps_channels[2].acq_data.given_freq_offset_hz = -1000;
  gps_channell_prepare(&gps_channels[2]);
  
  gps_channels[3].prn = 30;
  gps_channels[3].acq_data.given_freq_offset_hz = 2000;
  gps_channell_prepare(&gps_channels[3]);
  
  /*
  gps_channels[0].prn = 10;
  gps_channell_prepare(&gps_channels[0]);
  gps_channels[1].prn = 23;
  gps_channell_prepare(&gps_channels[1]);
  gps_channels[2].prn = 26;
  gps_channell_prepare(&gps_channels[2]);
  gps_channels[3].prn = 16;
  gps_channell_prepare(&gps_channels[3]);
  */
#if (ENABLE_CALC_POSITION)
  gps_pos_solve_init(gps_channels);
#endif
  
  uart_init();
  keys_init();
  signal_capture_init();
  
  signal_capture_need_data_copy();
  print_state_prepare(gps_channels);
  

  while(1)
  {
    uint8_t need_slow = gps_master_need_acq();
    if (need_slow > 0)
    {
      //Not realtime
      main_slow_data_proc();
    }
    else
    {
      //Realtime!
      uint8_t have_new_data = signal_capture_have_irq();
      if (have_new_data)
      {
        main_fast_data_proc();
      }
    }
    key_handling();
  }//end of while(1)
  
}//end of main()

//*********************************

// Acqusition data capure and process - not realtime
void main_slow_data_proc(void)
{
  uint8_t have_data = signal_capture_check_copied();
  if (have_data == 0)
  {
    //wait for IRQ
    while (signal_capture_have_irq() == 0) 
    {
      asm("nop");
    }
    
    //Process IRQ, data copy
    signal_capture_handling();
  }
  else
  {
    //have new data
    main_process_acq_data();
    signal_capture_need_data_copy();
  }
}

//Tracking
void main_fast_data_proc(void)
{
  uint8_t* signal_p = signal_capture_get_ready_buf();
  uint32_t time_cnt = signal_capture_get_packet_cnt();
  
  //4 channel multiplexing
  uint32_t index_big = time_cnt % (TRACKING_CH_LENGTH * GPS_SAT_CNT + 1); //real index, 0-16 range
  
  uint8_t sat_index = index_big / TRACKING_CH_LENGTH;
  if (sat_index >= GPS_SAT_CNT)
    sat_index = 0;
  
  uint32_t index = 0;
  if (index_big == (TRACKING_CH_LENGTH * GPS_SAT_CNT))
    index = 0xFF;
  else
  {
    index = index_big % TRACKING_CH_LENGTH;
  }
  
  //"index" can be 0-3 or 0xFF
  gps_tracking_process(&gps_channels[sat_index], signal_p, (uint8_t)index);
  
  gps_master_handling(gps_channels, index);
}


// Acquisition data process
// Can be long!
void main_process_acq_data(void)
{
  uint8_t* signal_p = signal_capture_get_copy_buf();
  acquisition_process(&gps_channels[0], signal_p);
  gps_master_handling(gps_channels, 0);
}

///////////////////////////////////

/*
int putchar(int c)
{
  return c;    
}
*/


void delay_ms(uint32_t ms)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
  for (; nCount!=0; nCount--);
}

