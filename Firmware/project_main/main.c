//Single satellite acquisition and tracking. 
//Printf is using semihosting via SWO (configured in )

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_rcc.h"
#include "delay_us_timer.h"
#include "simulator.h"
#include "gps_misc.h"
#include "config.h"
#include "signal_capture.h"
#include "common_ram.h"
#include "acquisition.h"
#include "tracking.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


volatile int16_t test_res = 0;
volatile static uint32_t diff_main = 0;

gps_ch_t gps_channels[GPS_SAT_CNT];

/* Private function prototypes -----------------------------------------------*/
void gps_new_data_handling(void);

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
                
  signal_capture_init();
  
  signal_capture_need_data_copy();
  
  while(1)
  {
    uint8_t need_slow = need_slow_data_proc();
    if (need_slow)
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
  }//end of while(1)
  
}//end of main()

//*********************************

void main_fast_data_proc(void)
{
  uint8_t* signal_p = signal_capture_get_ready_buf();
  uint32_t time_cnt = signal_capture_get_packet_cnt();
  
  //A kind of simulation of 4 channel multiplexing
  uint32_t index_big = time_cnt % (TRACKING_CH_LENGTH * GPS_SAT_CNT + 1); //real index, 0-16 range
  uint32_t index = index_big % TRACKING_CH_LENGTH; //in 0-3 range
  if (index_big == (TRACKING_CH_LENGTH * GPS_SAT_CNT))//dummy step
    index = 0xFF;
  
  //So "index" can be 0-3 or 0xFF
  gps_tracking_process(&gps_channels[0], signal_p, (uint8_t)index);
}


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
    gps_new_data_handling();
    signal_capture_need_data_copy();
  }
}


//Return 1 when acq. data processing is needed
uint8_t need_slow_data_proc(void)
{
  if (gps_channels[0].acq_data.state < GPS_ACQ_CODE_PHASE_SEARCH3_DONE)
    return 1;
  
  return 0;
}

// Acquisition data process
// Can be long!
void gps_new_data_handling(void)
{
  uint8_t* signal_p = signal_capture_get_copy_buf();
  acquisition_process(&gps_channels[0], signal_p);
  
  if (gps_channels[0].acq_data.state == GPS_ACQ_FREQ_SEARCH_DONE)
  {
    printf("FINAL FREQ=%d Hz\n", gps_channels[0].acq_data.found_freq_offset_hz);
  }
  
  if (gps_channels[0].acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH3_DONE)
  {
    printf("FINAL ACQ (SRCH_3) CODE=%d\n", gps_channels[0].acq_data.found_code_phase);
    gps_channels[0].acq_data.state = GPS_ACQ_DONE;
    gps_channels[0].tracking_data.state = GPS_NEED_PRE_TRACK;
  }
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

