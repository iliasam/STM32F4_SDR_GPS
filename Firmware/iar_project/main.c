
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

gps_ch_t gps_channel1;
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
  
  memset(&gps_channel1, 0, sizeof(gps_channel1));
  gps_channel1.prn = 13;
 // gps_channel1.acq_data.given_freq_offset_hz = 1200;
  gps_channell_prepare(&gps_channel1);
  
  delay_ms(100);
  signal_capture_init();
  
  /*
  uint16_t* signal_p = sim_generate_data();
  sim_add_noise(signal_p, 45);
  
  gps_generate_prn_data2(&gps_channel1, tmp_prn_data, 0);
  gps_shift_to_zero_freq((uint8_t*)signal_p, (uint8_t*)tmp_data_i, (uint8_t*)tmp_data_q, IF_FREQ_HZ + 2000);
  uint16_t avr_val;
  uint16_t best_phase = 0;
  correlation_search(tmp_prn_data, tmp_data_i, tmp_data_q, 0, (PRN_LENGTH * 2), &avr_val, &best_phase);
  */
  
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
  
  uint32_t index_big = time_cnt % (TRACKING_CH_LENGTH * 4 + 1);
  uint32_t index = index_big % TRACKING_CH_LENGTH;
  if (index_big == (TRACKING_CH_LENGTH * 4))
    index = 0xFF;
  gps_tracking_process(&gps_channel1, signal_p, (uint8_t)index);
}


//Acqusition data proc - not realtime
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
    
    //Process IRQ
    signal_capture_handling();
  }
  else
  {
    //have new data
    gps_new_data_handling();
    signal_capture_need_data_copy();
  }
  
  
  if (gps_channel1.acq_data.state == GPS_ACQ_CODE_PHASE_SEARCH3_DONE)
  {
    printf("FINAL ACQ (SRCH_3) CODE=%d\n", gps_channel1.acq_data.found_code_phase);
    gps_channel1.acq_data.state = GPS_ACQ_DONE;
    gps_channel1.tracking_data.state = GPS_NEED_PRE_TRACK;
  }
}


//Return 1 hen acq data processing is needed
uint8_t need_slow_data_proc(void)
{
  if (gps_channel1.acq_data.state < GPS_ACQ_CODE_PHASE_SEARCH3_DONE)
    return 1;
  
  return 0;
}

//Can be long!
void gps_new_data_handling(void)
{
  uint8_t* signal_p = signal_capture_get_copy_buf();
  acquisition_process(&gps_channel1, signal_p);
  while (acquisition_need_new_data(&gps_channel1) == 0)
  {
    acquisition_process(&gps_channel1, signal_p);
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

