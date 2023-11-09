
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

/* Private functions ---------------------------------------------------------*/

int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); //4 bits for subpriority
  dwt_init();
  
  gps_fill_summ_table();
  
  memset(&gps_channel1, 0, sizeof(gps_channel1));
  gps_channel1.prn = 1;
  
  delay_ms(100);
  signal_capture_init();
  
  /*
  uint16_t* signal_p = sim_generate_data();
  sim_add_noise(signal_p, 45);
  gps_generate_prn_data(tmp_prn_data, 1);
  
  //uint16_t tmp_val;
  //correlation_search(tmp_prn_data, tmp_data_i, tmp_data_q, &tmp_val);
  
  */
 
  signal_capture_need_data_copy();
  while(1)
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
    
  }//end of while(1)
  
}//end of main()

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

int putchar(int c)
{
  return c;    
}


void delay_ms(uint32_t ms)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
  for (; nCount!=0; nCount--);
}

