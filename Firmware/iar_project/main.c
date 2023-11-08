
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_rcc.h"
#include "delay_us_timer.h"
#include "simulator.h"
#include "gps_misc.h"
#include "config.h"
#include "signal_capture.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint16_t prn_data[PRN_SPI_WORDS_CNT];//data with no modulation
uint16_t corr1_buf[PRN_SPI_WORDS_CNT];
uint16_t corr2_buf[PRN_SPI_WORDS_CNT];
                
uint16_t tmp_data_i[PRN_SPI_WORDS_CNT];
uint16_t tmp_data_q[PRN_SPI_WORDS_CNT];

volatile int16_t test_res = 0;
/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); //4 bits for subpriority
  dwt_init();

  delay_ms(100);
  gps_fill_summ_table();
  
  signal_capture_init();
  
  /*
  uint16_t* signal_p = sim_generate_data();
  sim_add_noise(signal_p, 45);
  gps_generate_prn_data(prn_data, 1);
  
  uint16_t tmp_val;
  correlation_search(prn_data, tmp_data_i, tmp_data_q, &tmp_val);
  */
  
  while(1)
  {

  }
}

//123us



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

