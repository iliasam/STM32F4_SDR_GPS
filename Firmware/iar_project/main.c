
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_rcc.h"
#include "delay_us_timer.h"
#include "simulator.h"
#include "gps_misc.h"
#include "config.h"
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
  dwt_init();

  delay_ms(500);
  
  gps_fill_summ_table();
  
  uint16_t* signal_p = sim_generate_data();
  sim_add_noise(signal_p, 45);
  gps_generate_prn_data(prn_data, 1);
  
  gps_shift_to_zero_freq(
    (uint8_t*)signal_p, (uint8_t*)tmp_data_i, (uint8_t*)tmp_data_q, IF_FREQ_HZ);
  

    /*
  //Generate pure IF data
  gps_generate_sin(sin_data, PRN_SPI_WORDS_CNT, IF_FREQ_HZ);
  gps_generate_cos(cos_data, PRN_SPI_WORDS_CNT, IF_FREQ_HZ);
  
  //Shift to 0 Hz (signal_p x sin_data => tmp_data_i)
  gps_mult8((uint8_t*)signal_p, (uint8_t*)sin_data, (uint8_t*)tmp_data_i, PRN_SPI_WORDS_CNT * 2, 0);
  gps_mult8((uint8_t*)signal_p, (uint8_t*)cos_data, (uint8_t*)tmp_data_q, PRN_SPI_WORDS_CNT * 2, 0);
    */
  
  test_res = gps_correlation8(
    prn_data, tmp_data_i, tmp_data_q, corr1_buf, corr2_buf, 100);
  
  uint16_t tmp_val;
  correlation_search(prn_data, tmp_data_i, tmp_data_q, &tmp_val);
  
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

