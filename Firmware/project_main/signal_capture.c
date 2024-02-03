#include "signal_capture.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include "delay_us_timer.h"
#include "config.h"

//SPI is working in 16-bit mode, slave, receive only
//Data is written to the RAM using circular DMA
//One 16-bit word is one GPS PRN "chip"

// Circular buffer, updated by DMA in realtime
volatile uint16_t spi_rx_buffer[PRN_SPI_WORDS_CNT * 2];
// Software can read from this pointer
volatile uint16_t* spi_curr_ready_rx_buf = &spi_rx_buffer[PRN_SPI_WORDS_CNT];

// TMP buffer for long processing
uint16_t spi_rx_copy_buffer[PRN_SPI_WORDS_CNT];

uint8_t signal_capture_need_copy_flag = 0;
volatile uint8_t signal_capture_irq_unprocessed_flag = 0;
//1 packet = 1ms, 1 PRN chip
volatile uint32_t signal_capture_packet_cnt = 0;
volatile uint32_t signal_capture_irq_timestamp = 0;//DWT timer

volatile uint16_t test_cnt = 0;

void init_spi(void);
void init_dma(void);

//************************************************************************

// Return count of received data blocks, a kind of of system time
uint32_t signal_capture_get_packet_cnt(void)
{
  return signal_capture_packet_cnt;
}

// Return pointer to the buffer with copied data, fixed address
uint8_t* signal_capture_get_copy_buf(void)
{
  return (uint8_t*)spi_rx_copy_buffer;
}

// Return pointer to a "fast" buffer, that is already filled
// Data can be overwritten by DMA in 1ms! Need to be processed fast.
// Also reset have new data flag
uint8_t* signal_capture_get_ready_buf(void)
{
  signal_capture_irq_unprocessed_flag = 0;
  return (uint8_t*)spi_curr_ready_rx_buf;
}

// DMA half transfer and transfer complete IRQ
// Called every 1ms (see PRN_SPEED_HZ)
void SPI_DMA_IRQ_HANDLER(void)
{
  if (DMA_GetFlagStatus(SPI_DMA_STREAM, DMA_FLAG_HTIF3))//half complete
  {
    DMA_ClearFlag(SPI_DMA_STREAM, DMA_FLAG_HTIF3);
    spi_curr_ready_rx_buf = &spi_rx_buffer[0];
  }
  
  if (DMA_GetFlagStatus(SPI_DMA_STREAM, DMA_FLAG_TCIF3))//full complete
  {
    DMA_ClearFlag(SPI_DMA_STREAM, DMA_FLAG_TCIF3);
    spi_curr_ready_rx_buf = &spi_rx_buffer[PRN_SPI_WORDS_CNT];
  }
  
  
  signal_capture_packet_cnt++;
  signal_capture_irq_timestamp = get_dwt_value();
  signal_capture_irq_unprocessed_flag = 1;
  
  //LED4_GPIO_PORT->ODR ^= LED4_PIN;//debug
  uint32_t tmp_val = (signal_capture_packet_cnt & 256);
  if (tmp_val < 128)
    LED4_GPIO_PORT->ODR |= LED4_PIN;
  else
    LED4_GPIO_PORT->ODR &= ~LED4_PIN;
}

void signal_capture_init(void)
{
  init_dma();
  init_spi();
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin =  LED4_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStruct);
}

// Must be called periodically
void signal_capture_handling(void)
{
  if (signal_capture_irq_unprocessed_flag == 0)
  {
    return;
  }
    
  if (signal_capture_need_copy_flag)
  {
    uint32_t time_now = get_dwt_value();
    uint32_t time_diff = time_now - signal_capture_irq_timestamp;
    time_diff = time_diff / CPU_TICKS_US;//convert to us
    if (time_diff > 900) //PRN period is 1ms
      return; //to much time from IRQ, possibly do not have time to copy
    
    NVIC_DisableIRQ(SPI_DMA_IRQ);
    memcpy(spi_rx_copy_buffer, 
           (void*)spi_curr_ready_rx_buf, 
           sizeof(spi_rx_copy_buffer));
    test_cnt++;
    signal_capture_need_copy_flag = 0;
    signal_capture_irq_unprocessed_flag = 0;
    NVIC_EnableIRQ(SPI_DMA_IRQ);
  }
}

uint8_t signal_capture_have_irq(void)
{
  return signal_capture_irq_unprocessed_flag;
}

// Called by external code
// Notify signal capturing part that we need copied data
void signal_capture_need_data_copy(void)
{
  signal_capture_need_copy_flag = 1;
}

uint8_t signal_capture_check_copied(void)
{
  return (signal_capture_need_copy_flag == 0);
}

void init_spi(void)
{    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStruct;
  
  RCC_AHB1PeriphClockCmd(SPI_GPIO_CLK, ENABLE);
  GPIO_InitStruct.GPIO_Pin =  SPI_CLK_PIN | SPI_MOSI_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SPI_GPIO, &GPIO_InitStruct);
  
  GPIO_PinAFConfig(SPI_GPIO, SPI_CLK_PIN_SRC, SPI_AFIO);
  GPIO_PinAFConfig(SPI_GPIO, SPI_MOSI_PIN_SRC, SPI_AFIO);
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//clock polarity - idle low
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_PRESCALER;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
  SPI_Init(SPI_NAME, &SPI_InitStructure);
  
  SPI_I2S_DMACmd(SPI_NAME, SPI_I2S_DMAReq_Rx, ENABLE);
  //CS to low
  SPI_NSSInternalSoftwareConfig(SPI_NAME, SPI_NSSInternalSoft_Reset);
  
  SPI_Cmd(SPI_NAME, ENABLE);
}

void init_dma(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  
  DMA_DeInit(SPI_DMA_STREAM);
  
  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_BufferSize = PRN_SPI_WORDS_CNT * 2;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&(SPI_NAME->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_Channel = SPI_DMA_CH;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)spi_rx_buffer; 
  DMA_Init(SPI_DMA_STREAM, &DMA_InitStructure);
  
  DMA_ITConfig(SPI_DMA_STREAM, DMA_IT_HT, ENABLE);
  DMA_ITConfig(SPI_DMA_STREAM, DMA_IT_TC, ENABLE);
  
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SPI_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  DMA_Cmd(SPI_DMA_STREAM, ENABLE);
}
