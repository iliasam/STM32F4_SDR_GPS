#include "signal_capture.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include "config.h"

#define DUMMY_Address         0x20000000

uint16_t spi_rx_buffer[PRN_SPI_WORDS_CNT * 2];

void init_spi(void);
void init_dma(void);

//************************************************************************

void SPI_IRQ_HANDLER(void)
{
  if (DMA_GetFlagStatus(SPI_DMA_STREAM, DMA_FLAG_HTIF3))
  {
    DMA_ClearFlag(SPI_DMA_STREAM, DMA_FLAG_HTIF3);
  }
  
  if (DMA_GetFlagStatus(SPI_DMA_STREAM, DMA_FLAG_TCIF3))
  {
    DMA_ClearFlag(SPI_DMA_STREAM, DMA_FLAG_TCIF3);
  }
  
  LED4_GPIO_PORT->ODR ^= LED4_PIN;
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

void init_spi(void)
{    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  SPI_InitTypeDef    SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStruct;
  
  RCC_AHB1PeriphClockCmd(SPI_GPIO_CLK, ENABLE);
  GPIO_InitStruct.GPIO_Pin =  SPI_CLK_PIN | SPI_MISO_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SPI_GPIO, &GPIO_InitStruct);
  
  GPIO_PinAFConfig(SPI_GPIO, SPI_CLK_PIN_SRC, SPI_AFIO);
  GPIO_PinAFConfig(SPI_GPIO, SPI_MISO_PIN_SRC, SPI_AFIO);
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;
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
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  DMA_Cmd(SPI_DMA_STREAM, ENABLE);
}
