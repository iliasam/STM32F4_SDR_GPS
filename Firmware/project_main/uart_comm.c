
/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "uart_comm.h"
#include "stm32f4xx_rcc.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void uart_prim_init_gpio(void);
void uart_prim_init(void);
void uart_prim_init_dma(void);


/* Private functions ---------------------------------------------------------*/

void uart_init(void)
{
  uart_prim_init_gpio();
  uart_prim_init();
  uart_prim_init_dma();
}

void uart_prim_init_gpio(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(PRIMARY_UART_GPIO_CLK, ENABLE);
  
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  
  GPIO_InitStructure.GPIO_Pin = PRIMARY_UART_RX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PRIMARY_UART_RX_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PRIMARY_UART_TX_PIN;
  GPIO_Init(PRIMARY_UART_TX_GPIO, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(
    PRIMARY_UART_RX_GPIO, PRIMARY_UART_RX_PIN_SRC, PRIMARY_UART_AF_NAME);
  GPIO_PinAFConfig(
    PRIMARY_UART_TX_GPIO, PRIMARY_UART_TX_PIN_SRC, PRIMARY_UART_AF_NAME);
}

void uart_prim_init(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_APB1PeriphClockCmd(PRIMARY_UART_CLK, ENABLE);
  
  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = PRIMARY_UART_BAUDRATE;
  USART_Init(PRIMARY_UART_NAME, &USART_InitStructure);
  
  /*
  USART_ITConfig(PRIMARY_UART_NAME, USART_IT_RXNE, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  */
  
  USART_DMACmd(PRIMARY_UART_NAME, USART_DMAReq_Tx, ENABLE);
  
  USART_Cmd(PRIMARY_UART_NAME, ENABLE);
}

void uart_prim_init_dma(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHB1PeriphClockCmd(PRIMARY_DMA_RCC, ENABLE);
  
  DMA_DeInit(PRIMARY_DMA_TX_STREAM);
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = PRIMARY_DMA_TX_CH;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&PRIMARY_UART_NAME->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;//not used now
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = 0;//not used now
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
  DMA_Init(PRIMARY_DMA_TX_STREAM, &DMA_InitStructure);
}


void uart_prim_dma_send_data(uint8_t* data, uint16_t size)
{
  if (PRIMARY_DMA_TX_STREAM->NDTR > 0)
    return;
  
  DMA_Cmd(PRIMARY_DMA_TX_STREAM, DISABLE);
  
  DMA_ClearFlag(PRIMARY_DMA_TX_STREAM, PRIMARY_DMA_FLAG);//do not work without this?
  PRIMARY_DMA_TX_STREAM->NDTR = size;
  PRIMARY_DMA_TX_STREAM->M0AR = (uint32_t)data;
  DMA_Cmd(PRIMARY_DMA_TX_STREAM, ENABLE);
}

uint8_t uart_prim_is_busy(void)
{
  return (PRIMARY_DMA_TX_STREAM->NDTR > 0) ? 1 : 0;
}

void PRIMARY_UART_IRQ_HANDLER(void)
{
  /* USART in Receiver mode */
  if (USART_GetITStatus(PRIMARY_UART_NAME, USART_IT_RXNE) == SET)
  {
    //USART_ITConfig(PRIMARY_UART_NAME, USART_IT_RXNE, DISABLE);
  }
  else
  {
    asm("nop");
  }
}
