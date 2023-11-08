
// 8*8=64MHz
//#define MCU_CLK_MULTIPLIER      RCC_PLLMul_8

#define SPI_NAME                SPI2
#define SPI_PRESCALER           SPI_BaudRatePrescaler_4
#define SPI_DMA_STREAM          DMA1_Stream3
#define SPI_DMA_CH              DMA_Channel_0
#define SPI_IRQ_HANDLER         DMA1_Stream3_IRQHandler

#define SPI_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define SPI_CLK_PIN             GPIO_Pin_13
#define SPI_MISO_PIN            GPIO_Pin_14
#define SPI_CLK_PIN_SRC         GPIO_PinSource13
#define SPI_MISO_PIN_SRC        GPIO_PinSource14
#define SPI_AFIO                GPIO_AF_SPI2
#define SPI_GPIO                GPIOB

#define SIM_PRN_CODE            1

#define IF_FREQ_HZ              (int)(4096000 + 0000)
#define SPI_BAUDRATE_HZ         (int)(16384000)
#define PRN_SPEED_HZ            1000 //1ms period
#define BITS_IN_PRN             (SPI_BAUDRATE_HZ / PRN_SPEED_HZ) //16Kbit
#define PRN_SPI_WORDS_CNT       (BITS_IN_PRN / 16) //1024 words
#define PRN_LENGTH              1023

//--------------------------
#define LED4_PIN                         GPIO_Pin_12
#define LED4_GPIO_PORT                   GPIOD

