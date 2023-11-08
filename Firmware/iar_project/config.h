
// 8*8=64MHz
//#define MCU_CLK_MULTIPLIER      RCC_PLLMul_8

#define SPI_PRESCALER           SPI_BaudRatePrescaler_4
#define SPI_CLK_PIN             GPIO_Pin_5
#define SPI_MOSI_PIN            GPIO_Pin_7
#define SPI_GPIO                GPIOA
#define SPI_DMA_CH              DMA1_Channel3

#define SIM_PRN_CODE            1

#define IF_FREQ_HZ              (int)(4096000 + 0000)
#define SPI_BAUDRATE_HZ         (int)(16384000)
#define PRN_SPEED_HZ            1000 //1ms period
#define BITS_IN_PRN             (SPI_BAUDRATE_HZ / PRN_SPEED_HZ) //16Kbit
#define PRN_SPI_WORDS_CNT       (BITS_IN_PRN / 16) //1024 words
#define PRN_LENGTH              1023


