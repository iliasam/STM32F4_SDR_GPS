#ifndef _CONFIG_H
#define _CONFIG_H

//MCU speed is 168MHz
#define CPU_TICKS_US            (168)

#define SPI_NAME                SPI2
#define SPI_PRESCALER           SPI_BaudRatePrescaler_4
#define SPI_DMA_STREAM          DMA1_Stream3
#define SPI_DMA_CH              DMA_Channel_0
#define SPI_DMA_IRQ             DMA1_Stream3_IRQn
#define SPI_DMA_IRQ_HANDLER     DMA1_Stream3_IRQHandler

#define SPI_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define SPI_CLK_PIN             GPIO_Pin_13
#define SPI_MISO_PIN            GPIO_Pin_14
#define SPI_CLK_PIN_SRC         GPIO_PinSource13
#define SPI_MISO_PIN_SRC        GPIO_PinSource14
#define SPI_AFIO                GPIO_AF_SPI2
#define SPI_GPIO                GPIOB

#define SIM_PRN_CODE            1

#define IF_FREQ_HZ              (int)(4092000)
#define SPI_BAUDRATE_HZ         (int)(16368000)
#define PRN_SPEED_HZ            1000 //1ms period
#define BITS_IN_PRN             (SPI_BAUDRATE_HZ / PRN_SPEED_HZ) //16Kbit
#define PRN_SPI_WORDS_CNT       (BITS_IN_PRN / 16) //1024 words
#define PRN_LENGTH              1023



#define ACQ_SEARCH_FREQ_HZ      (7000) //Search zone is x2
#define ACQ_SEARCH_STEP_HZ      (500)


#define TRACKING_DLL1_C1        (1.0f)
#define TRACKING_DLL1_C2        (300.0f)

#define TRACKING_PLL1_C1        (10.0f)
#define TRACKING_PLL1_C2        (1000.0f)
#define TRACKING_FLL1	        (2000.0f)

//#define IF_NCO_STEP_HZ        ((float)SPI_BAUDRATE_HZ / (float)(1 << 32)) //NCO accumulator is 32bit
#define IF_NCO_STEP_HZ	        (0.003810972f)

// Number of analysed PRN periods in one channal
#define TRACKING_CH_LENGTH      4

//Number of used satellites
#define GPS_SAT_CNT	        4

//--------------------------
//Debug only
#define LED4_PIN                         GPIO_Pin_12
#define LED4_GPIO_PORT                   GPIOD


#endif //_CONFIG_H

