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
#define SPI_MOSI_PIN            GPIO_Pin_15 //Slave Input!
#define SPI_CLK_PIN_SRC         GPIO_PinSource13
#define SPI_MOSI_PIN_SRC        GPIO_PinSource15
#define SPI_AFIO                GPIO_AF_SPI2
#define SPI_GPIO                GPIOB


#define IF_FREQ_HZ              (int)(4092000)
#define SPI_BAUDRATE_HZ         (int)(16368000)
#define PRN_SPEED_HZ            1000 //1ms period
#define BITS_IN_PRN             (SPI_BAUDRATE_HZ / PRN_SPEED_HZ) //16Kbit
#define PRN_SPI_WORDS_CNT       (BITS_IN_PRN / 16) //1023 16bit words
#define PRN_LENGTH              1023 //in chips

#define ENABLE_RTCM_SEND        0

//Calculate receiver position by observations
#define ENABLE_CALC_POSITION    1

// Accurate code phase averaging
#define ENABLE_CODE_FILTER      1
//Number of measurements, speed is 25 measurements in one channel per 100ms
#define CODE_FILTER_LENGTH      100


#define ACQ_SEARCH_FREQ_HZ      (7000) //Frequency Search zone is x2. Doppler offset
#define ACQ_SEARCH_STEP_HZ      (500)
/// Number of freq steps
#define ACQ_COUNT       (ACQ_SEARCH_FREQ_HZ * 2 / ACQ_SEARCH_STEP_HZ + 1)

/// 1 step is 0.5 of chip
#define ACQ_PHASE1_HIST_STEP	(64)
#define ACQ_PHASE1_HIST_SIZE	((PRN_LENGTH + 1) * 2 / ACQ_PHASE1_HIST_STEP) //32

#define PRE_TRACK_POINTS_MAX_CNT        30

//#define IF_NCO_STEP_HZ        ((float)SPI_BAUDRATE_HZ / (float)(1 << 32)) //NCO accumulator is 32bit
#define IF_NCO_STEP_HZ	        (0.003810972f)

// Number of analysed PRN periods in one channal
#define TRACKING_CH_LENGTH      4

//Number of used satellites
#define GPS_SAT_CNT	        4

#define TRACKING_DLL1_C1        (1.0f)
#define TRACKING_DLL1_C2        (300.0f)

#define TRACKING_PLL1_C1        (4.0f)
#define TRACKING_PLL1_C2        (3000.0f)

#define TRACKING_PLL2_C1        (8.0f)
#define TRACKING_PLL2_C2        (5000.0f)

#define TRACKING_FLL1_C1        (200.0f)
#define TRACKING_FLL1_C2        (2000.0f)

#define GPS_BUILD_WEEK          2290 //NOV 2023

//*************************************************************************

//PRIMARY UART - sending RTCM

#define PRIMARY_UART_NAME       USART2
#define PRIMARY_UART_AF_NAME    GPIO_AF_USART2
#define PRIMARY_UART_BAUDRATE   115200

#define PRIMARY_UART_GPIO_CLK   RCC_AHB1Periph_GPIOA
#define PRIMARY_UART_CLK        RCC_APB1Periph_USART2

#define PRIMARY_UART_RX_PIN     GPIO_Pin_3
#define PRIMARY_UART_RX_PIN_SRC GPIO_PinSource3
#define PRIMARY_UART_RX_GPIO    GPIOA

#define PRIMARY_UART_TX_PIN     GPIO_Pin_2
#define PRIMARY_UART_TX_PIN_SRC GPIO_PinSource2
#define PRIMARY_UART_TX_GPIO    GPIOA

#define PRIMARY_UART_IRQ_HANDLER      USART2_IRQHandler

#define PRIMARY_DMA_RCC         RCC_AHB1Periph_DMA1
#define PRIMARY_DMA_TX_STREAM   DMA1_Stream6
#define PRIMARY_DMA_FLAG        DMA_FLAG_TCIF6
#define PRIMARY_DMA_TX_CH       DMA_Channel_4

//SECONDARY UART - sending GPS status to the terminal emulator

#define SECONDARY_UART_NAME     USART3
#define SECONDARY_UART_AF_NAME  GPIO_AF_USART3
#define SECONDARY_UART_BAUDRATE 115200

#define SECONDARY_UART_GPIO_CLK RCC_AHB1Periph_GPIOD
#define SECONDARY_UART_CLK      RCC_APB1Periph_USART3

#define SECONDARY_UART_RX_PIN   GPIO_Pin_9
#define SECONDARY_UART_RX_PIN_SRC GPIO_PinSource9
#define SECONDARY_UART_RX_GPIO  GPIOD

#define SECONDARY_UART_TX_PIN   GPIO_Pin_8
#define SECONDARY_UART_TX_PIN_SRC GPIO_PinSource8
#define SECONDARY_UART_TX_GPIO  GPIOD

#define SECONDARY_UART_IRQ_HANDLER      USART3_IRQHandler

#define SECONDARY_DMA_RCC       RCC_AHB1Periph_DMA1
#define SECONDARY_DMA_TX_STREAM DMA1_Stream4
#define SECONDARY_DMA_FLAG      DMA_FLAG_TCIF4
#define SECONDARY_DMA_TX_CH     DMA_Channel_7


//--------------------------
//Debug only
#define LED4_PIN                GPIO_Pin_12
#define LED4_GPIO_PORT          GPIOD

#define BUTTON1_PIN             GPIO_Pin_0
#define BUTTON1_GPIO            GPIOA


#endif //_CONFIG_H

