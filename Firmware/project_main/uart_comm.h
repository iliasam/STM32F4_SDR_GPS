
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_COMM_H
#define __UART_COMM_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void uart_init(void);
void uart_prim_dma_send_data(uint8_t* data, uint16_t size);
uint8_t uart_prim_is_busy(void);

#endif /* __UART_COMM_H */
