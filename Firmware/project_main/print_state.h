
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRINT_STATE_H
#define __PRINT_STATE_H

/* Includes ------------------------------------------------------------------*/
#include "gps_misc.h"
#include <stdio.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void print_state_handling(uint32_t time_ms);
void print_state_prepare(gps_ch_t* channels);
void print_state_update_acquisition(gps_ch_t* channels, uint32_t time_ms);
void print_state_update_tracking(gps_ch_t* channels, uint32_t time_ms);
void print_state_draw_plot_grid(void);

#endif /* __PRINT_STATE_H */
