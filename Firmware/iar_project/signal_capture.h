#ifndef _SIGNAL_CAPTURE_H
#define _SIGNAL_CAPTURE_H

#include "stdint.h"

void signal_capture_init(void);
void signal_capture_need_data_copy(void);
void signal_capture_handling(void);
uint8_t signal_capture_have_irq(void);
uint8_t signal_capture_check_copied(void);
uint8_t* signal_capture_get_copy_buf(void);

#endif


