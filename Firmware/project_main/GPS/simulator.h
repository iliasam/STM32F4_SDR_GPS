#include "stdint.h"

uint16_t* sim_generate_data(void);
void sim_add_noise(uint16_t* buff_p, uint8_t noise_level);

void gps_generate_prn_data(uint16_t* data, uint8_t prn);
