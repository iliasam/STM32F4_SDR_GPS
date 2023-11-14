#include "stdint.h"

typedef enum
{
	GPS_ACQ_NEED_FREQ_SEARCH = 0,
	GPS_ACQ_FREQ_SEARCH_RUN,
	GPS_ACQ_FREQ_SEARCH_DONE,
} gps_acq_state_t;

typedef struct
{
	uint8_t			freq_index; //Freq search index
	uint8_t			single_freq_length;
	int16_t			found_freq_offset_hz;
	uint16_t		found_code_phase;

	uint16_t		code_search_start;
	uint16_t		code_search_stop;
	uint16_t		code_hist_step;

	gps_acq_state_t state;

	float hist_ratio;
} gps_acq_t;

typedef struct
{
	gps_acq_t	acq_data;//Acq variables
	uint8_t		prn; //Sat PRN code
} gps_ch_t;

//********************************************************************************

void gps_fill_summ_table(void);
uint8_t gps_check_bit16(uint16_t* ptr, uint8_t pos);
void gps_generate_sin(uint16_t* ptr, uint16_t length, float freq_hz);
void gps_generate_cos(uint16_t* ptr, uint16_t length, float freq_hz);

void gps_mult8(uint8_t* src1_p, uint8_t* src2_p, uint8_t* dst_p, uint16_t length, uint16_t offset);
uint8_t gps_summ8(uint8_t data);
uint16_t gps_summ(uint16_t* data, uint16_t length);
int16_t gps_correlation8(
  uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, uint16_t offset);
uint16_t correlation_search(
	uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, 
	uint16_t start_shift, uint16_t stop_shift,  uint16_t* aver_val, uint16_t* phase);

void gps_mult_iq8(uint8_t* src_i, uint8_t* src_q, uint8_t* src2, uint8_t* dst_i, uint8_t* dst_q, uint16_t length, uint16_t offset);
void gps_mult_iq32(uint8_t* src_i, uint8_t* src_q, uint8_t* src2, uint8_t* dst_i, uint8_t* dst_q, uint16_t length, uint16_t offset);

void gps_mult8_fast(uint8_t* src1_p, uint8_t* src2_p, uint8_t* dst_p, uint16_t length, uint16_t offset);

void gps_shift_to_zero_freq(
  uint8_t* signal_data, uint8_t* data_i, uint8_t* data_q, uint32_t freq);