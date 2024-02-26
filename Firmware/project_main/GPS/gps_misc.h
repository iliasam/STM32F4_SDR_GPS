#ifndef _GPS_MISC_H
#define _GPS_MISC_H

#include <stdint.h>
#include <time.h>
#include "config.h"


#define GPS_NAV_WORD_LENGTH	        30 //Length in bits
#define GPS_NAV_SUBFRAME_LENGTH_BYTES	38 //(300/8=38byte)

//Offset between GPS and UTC time, seconds
#define GPS_UTC_TIME_OFFSET_S           18

#define M_PI 3.1415926535f


typedef enum
{
  GPS_ACQ_NEED_FREQ_SEARCH = 0,
  GPS_ACQ_FREQ_SEARCH_RUN,
  GPS_ACQ_FREQ_SEARCH_DONE,
  GPS_ACQ_CODE_PHASE_SEARCH1,
  GPS_ACQ_CODE_PHASE_SEARCH1_DONE,
  GPS_ACQ_CODE_PHASE_SEARCH2,
  GPS_ACQ_CODE_PHASE_SEARCH2_DONE,
  GPS_ACQ_CODE_PHASE_SEARCH3,
  GPS_ACQ_CODE_PHASE_SEARCH3_DONE,
  GPS_ACQ_DONE,
} gps_acq_state_t;

typedef enum
{
  GPS_TRACKNG_IDLE,
  GPS_NEED_PRE_TRACK,
  GPS_PRE_TRACK_RUN,
  GPS_PRE_TRACK_DONE,
  GPS_TRACKING_RUN,
} gps_tracking_state_t;

typedef struct
{
  uint8_t       freq_index; //Freq search index, 0 is -ACQ_SEARCH_FREQ_HZ, max is (ACQ_COUNT-1)
  int16_t       found_freq_offset_hz; //Found coarse doppler offset, hz
  int16_t       given_freq_offset_hz; //Given by user coarse doppler offset, hz
  uint16_t      found_code_phase; //Found code phase value, 0..(2*PRN_LENGTH) range
  
  uint16_t      code_search_start;//Code phase value, 0..(2*PRN_LENGTH) range
  uint16_t      code_search_stop;//Code phase value, 0..(2*PRN_LENGTH) range
  uint16_t      code_hist_step;//Length of histogram bin, step is 0.5 chip
  
  gps_acq_state_t state;
  uint8_t       code_phase_histogram[ACQ_PHASE1_HIST_SIZE];//Used for detecting coarse code phase
  
  uint32_t      start_timestamp;//Used for timeout detectinon
  
  float hist_ratio; //Result quality
} gps_acq_t; //Acquisition information type

typedef struct
{
  uint16_t      code_search_start;//1 step is 0.5 chip, pretracking
  uint16_t      code_search_stop;//1 step is 0.5 chip, pretracking
  
  float	        if_freq_offset_hz;//doppler offset
  uint32_t      if_freq_accum;//NCO accumulator for keeping phase stable
  uint16_t      pre_track_phases[PRE_TRACK_POINTS_MAX_CNT];//Array of code phases, sorted
  uint8_t       pre_track_count;//Incremented at the end of channel time slot
  uint32_t      prev_track_timestamp;
  
  float	        code_phase_fine;//Accurate code phase - 1 step is (0.5chip / 8)
  float	        old_code_phase_fine;//Used by master processing
  uint8_t       code_phase_swap_flag;//Set and clear by master processing

  float	        dll_code_err;//Prev. code error, for filtering
  float	        pll_code_err;//Prev. carrier phase error, for filtering, rad
  int16_t       fll_old_i;
  int16_t       fll_old_q;
  float	        fll_err;//rad
  
  int16_t       pll_check_buf[TRACKING_CH_LENGTH];//For checking that PLL is in right lock
  uint8_t       pll_bad_state_cnt;//Couner of bad state detections
  uint16_t      pll_bad_state_master_cnt;//Counter of bad state detections
  
  uint32_t      i_part_summ; //For SNR calculation
  uint32_t      q_part_summ; //For SNR calculation
  uint16_t      snr_summ_cnt;
  float	        snr_value;
  
#if (ENABLE_CODE_FILTER)  
  uint32_t      filt_start_time_ms; //Time of code fiter start
  uint16_t      code_filt_cnt; //Number of of intergated point
  float	        code_phase_fine_filt;//Filter result - 1 step is (0.5chip / 8)
#endif
  
  gps_tracking_state_t	state;
} gps_tracking_t; //Tracking information type

typedef struct
{
  uint8_t       period_sync_ok_flag;
  uint8_t       right_period_cnt;//Incremented if nav. data period is looking OK
  uint32_t      old_swap_time;//Perv. time of sign switch detection, PRN counter ticks. Not accurate. Updated with period 20ms*n
  uint8_t       old_reminder;//Used for nav bit extraction
  
  uint8_t       accurate_swap_time;//in 0-19 range
  uint8_t       accurate_swap_ok;//accurate_swap_time - OK
  
  uint8_t       last_bit_pos_cnt;
  uint8_t       last_bit_neg_cnt;
  
  uint8_t       inv_polarity_flag;
  uint8_t       polarity_found;
  uint8_t       inv_preabmle_cnt;//Counter of inverte preambles detected
  
  uint8_t       word_buf[GPS_NAV_WORD_LENGTH];//For collecting "word" bits
  uint8_t       word_cnt;//Number of received words of current subframe
  uint8_t       word_bit_cnt;//Used when preamble is found
  uint8_t       old_D29;//Previous word bit 29, used for parity calc, 0/1
  uint8_t       old_D30;//Previous word bit 30, used for parity calc.
  
  uint32_t      word_detection_timestamp;//Time when last correct word was detected, in 1ms ticks
  uint32_t      word_cnt_test;
  
  uint32_t      last_subframe_time;//Time of the subframe start in PRN/ms, updated with 6s period, based on accur time.
  uint32_t      first_subframe_time;//Time of the subframe start in PRN/ms, updated once, for detecting ref. sat.
  uint16_t      subframe_cnt;//Subframe counter, increased in tracking, reset once by master processing
  uint8_t       new_subframe_flag;//Reset by master processing
  
  uint8_t       subframe_data[GPS_NAV_SUBFRAME_LENGTH_BYTES];//Every bit here is one nav. data bit
} gps_nav_data_t; //Nav. data information type

typedef struct
{
  double pseudorange_m;
  double tow_s;
} gps_obs_data_t;

// Types below are taken from RTKLIB

typedef struct { /* time struct */
  time_t time;  /* time (s) expressed by standard time_t */
  double sec;   /* fraction of second under 1 s */
} gtime_t;

typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type */
  int sat;            /* satellite number */
  int iode, iodc;      /* IODE,IODC */
  int sva;            /* SV accuracy (URA index) */
  int svh;            /* SV health (0:ok) */
  int week;           /* GPS/QZS: gps week, GAL: galileo week */
  int code;           /* GPS/QZS: code on L2, GAL/CMP: data sources */
  int flag;           /* GPS/QZS: L2 P data flag, CMP: nav type */
  gtime_t toe, toc, ttr; /* Toe,Toc,T_trans */
  /* SV orbit parameters */
  double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
  double crc, crs, cuc, cus, cic, cis;
  double toes;        /* Toe (s) in week */
  double fit;         /* fit interval (h) */
  double f0, f1, f2;    /* SV clock parameters (af0,af1,af2) */
  double tgd[4];      /* group delay parameters */
  /* GPS/QZS:tgd[0]=TGD */
} eph_t;

typedef struct {
  eph_t eph;           /* GPS/QZS/GAL/COM ephemeris struct (from rtklib.h) */
  int ctype;           /* code type */
  double tow_gpst;     /* ephemeris tow in GPST */
  int week_gpst;       /* ephemeris week in GPST */
  int cnt;             /* ephemeris decode counter */
  int cntth;           /* ephemeris decode count threshold */
  int update;          /* ephemeris update flag, set when new eph. is received */
  int prn;             /* PRN */
  int week_gst;
  
  uint16_t sub_cnt;
  
  uint8_t received_mask;//for sending RTCM, cleared after send
  uint8_t received_mask_proc;//for processing, not cleared
} sdreph_t;

typedef struct
{
  gps_acq_t             acq_data;//Acq variables
  gps_tracking_t        tracking_data;
  gps_nav_data_t        nav_data;
  gps_obs_data_t	obs_data;
  sdreph_t		eph_data;
  uint8_t	        prn; //Sat PRN code
  uint8_t	        prn_code[PRN_LENGTH];//PRN data (0/1), generated at the start, 1023 bytes
} gps_ch_t;

void gps_fill_summ_table(void);
void gps_channell_prepare(gps_ch_t* channel);

int16_t gps_correlation8(
  uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, uint16_t offset);
void gps_correlation_iq(
  uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q, uint16_t offset, 
  int16_t* res_i, int16_t* res_q);

uint16_t correlation_search(
  uint16_t* prn_p, uint16_t* data_i, uint16_t* data_q,
  uint16_t start_shift, uint16_t stop_shift, uint16_t* aver_val, uint16_t* phase);

void gps_shift_to_zero_freq(
  uint8_t* signal_data, uint8_t* data_i, uint8_t* data_q, float freq_hz);
void gps_shift_to_zero_freq_track(
  gps_tracking_t* trk_channel, uint8_t* signal_data, uint8_t* data_i, uint8_t* data_q);

void gps_generate_prn_data2(
  gps_ch_t* channel, uint16_t* data, uint16_t offset_bits);

void gps_rewind_if_phase(gps_tracking_t* trk_channel, uint8_t steps);

#endif //_GPS_MISC_H
