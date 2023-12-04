#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "nav_data_decode.h"
#include "config.h"
#include <math.h>

#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */

void decode_subfrm1(const uint8_t *buff, sdreph_t *eph);

uint32_t getbitu(const unsigned char *buff, int pos, int len);
int32_t getbits(const unsigned char *buff, int pos, int len);
uint32_t getbitu2(const uint8_t *buff, int p1, int l1, int p2, int l2);

int adjgpsweek(int week);

gtime_t gpst2time(int week, double sec);

//******************************************************************


//****************************************************
//****************************************************

uint8_t gps_nav_data_decode_subframe(gps_ch_t* channel)
{
  uint32_t id = getbitu(channel->nav_data.subframe_data, 49, 3); /* subframe ID */
  
  switch (id) 
  {
  case 1: decode_subfrm1(channel->nav_data.subframe_data, &channel->eph_data); break;
  //case 2: decode_subfrm2(buff, eph); break;
  //case 3: decode_subfrm3(buff, eph); break;
  }
  printf("SUB ID=%ld\n", id);
  
  return (uint8_t)id;
}

void decode_subfrm1(const uint8_t *buff, sdreph_t *eph)
{
  double toc;
  int week;
  
  eph->tow_gpst = getbitu(buff, 30, 17)*6.0;
  week = getbitu(buff, 60, 10) + 1024;
  eph->eph.code = getbitu(buff, 70, 2);
  eph->eph.sva = getbitu(buff, 72, 4);
  eph->eph.svh = getbitu(buff, 76, 6);
  eph->eph.iodc = getbitu2(buff, 82, 2, 210, 8);
  eph->eph.flag = getbitu(buff, 90, 1);
  eph->eph.tgd[0] = getbits(buff, 196, 8)*P2_31;
  toc = getbitu(buff, 218, 16)*16.0;
  eph->eph.f2 = getbits(buff, 240, 8)*P2_55;
  eph->eph.f1 = getbits(buff, 248, 16)*P2_43;
  eph->eph.f0 = getbits(buff, 270, 22)*P2_31;
  
  eph->eph.week = adjgpsweek(week);
  eph->week_gpst = eph->eph.week;
  eph->eph.ttr = gpst2time(eph->eph.week, eph->tow_gpst);
  eph->eph.toc = gpst2time(eph->eph.week, toc);
  
  /* subframe decode counter */
  eph->cnt++;
  eph->received_mask |= 1;
}

//**********************************************************

uint32_t getbitu(const unsigned char *buff, int pos, int len)
{
  uint32_t bits = 0;
  int i;
  for (i = pos; i < pos + len; i++) 
    bits = (bits << 1) + ((buff[i / 8] >> (i % 8)) & 1u);
  return bits;
}

int32_t getbits(const unsigned char *buff, int pos, int len)
{
  unsigned int bits = getbitu(buff, pos, len);
  if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) 
    return (int)bits;
  return (int)(bits | (~0u << len)); /* extend sign */
}

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data (two components case)
* args   : uint8_t *buff    I   byte data
*          int    p1        I   first bit start position (bits)
*          int    l1        I   first bit length (bits)
*          int    p2        I   second bit start position (bits)
*          int    l2        I   seconf bit length (bits)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
uint32_t getbitu2(const uint8_t *buff, int p1, int l1, int p2, int l2)
{
  return (getbitu(buff, p1, l1) << l2) + getbitu(buff, p2, l2);
}


int adjgpsweek(int week)
{
  const int fixed_build_gps_week = 2290;//NOV 2023
  return week + (fixed_build_gps_week - week + 512) / 1024 * 1024;
}

//From rtkcm.c
gtime_t gpst2time(int week, double sec)
{
  gtime_t t;
  t.time = 315964800;//Ticks between Unix epoch and GPS epoch
  t.sec = 0;
  
  if (sec < -1E9 || 1E9 < sec) 
    sec = 0.0;
  t.time += 86400 * 7 * week + (int)sec;
  t.sec = sec - (int)sec;
  return t;
}

