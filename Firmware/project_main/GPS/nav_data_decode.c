//Decoding Ephemeris data from satellites
//Notice that data from subframes is stored in common data structure immediately
//without buffering, which is not OK
//This code is partially taken from https://github.com/taroz/GNSS-SDRLIB

#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "nav_data_decode.h"
#include "rtk_common.h"
#include "config.h"
#include <math.h>


void decode_subfrm1(const uint8_t *buff, sdreph_t *eph);
void decode_subfrm2(const uint8_t *buff, sdreph_t *eph);
void decode_subfrm3(const uint8_t *buff, sdreph_t *eph);
void decode_subfrm4(const uint8_t *buff, sdreph_t *eph);
void decode_subfrm5(const uint8_t *buff, sdreph_t *eph);

uint32_t getbitu(const unsigned char *buff, int pos, int len);
int32_t getbits(const unsigned char *buff, int pos, int len);
uint32_t getbitu2(const uint8_t *buff, int p1, int l1, int p2, int l2);
int32_t getbits2(const uint8_t *buff, int p1, int l1, int p2, int l2);

int adjgpsweek(int week);

//******************************************************************


//****************************************************
//****************************************************

uint8_t gps_nav_data_decode_subframe(gps_ch_t* channel)
{
  uint32_t id = getbitu(channel->nav_data.subframe_data, 49, 3); /* subframe ID */
  
  channel->eph_data.eph.sat = channel->prn;
  switch (id) 
  {
  case 1: decode_subfrm1(channel->nav_data.subframe_data, &channel->eph_data); break;
  case 2: decode_subfrm2(channel->nav_data.subframe_data, &channel->eph_data); break;
  case 3: decode_subfrm3(channel->nav_data.subframe_data, &channel->eph_data); break;
  case 4: decode_subfrm4(channel->nav_data.subframe_data, &channel->eph_data); break;
  case 5: decode_subfrm5(channel->nav_data.subframe_data, &channel->eph_data); break;
  }
  
  channel->eph_data.sub_cnt++;
  printf("PRN=%2d SUB ID=%ld\n", channel->prn,  id);
  
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
  eph->received_mask_proc |= 1;
}

void decode_subfrm2(const uint8_t *buff, sdreph_t *eph)
{
  double sqrtA;
  //int oldiode = eph->eph.iode;
  
  eph->tow_gpst = getbitu(buff, 30, 17)*6.0;
  eph->eph.iode = getbitu(buff, 60, 8);
  eph->eph.crs = getbits(buff, 68, 16)*P2_5;
  eph->eph.deln = getbits(buff, 90, 16)*P2_43*SC2RAD;
  eph->eph.M0 = getbits2(buff, 106, 8, 120, 24)*P2_31*SC2RAD;
  eph->eph.cuc = getbits(buff, 150, 16)*P2_29;
  eph->eph.e = getbitu2(buff, 166, 8, 180, 24)*P2_33;
  eph->eph.cus = getbits(buff, 210, 16)*P2_29;
  sqrtA = getbitu2(buff, 226, 8, 240, 24)*P2_19;
  eph->eph.toes = getbitu(buff, 270, 16)*16.0;
  eph->eph.fit = getbitu(buff, 286, 1);
  eph->eph.A = sqrtA * sqrtA;
  eph->eph.toe = gpst2time(eph->eph.week, eph->eph.toes);
  
  /* subframe counter */
  eph->cnt++;
  eph->received_mask |= 2;
  eph->received_mask_proc |= 2;
}

void decode_subfrm3(const uint8_t *buff, sdreph_t *eph)
{
  //int oldiode = eph->eph.iode;
  
  eph->tow_gpst = getbitu(buff, 30, 17)*6.0;
  eph->eph.cic = getbits(buff, 60, 16)*P2_29;
  eph->eph.OMG0 = getbits2(buff, 76, 8, 90, 24)*P2_31*SC2RAD;
  eph->eph.cis = getbits(buff, 120, 16)*P2_29;
  eph->eph.i0 = getbits2(buff, 136, 8, 150, 24)*P2_31*SC2RAD;
  eph->eph.crc = getbits(buff, 180, 16)*P2_5;
  eph->eph.omg = getbits2(buff, 196, 8, 210, 24)*P2_31*SC2RAD;
  eph->eph.OMGd = getbits(buff, 240, 24)*P2_43*SC2RAD;
  eph->eph.iode = getbitu(buff, 270, 8);
  eph->eph.idot = getbits(buff, 278, 14)*P2_43*SC2RAD;
  
  /* subframe counter */
  eph->cnt++;
  eph->received_mask |= 4;
  eph->received_mask_proc |= 4;
}

void decode_subfrm4(const uint8_t *buff, sdreph_t *eph)
{
  eph->tow_gpst = getbitu(buff, 30, 17)*6.0; /* transmission time of subframe */
  eph->cnt++;
  eph->received_mask |= 8;
  eph->received_mask_proc |= 8;
}

void decode_subfrm5(const uint8_t *buff, sdreph_t *eph)
{
  eph->tow_gpst = getbitu(buff, 30, 17)*6.0; /* transmission time of subframe */
  eph->received_mask |= 16;
  eph->received_mask_proc |= 16;
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

int32_t getbits2(const uint8_t *buff, int p1, int l1, int p2, int l2)
{
  if (getbitu(buff, p1, 1))
    return (int32_t)((getbits(buff, p1, l1) << l2) + getbitu(buff, p2, l2));
  else
    return (int32_t)getbitu2(buff, p1, l1, p2, l2);
}

int adjgpsweek(int week)
{
  const int fixed_build_gps_week = 2290;//NOV 2023
  return week + (fixed_build_gps_week - week + 512) / 1024 * 1024;
}
