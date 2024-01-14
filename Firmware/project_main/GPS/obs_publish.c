#include "config.h"

#if (ENABLE_RTCM_SEND)
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "obs_publish.h"
#include "nav_data_decode.h"
#include "rtk_common.h"
#include "uart_comm.h"
#include <math.h>

//******************************************************************

#define CODE_L1C	1
#define RTCM3PREAMB     0xD3        /* rtcm ver.3 frame preamble */

//Need to be static becase TX data for DMA is stored here
rtcm_t rtcm_data = { 0 };

static const unsigned int tbl_CRC24Q[] = {
  0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
  0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
  0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
  0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
  0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
  0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
  0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
  0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
  0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
  0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
  0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
  0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
  0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
  0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
  0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
  0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
  0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
  0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
  0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
  0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
  0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
  0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
  0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
  0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
  0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
  0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
  0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
  0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
  0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
  0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
  0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
  0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

int gen_rtcm3(rtcm_t *rtcm, int type, int sync);
int init_rtcm(rtcm_t *rtcm);


//****************************************************
//****************************************************

void sdrobs2obsd(gps_ch_t* channels, int ns, obsd_t *out)
{
  int i;
  for (i = 0; i < ns;i++)
  {
    out[i].time = gpst2time(channels[i].eph_data.week_gpst, channels[i].obs_data.tow_s);
    out[i].rcv = 1;
    out[i].sat = channels[i].prn;
    out[i].P[0] = channels[i].obs_data.pseudorange_m;
    out[i].L[0] = 0;
    out[i].D[0] = (float)channels[i].tracking_data.if_freq_offset_hz;
    out[i].SNR[0] = (unsigned char)(channels[i].tracking_data.snr_value + 20.0f) * 4;
    out[i].LLI[0] = 0;
    
    // signal type 
    out[i].code[0] = CODE_L1C;
  }
}


void sendrtcmobs(obsd_t *obsd, int nsat)
{
  init_rtcm(&rtcm_data);
  
  rtcm_data.time = obsd[0].time;
  rtcm_data.obs.n = nsat;
  rtcm_data.obs.data = obsd;
  
  /* GPS rtcm msm */
  gen_rtcm3(&rtcm_data, 1075,0);
  
  uart_prim_dma_send_data(rtcm_data.buff, rtcm_data.nbyte);
}

void sendrtcmnav(gps_ch_t* channel)
{
  memset(&rtcm_data, 0, sizeof(rtcm_data));
  init_rtcm(&rtcm_data);
  rtcm_data.eph = &channel->eph_data.eph;
  rtcm_data.ephsat = channel->prn;
  
  gen_rtcm3(&rtcm_data, 1019, 0); /* rtcm generation */
  uart_prim_dma_send_data(rtcm_data.buff, rtcm_data.nbyte);
}

unsigned int crc24q(const unsigned char *buff, int len)
{
  unsigned int crc = 0;
  int i;
  
  for (i = 0;i < len;i++) 
    crc = ((crc << 8) & 0xFFFFFF) ^ tbl_CRC24Q[(crc >> 16) ^ buff[i]];
  return crc;
}

int gen_rtcm3(rtcm_t *rtcm, int type, int sync)
{
  unsigned int crc;
  int i = 0;
  rtcm->nbit = rtcm->len = rtcm->nbyte = 0;
  
  /* set preamble and reserved */
  setbitu(rtcm->buff, i, 8, RTCM3PREAMB); i += 8;
  setbitu(rtcm->buff, i, 6, 0); i += 6;
  setbitu(rtcm->buff, i, 10, 0); i += 10;
  
  /* encode rtcm 3 message body */
  if (!encode_rtcm3(rtcm, type, sync)) 
    return 0;
  
  /* padding to align 8 bit boundary */
  for (i = rtcm->nbit;i % 8;i++) {
    setbitu(rtcm->buff, i, 1, 0);
  }
  /* message length (header+data) (bytes) */
  if ((rtcm->len = i / 8) >= 3 + 1024) {
    rtcm->nbit = rtcm->len = 0;
    return 0;
  }
  /* message length without header and parity */
  setbitu(rtcm->buff, 14, 10, rtcm->len - 3);
  
  /* crc-24q */
  crc = crc24q(rtcm->buff, rtcm->len);
  setbitu(rtcm->buff, i, 24, crc);
  
  /* length total (bytes) */
  rtcm->nbyte = rtcm->len + 3;
  
  return 1;
}

int init_rtcm(rtcm_t *rtcm)
{
  gtime_t time0 = {0};
  obsd_t data0 = {{0}};
  
  int i, j;
  
  rtcm->staid = rtcm->stah = rtcm->seqno = rtcm->outtype = 0;
  rtcm->time = rtcm->time_s = time0;
  
  rtcm->msg[0] = rtcm->msgtype[0] = rtcm->opt[0] = '\0';
  for (i = 0;i < 6;i++) 
    rtcm->msmtype[i][0] = '\0';
  rtcm->obsflag = 0;
  for (i = 0;i < MAXSAT;i++) 
    for (j = 0;j < NFREQ + NEXOBS;j++) 
    {
      rtcm->cp[i][j] = 0.0;
      rtcm->lock[i][j] = rtcm->loss[i][j] = 0;
      rtcm->lltime[i][j] = time0;
    }
  rtcm->nbyte = rtcm->nbit = rtcm->len = 0;
  
  rtcm->obs.data = NULL;

  /* reallocate memory for observation and ephemris buffer */
  if (!(rtcm->obs.data = (obsd_t *)malloc(sizeof(obsd_t)*MAXSAT)))
  {
    return 0;
  }
  rtcm->obs.n = 0;
  
  for (i = 0;i < MAXSAT;i++)
    rtcm->obs.data[i] = data0;
  return 1;
}

#endif //#if (ENABLE_RTCM_SEND)
