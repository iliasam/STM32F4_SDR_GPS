#include "rtk_common.h"
#include "math.h"

#define UNIX2GPS        315964800

static char *obscodes2[] = {       /* observation code strings */
  
  ""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
  "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
  "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
  "6A","6B","6C","6X","6Z", "6S","6L","8L","8Q","8X", /* 30-39 */
  "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q",""    /* 40-49 */
};

const unsigned char obsfreqs[] = { /* 1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,7:L3 */
  
  0, 1, 1, 1, 1,  1, 1, 1, 1, 1, /*  0- 9 */
  1, 1, 1, 1, 2,  2, 2, 2, 2, 2, /* 10-19 */
  2, 2, 2, 2, 3,  3, 3, 5, 5, 5, /* 20-29 */
  4, 4, 4, 4, 4,  4, 4, 6, 6, 6, /* 30-39 */
  2, 2, 4, 4, 3,  3, 3, 1, 1, 0  /* 40-49 */
};

double timediff(gtime_t t1, gtime_t t2)
{
  return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}

//From rtkcm.c
gtime_t gpst2time(int week, double sec)
{
  gtime_t t;
  t.time = UNIX2GPS;//Ticks between Unix epoch and GPS epoch
  t.sec = 0;
  
  if (sec < -1E9 || 1E9 < sec)
    sec = 0.0;
  t.time += 86400 * 7 * week + (int)sec;
  t.sec = sec - (int)sec;
  return t;
}

gtime_t timeadd(gtime_t t, double sec)
{
  t.sec += sec;
  sec = floor(t.sec);
  t.sec += sec;
  t.sec -= sec;
  return t;
}

char *code2obs(unsigned char code, int *freq)
{
  if (freq) *freq = 0;
  if (code <= CODE_NONE || MAXCODE < code) return "";
  if (freq) *freq = obsfreqs[code];
  return (char *)obscodes2[code];
}

double time2gpst(gtime_t t, int *week)
{
  gtime_t t0;
  t0.time = UNIX2GPS;//Ticks between Unix epoch and GPS epoch
  t0.sec = 0;
  
  time_t sec = t.time - t0.time;
  int w = (int)(sec / (86400 * 7));
  
  if (week) *week = w;
  return (double)(sec - w * 86400 * 7) + t.sec;
}

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