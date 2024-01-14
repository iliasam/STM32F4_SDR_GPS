#ifndef _RTCM_COMMON_H
#define _RTCM_COMMON_H

#include "stdint.h"
#include "gps_misc.h"
#include "obs_publish.h"

#define MINPRNGPS   1   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#define MAXSAT	4
#define NFREQ	1
#define NEXOBS	0
#define SYS_GPS     0x01                /* navigation system: GPS */
#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */


#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define MAXCODE     48                  /* max number of obs code */


typedef struct {        /* observation data */
	int n;         /* number of obervation data/allocated */
	obsd_t *data;       /* observation data records */
} obs_t;


typedef struct {        /* RTCM control struct type */
  int staid;          /* station id */
  int stah;           /* station health */
  int seqno;          /* sequence number for rtcm 2 or iods msm */
  int outtype;        /* output message type */
  gtime_t time;       /* message time */
  gtime_t time_s;     /* message start time */
  obs_t obs; /* observation data (uncorrected) */
  eph_t *eph;         /* satellite ephemerides */
  int ephsat;         /* update satellite of ephemeris */
  char msg[128];      /* special message */
  char msgtype[256];  /* last message type */
  char msmtype[6][128]; /* msm signal types */
  int obsflag;        /* obs data complete flag (1:ok,0:not complete) */
  double cp[MAXSAT][NFREQ + NEXOBS]; /* carrier-phase measurement */
  unsigned char lock[MAXSAT][NFREQ + NEXOBS]; /* lock time */
  unsigned char loss[MAXSAT][NFREQ + NEXOBS]; /* loss of lock count */
  gtime_t lltime[NSATGPS][NFREQ + NEXOBS]; /* last lock time */
  int nbyte;          /* number of bytes in message buffer */
  int nbit;           /* number of bits in word buffer */
  int len;            /* message length (bytes) */
  unsigned char buff[300]; /* message buffer */
  //unsigned int word;  /* word buffer for rtcm 2 */
  char opt[256];      /* RTCM dependent options */
} rtcm_t;

void setbitu(unsigned char *buff, int pos, int len, unsigned int data);
int encode_rtcm3(rtcm_t *rtcm, int type, int sync);


double timediff(gtime_t t1, gtime_t t2);
gtime_t gpst2time(int week, double sec);
gtime_t timeadd(gtime_t t, double sec);
char *code2obs(unsigned char code, int *freq);
double time2gpst(gtime_t t, int *week);

#endif



