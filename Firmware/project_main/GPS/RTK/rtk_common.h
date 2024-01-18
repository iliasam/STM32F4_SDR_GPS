#ifndef _RTCM_COMMON_H
#define _RTCM_COMMON_H

#include "stdint.h"
#include "gps_misc.h"

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
#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */


#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define MAXCODE     48                  /* max number of obs code */

typedef struct {        /* observation data record */
  gtime_t time;       /* receiver sampling time (GPST) */
  unsigned char sat, rcv; /* satellite/receiver number */
  unsigned char SNR[1]; /* signal strength (0.25 dBHz) */
  unsigned char LLI[1]; /* loss of lock indicator */
  unsigned char code[1]; /* code indicator (CODE_???) */
  double L[1]; /* observation data carrier-phase (cycle) */
  double P[1]; /* observation data pseudorange (m) */
  float  D[1]; /* observation data doppler frequency (Hz) */
} obsd_t;

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

typedef struct {        /* navigation data type */
  int n;                /* number of broadcast ephemeris */
  eph_t *eph[MAXSAT];   /* GPS ephemeris */
  double ion_gps[8];    /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
} nav_t;

void setbitu(unsigned char *buff, int pos, int len, unsigned int data);
int encode_rtcm3(rtcm_t *rtcm, int type, int sync);


double timediff(gtime_t t1, gtime_t t2);
gtime_t gpst2time(int week, double sec);
gtime_t timeadd(gtime_t t, double sec);
char *code2obs(unsigned char code, int *freq);
double time2gpst(gtime_t t, int *week);
void sdrobs2obsd(gps_ch_t* channels, int ns, obsd_t *out);
#endif



