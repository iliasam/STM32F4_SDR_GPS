#ifndef _OBS_PUBLISH_H
#define _OBS_PUBLISH_H

#include "stdint.h"
#include "gps_misc.h"

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


void sdrobs2obsd(gps_ch_t* channels, int ns, obsd_t *out);

void sendrtcmobs(obsd_t *obsd, int nsat);
void sendrtcmnav(gps_ch_t* channel);

#endif



