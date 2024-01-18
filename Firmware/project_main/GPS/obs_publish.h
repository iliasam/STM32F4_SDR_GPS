#ifndef _OBS_PUBLISH_H
#define _OBS_PUBLISH_H

#include "stdint.h"
#include "gps_misc.h"
#include "rtk_common.h"

void sendrtcmobs(obsd_t *obsd, int nsat);
void sendrtcmnav(gps_ch_t* channel);

#endif



