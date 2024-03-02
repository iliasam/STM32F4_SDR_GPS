//Based on RTKLIB

#ifndef _GPS_SOLVING_H
#define _GPS_SOLVING_H

#include "stdint.h"
#include "gps_misc.h"
#include "obs_publish.h"
#include "rtk_common.h"

#define SOLQ_NONE   0
#define SOLQ_SINGLE 5

#define PI          3.1415926535897932  /* pi */
#define R2D         (180.0/PI)          /* rad to deg */

typedef struct {        /* solution type */
	gtime_t time;       /* time (GPST) */
	double rr[6];       /* position/velocity (m|m/s) */
						/* {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu} */
	float  qr[6];       /* position variance/covariance (m^2) */
						/* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
						/* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
	double dtr[6];      /* receiver clock bias to time systems (s) */
	unsigned char type; /* type (0:xyz-ecef,1:enu-baseline) */
	unsigned char stat; /* solution status (SOLQ_???) */
	unsigned char ns;   /* number of valid satellites */
	float age;          /* age of differential (s) */
	float ratio;        /* AR ratio factor for valiation */
} sol_t;

void gps_pos_solve_init(gps_ch_t* channels);
void gps_pos_solve(obsd_t *obs_p);
void ecef2pos(const double *r, double *pos);
int pntpos(const obsd_t *obs, int n, const nav_t *nav, sol_t *sol);
int pntpos_iterative(const obsd_t *obs, int n, const nav_t *nav, sol_t *sol);
uint8_t solving_is_busy(void);


void solve_test(void);
#endif



