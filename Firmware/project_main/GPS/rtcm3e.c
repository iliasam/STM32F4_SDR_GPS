/*------------------------------------------------------------------------------
* rtcm3e.c : rtcm ver.3 message encoder functions
*
*          Copyright (C) 2012-2014 by T.TAKASU, All rights reserved.
*
* references :
*     see rtcm.c
*
* version : $Revision:$ $Date:$
* history : 2012/12/05 1.0  new
*           2012/12/16 1.1  fix bug on ssr high rate clock correction
*           2012/12/24 1.2  fix bug on msm carrier-phase offset correction
*                           fix bug on SBAS sat id in 1001-1004
*                           fix bug on carrier-phase in 1001-1004,1009-1012
*           2012/12/28 1.3  fix bug on compass carrier wave length
*           2013/01/18 1.4  fix bug on ssr message generation
*           2013/05/11 1.5  change type of arg value of setbig()
*           2013/05/19 1.5  gpst -> bdt of time-tag in beidou msm message
*           2013/04/27 1.7  comply with rtcm 3.2 with amendment 1/2 (ref[15])
*                           delete MT 1046 according to ref [15]
*           2014/05/15 1.8  set NT field in MT 1020 glonass ephemeris
*-----------------------------------------------------------------------------*/
#include "rtcm_common.h"
#include "obs_publish.h"
#include "nav_data_decode.h"
#include "math.h"
#include "string.h"
#include "time.h"



static const char rcsid[]="$Id:$";

/* constants and macros ------------------------------------------------------*/

#define PRUNIT_GPS  299792.458          /* rtcm 3 unit of gps pseudorange (m) */
#define PRUNIT_GLO  599584.916          /* rtcm 3 unit of glo pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */
#define P2_10       0.0009765625          /* 2^-10 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */

#define ROUND(x)    ((int)floor((x)+0.5))
#define ROUND_U(x)  ((unsigned int)floor((x)+0.5))
#define MIN(x,y)    ((x)<(y)?(x):(y))



const static double gpst0[] = { 1980,1, 6,0,0,0 }; /* gps time reference */

const char *obscodes[] = {       /* observation code strings */

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

const char *msm_sig_gps[32] = {
	/* GPS: ref [13] table 3.5-87, ref [14][15] table 3.5-91 */
	""  ,"1C","1P","1W","1Y","1M",""  ,"2C","2P","2W","2Y","2M", /*  1-12 */
	""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
	""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
};

/* msm signal id table -------------------------------------------------------*/
extern const char *msm_sig_gps[32];

void setbitu(unsigned char *buff, int pos, int len, unsigned int data)
{
	unsigned int mask = 1u << (len - 1);
	int i;
	if (len <= 0 || 32 < len) return;
	for (i = pos;i < pos + len;i++, mask >>= 1) {
		if (data&mask) buff[i / 8] |= 1u << (7 - i % 8); else buff[i / 8] &= ~(1u << (7 - i % 8));
	}
}

void setbits(unsigned char *buff, int pos, int len, int data)
{
	if (data < 0) data |= 1 << (len - 1); else data &= ~(1 << (len - 1)); /* set sign bit */
	setbitu(buff, pos, len, (unsigned int)data);
}

double time2gpst(gtime_t t, int *week)
{
	gtime_t t0;
	t0.time = 315964800;//Ticks between Unix epoch and GPS epoch
	t0.sec = 0;

	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - w * 86400 * 7) + t.sec;
}

int satsys(int sat, int *prn)
{
	int sys = 0;
	if (sat <= 0) 
		sat = 0;
	else if (sat <= NSATGPS)
	{
		sys = SYS_GPS; sat += MINPRNGPS - 1;
	}
	else sat = 0;
	if (prn) *prn = sat;
	return sys;
}

//double satwavelen(int sat, int frq)
//{
//	return CLIGHT / FREQ1;
//}

char *code2obs(unsigned char code, int *freq)
{
	if (freq) *freq = 0;
	if (code <= CODE_NONE || MAXCODE < code) return "";
	if (freq) *freq = obsfreqs[code];
	return (char *)obscodes[code];
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/* ssr update intervals ------------------------------------------------------*/
static const double ssrudint[16]={
    1,2,5,10,15,30,60,120,240,300,600,900,1800,3600,7200,10800
};
/* set sign-magnitude bits ---------------------------------------------------*/
static void setbitg(unsigned char *buff, int pos, int len, int value)
{
    setbitu(buff,pos,1,value<0?1:0);
    setbitu(buff,pos+1,len-1,value<0?-value:value);
}
/* set signed 38 bit field ---------------------------------------------------*/
static void set38bits(unsigned char *buff, int pos, double value)
{
    int word_h=(int)floor(value/64.0);
    unsigned int word_l=(unsigned int)(value-word_h*64.0);
    setbits(buff,pos  ,32,word_h);
    setbitu(buff,pos+32,6,word_l);
}

double timediff(gtime_t t1, gtime_t t2)
{
	return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}

/* lock time -----------------------------------------------------------------*/
static int locktime(gtime_t time, gtime_t *lltime, unsigned char LLI)
{
    if (!lltime->time || (LLI & 1)) 
		*lltime = time;

    return (int)timediff(time, *lltime);
}

/* lock time indicator (ref [2] table 3.4-2) ---------------------------------*/
static int to_lock(int lock)
{
    if (lock<0  ) return 0;
    if (lock<24 ) return lock;
    if (lock<72 ) return (lock+24  )/2;
    if (lock<168) return (lock+120 )/4;
    if (lock<360) return (lock+408 )/8;
    if (lock<744) return (lock+1176)/16;
    if (lock<937) return (lock+3096)/32;
    return 127;
}
/* msm lock time indicator (ref [11] table 3.4-1D) ---------------------------*/
static int to_msm_lock(int lock)
{
    if (lock<32    ) return 0;
    if (lock<64    ) return 1;
    if (lock<128   ) return 2;
    if (lock<256   ) return 3;
    if (lock<512   ) return 4;
    if (lock<1024  ) return 5;
    if (lock<2048  ) return 6;
    if (lock<4096  ) return 7;
    if (lock<8192  ) return 8;
    if (lock<16384 ) return 9;
    if (lock<32768 ) return 10;
    if (lock<65536 ) return 11;
    if (lock<131072) return 12;
    if (lock<262144) return 13;
    if (lock<524288) return 14;
    return 15;
}
/* msm lock time indicator with extended-resolution (ref [11] table 3.4-1E) --*/
static int to_msm_lock_ex(int lock)
{
    if (lock<0       ) return 0;
    if (lock<64      ) return lock;
    if (lock<128     ) return (lock+64       )/2;
    if (lock<256     ) return (lock+256      )/4;
    if (lock<512     ) return (lock+768      )/8;
    if (lock<1024    ) return (lock+2048     )/16;
    if (lock<2048    ) return (lock+5120     )/32;
    if (lock<4096    ) return (lock+12288    )/64;
    if (lock<8192    ) return (lock+28672    )/128;
    if (lock<16384   ) return (lock+65536    )/256;
    if (lock<32768   ) return (lock+147456   )/512;
    if (lock<65536   ) return (lock+327680   )/1024;
    if (lock<131072  ) return (lock+720896   )/2048;
    if (lock<262144  ) return (lock+1572864  )/4096;
    if (lock<524288  ) return (lock+3407872  )/8192;
    if (lock<1048576 ) return (lock+7340032  )/16384;
    if (lock<2097152 ) return (lock+15728640 )/32768;
    if (lock<4194304 ) return (lock+33554432 )/65536;
    if (lock<8388608 ) return (lock+71303168 )/131072;
    if (lock<16777216) return (lock+150994944)/262144;
    if (lock<33554432) return (lock+318767104)/524288;
    if (lock<67108864) return (lock+671088640)/1048576;
    return 704;
}


/* encode type 1019: gps ephemerides -----------------------------------------*/
static int encode_type1019(rtcm_t *rtcm, int sync)
{
    eph_t *eph = rtcm->eph;
    unsigned int sqrtA,e;
    int i=24,prn,week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,tgd;

    if (satsys(rtcm->ephsat, &prn) != SYS_GPS) 
      return 0;
    week=eph->week%1024;
    toe  =ROUND(eph->toes/16.0);
    toc  =ROUND(time2gpst(eph->toc,NULL)/16.0);
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    deln =ROUND(eph->deln/P2_43/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_43/SC2RAD);
    crs  =ROUND(eph->crs/P2_5 );
    crc  =ROUND(eph->crc/P2_5 );
    cus  =ROUND(eph->cus/P2_29);
    cuc  =ROUND(eph->cuc/P2_29);
    cis  =ROUND(eph->cis/P2_29);
    cic  =ROUND(eph->cic/P2_29);
    af0  =ROUND(eph->f0 /P2_31);
    af1  =ROUND(eph->f1 /P2_43);
    af2  =ROUND(eph->f2 /P2_55);
    tgd  =ROUND(eph->tgd[0]/P2_31);
    
    setbitu(rtcm->buff,i,12,1019     ); i+=12;
    setbitu(rtcm->buff,i, 6,prn      ); i+= 6;
    setbitu(rtcm->buff,i,10,week     ); i+=10;
    setbitu(rtcm->buff,i, 4,eph->sva ); i+= 4;
    setbitu(rtcm->buff,i, 2,eph->code); i+= 2;
    setbits(rtcm->buff,i,14,idot     ); i+=14;
    setbitu(rtcm->buff,i, 8,eph->iode); i+= 8;
    setbitu(rtcm->buff,i,16,toc      ); i+=16;
    setbits(rtcm->buff,i, 8,af2      ); i+= 8;
    setbits(rtcm->buff,i,16,af1      ); i+=16;
    setbits(rtcm->buff,i,22,af0      ); i+=22;
    setbitu(rtcm->buff,i,10,eph->iodc); i+=10;
    setbits(rtcm->buff,i,16,crs      ); i+=16;
    setbits(rtcm->buff,i,16,deln     ); i+=16;
    setbits(rtcm->buff,i,32,M0       ); i+=32;
    setbits(rtcm->buff,i,16,cuc      ); i+=16;
    setbitu(rtcm->buff,i,32,e        ); i+=32;
    setbits(rtcm->buff,i,16,cus      ); i+=16;
    setbitu(rtcm->buff,i,32,sqrtA    ); i+=32;
    setbitu(rtcm->buff,i,16,toe      ); i+=16;
    setbits(rtcm->buff,i,16,cic      ); i+=16;
    setbits(rtcm->buff,i,32,OMG0     ); i+=32;
    setbits(rtcm->buff,i,16,cis      ); i+=16;
    setbits(rtcm->buff,i,32,i0       ); i+=32;
    setbits(rtcm->buff,i,16,crc      ); i+=16;
    setbits(rtcm->buff,i,32,omg      ); i+=32;
    setbits(rtcm->buff,i,24,OMGd     ); i+=24;
    setbits(rtcm->buff,i, 8,tgd      ); i+= 8;
    setbitu(rtcm->buff,i, 6,eph->svh ); i+= 6;
    setbitu(rtcm->buff,i, 1,eph->flag); i+= 1;
    setbitu(rtcm->buff,i, 1,eph->fit>0.0?0:1); i+=1;
    rtcm->nbit=i;
    return 1;
}

/* satellite no to msm satellite id ------------------------------------------*/
static int to_satid(int sys, int sat)
{
    int prn;
    if (satsys(sat,&prn)!=sys) return 0;
    return prn;
}
/* observation code to msm signal id -----------------------------------------*/
static int to_sigid(int sys, unsigned char code, int *freq)
{
    const char **msm_sig = msm_sig_gps;
    char *sig;
    int i;
    
    /* signal conversion for undefined signal by rtcm */
    if (!*(sig=code2obs(code,freq))) return 0;
    for (i=0;i<32;i++) {
        if (!strcmp(sig,msm_sig[i])) return i+1;
    }
    return 0;
}

/* generate msm satellite, signal and cell index -----------------------------*/
static void gen_msm_index(rtcm_t *rtcm, int sys, int *nsat, int *nsig,
	int *ncell, unsigned char *sat_ind,
	unsigned char *sig_ind, unsigned char *cell_ind)
{
	int i, j, sat, sig, cell, f;

	*nsat = *nsig = *ncell = 0;

	/* generate satellite and signal index */
	for (i = 0;i < rtcm->obs.n;i++) {
		if (!(sat = to_satid(sys, rtcm->obs.data[i].sat))) continue;

		for (j = 0;j < NFREQ + NEXOBS;j++) {
			if (!(sig = to_sigid(sys, rtcm->obs.data[i].code[j], &f))) continue;

			sat_ind[sat - 1] = sig_ind[sig - 1] = 1;
		}
	}
	for (i = 0;i < 64;i++) {
		if (sat_ind[i]) sat_ind[i] = ++(*nsat);
	}
	for (i = 0;i < 32;i++) {
		if (sig_ind[i]) sig_ind[i] = ++(*nsig);
	}
	/* generate cell index */
	for (i = 0;i < rtcm->obs.n;i++) {
		if (!(sat = to_satid(sys, rtcm->obs.data[i].sat))) continue;

		for (j = 0;j < NFREQ + NEXOBS;j++) {
			if (!(sig = to_sigid(sys, rtcm->obs.data[i].code[j], &f))) continue;

			cell = sig_ind[sig - 1] - 1 + (sat_ind[sat - 1] - 1)*(*nsig);
			cell_ind[cell] = 1;
		}
	}
	for (i = 0;i < *nsat*(*nsig);i++) {
		if (cell_ind[i] && *ncell < 64) cell_ind[i] = ++(*ncell);
	}
}


/* generate msm satellite data fields ----------------------------------------*/
static void gen_msm_sat(rtcm_t *rtcm, int sys, int nsat,
                        const unsigned char *sat_ind, double *rrng,
                        double *rrate, unsigned char *info)
{
    obsd_t *data;
    double lambda,rrng_s,rrate_s;
    int i,j,k,sat,sig,f;
    
    for (i=0;i<64;i++) rrng[i]=rrate[i]=0.0;
    
    for (i=0;i<rtcm->obs.n;i++) {
        data=rtcm->obs.data+i;
        if (!(sat=to_satid(sys,data->sat))) continue;
        
        for (j=0;j<NFREQ+NEXOBS;j++) {
            if (!(sig=to_sigid(sys,data->code[j],&f))) continue;
            k=sat_ind[sat-1]-1;
            lambda= CLIGHT / FREQ1;
            
            /* rough range (ms) and rough phase-range-rate (m/s) */
            rrng_s =ROUND( data->P[j]/RANGE_MS/P2_10)*RANGE_MS*P2_10;
            rrate_s=ROUND(-data->D[j]*lambda)*1.0;
            if (rrng [k]==0.0&&data->P[j]!=0.0) rrng [k]=rrng_s;
            if (rrate[k]==0.0&&data->D[j]!=0.0) rrate[k]=rrate_s;
            
            /* extended satellite info */
            //info[k]= 0;
        }
    }
}
/* generate msm signal data fields -------------------------------------------*/
static void gen_msm_sig(rtcm_t *rtcm, int sys, int nsat, int nsig, int ncell,
	const unsigned char *sat_ind,
	const unsigned char *sig_ind,
	const unsigned char *cell_ind, const double *rrng,
	const double *rrate, double *psrng, double *phrng,
	double *rate, int *lock, unsigned char *half,
	float *cnr)
{
	obsd_t *data;
	double lambda, psrng_s, phrng_s, rate_s;
	int i, j, k, sat, sig, cell, f, lt, LLI;

	for (i = 0;i < ncell;i++) {
		if (psrng) psrng[i] = 0.0;
		if (phrng) phrng[i] = 0.0;
		if (rate) rate[i] = 0.0;
	}
	for (i = 0;i < rtcm->obs.n;i++) {
		data = rtcm->obs.data + i;

		if (!(sat = to_satid(sys, data->sat))) continue;

		for (j = 0;j < NFREQ + NEXOBS;j++) {
			if (!(sig = to_sigid(sys, data->code[j], &f))) continue;
			k = sat_ind[sat - 1] - 1;
			if ((cell = cell_ind[sig_ind[sig - 1] - 1 + k * nsig]) >= 64) continue;

			lambda = CLIGHT / FREQ1;
			psrng_s = data->P[j] == 0.0 ? 0.0 : data->P[j] - rrng[k];
			phrng_s = data->L[j] == 0.0 || lambda <= 0.0 ? 0.0 : data->L[j] * lambda - rrng[k];
			rate_s = data->D[j] == 0.0 || lambda <= 0.0 ? 0.0 : -data->D[j] * lambda - rrate[k];

			/* subtract phase - psudorange integer cycle offset */
			LLI = data->LLI[j];
			if ((LLI & 1) || fabs(phrng_s - rtcm->cp[data->sat - 1][j]) > 1171.0) {
				rtcm->cp[data->sat - 1][j] = ROUND(phrng_s / lambda)*lambda;
				LLI |= 1;
			}
			phrng_s -= rtcm->cp[data->sat - 1][j];

			lt = locktime(data->time, rtcm->lltime[data->sat - 1] + j, LLI);

			if (psrng&&psrng_s != 0.0) psrng[cell - 1] = psrng_s;
			if (phrng&&phrng_s != 0.0) phrng[cell - 1] = phrng_s;
			if (rate &&rate_s != 0.0) rate[cell - 1] = rate_s;
			if (lock) lock[cell - 1] = lt;
			if (half) half[cell - 1] = (data->LLI[j] & 2) ? 1 : 0;
			if (cnr) cnr[cell - 1] = (float)(data->SNR[j] * 0.25);
		}
	}
}


/* encode msm header ---------------------------------------------------------*/
static int encode_msm_head(int type, rtcm_t *rtcm, int sys, int sync, int *nsat,
                           int *ncell, double *rrng, double *rrate,
                           unsigned char *info, double *psrng, double *phrng,
                           double *rate, int *lock, unsigned char *half,
                           float *cnr)
{
	double tow;
	unsigned char sat_ind[64] = { 0 }, sig_ind[32] = { 0 }, cell_ind[32 * 64] = { 0 };
	unsigned int dow, epoch;
	int i = 24, j, tt, nsig = 0;

	type += 1070;

	/* generate msm satellite, signal and cell index */
	gen_msm_index(rtcm, sys, nsat, &nsig, ncell, sat_ind, sig_ind, cell_ind);


	/* gps, qzs and galileo time (tow-ms) */
	epoch = ROUND_U(time2gpst(rtcm->time, NULL)*1E3);

	/* cumulative session transmitting time (s) */
	tt = locktime(rtcm->time, &rtcm->time_s, 0);

	/* encode msm header (ref [11] table 3.5-73) */
	setbitu(rtcm->buff, i, 12, type); i += 12; /* message number */
	setbitu(rtcm->buff, i, 12, rtcm->staid); i += 12; /* reference station id */
	setbitu(rtcm->buff, i, 30, epoch); i += 30; /* epoch time */
	setbitu(rtcm->buff, i, 1, sync); i += 1; /* multiple message bit */
	setbitu(rtcm->buff, i, 3, rtcm->seqno); i += 3; /* issue of data station */
	setbitu(rtcm->buff, i, 7, to_lock(tt)); i += 7; /* session time indicator */
	setbitu(rtcm->buff, i, 2, 0); i += 2; /* clock streering indicator */
	setbitu(rtcm->buff, i, 2, 0); i += 2; /* external clock indicator */
	setbitu(rtcm->buff, i, 1, 0); i += 1; /* smoothing indicator */
	setbitu(rtcm->buff, i, 3, 0); i += 3; /* smoothing interval */

	/* satellite mask */
	for (j = 0;j < 64;j++) {
		setbitu(rtcm->buff, i, 1, sat_ind[j] ? 1 : 0); i += 1;
	}
	/* signal mask */
	for (j = 0;j < 32;j++) {
		setbitu(rtcm->buff, i, 1, sig_ind[j] ? 1 : 0); i += 1;
	}
	/* cell mask */
	for (j = 0;j < *nsat*nsig&&j < 64;j++) {
		setbitu(rtcm->buff, i, 1, cell_ind[j] ? 1 : 0); i += 1;
	}
	/* generate msm satellite data fields */
	gen_msm_sat(rtcm, sys, *nsat, sat_ind, rrng, rrate, info);

	/* generate msm signal data fields */
	gen_msm_sig(rtcm, sys, *nsat, nsig, *ncell, sat_ind, sig_ind, cell_ind, rrng, rrate,
		psrng, phrng, rate, lock, half, cnr);

	return i;
}
/* encode rough range integer ms ---------------------------------------------*/
static int encode_msm_int_rrng(rtcm_t *rtcm, int i, const double *rrng,
                               int nsat)
{
    unsigned int int_ms;
    int j;
    
    for (j=0;j<nsat;j++) {
        if (rrng[j]==0.0) {
            int_ms=255;
        }
        else if (rrng[j]<0.0||rrng[j]>RANGE_MS*255.0) {
            int_ms=255;
        }
        else {
            int_ms=ROUND_U(rrng[j]/RANGE_MS/P2_10)>>10;
        }
        setbitu(rtcm->buff,i,8,int_ms); i+=8;
    }
    return i;
}
/* encode rough range modulo 1 ms --------------------------------------------*/
static int encode_msm_mod_rrng(rtcm_t *rtcm, int i, const double *rrng,
                               int nsat)
{
    unsigned int mod_ms;
    int j;
    
    for (j=0;j<nsat;j++) {
        if (rrng[j]<=0.0||rrng[j]>RANGE_MS*255.0) {
            mod_ms=0;
        }
        else {
            mod_ms=ROUND_U(rrng[j]/RANGE_MS/P2_10)&0x3FFu;
        }
        setbitu(rtcm->buff,i,10,mod_ms); i+=10;
    }
    return i;
}

/* encode fine pseudorange ---------------------------------------------------*/
static int encode_msm_psrng(rtcm_t *rtcm, int i, const double *psrng, int ncell)
{
    int j,psrng_val;
    
    for (j=0;j<ncell;j++) {
        if (psrng[j]==0.0) {
            psrng_val=-16384;
        }
        else if (fabs(psrng[j])>292.7) {
            psrng_val=-16384;
        }
        else {
            psrng_val=ROUND(psrng[j]/RANGE_MS/P2_24);
        }
        setbits(rtcm->buff,i,15,psrng_val); i+=15;
    }
    return i;
}

/* encode fine phase-range ---------------------------------------------------*/
static int encode_msm_phrng(rtcm_t *rtcm, int i, const double *phrng, int ncell)
{
    int j,phrng_val;
    
    for (j=0;j<ncell;j++) {
        if (phrng[j]==0.0) {
            phrng_val=-2097152;
        }
        else if (fabs(phrng[j])>1171.0) {
            phrng_val=-2097152;
        }
        else {
            phrng_val=ROUND(phrng[j]/RANGE_MS/P2_29);
        }
        setbits(rtcm->buff,i,22,phrng_val); i+=22;
    }
    return i;
}

/* encode lock-time indicator ------------------------------------------------*/
static int encode_msm_lock(rtcm_t *rtcm, int i, const int *lock, int ncell)
{
    int j,lock_val;
    
    for (j=0;j<ncell;j++) {
        lock_val=to_msm_lock(lock[j]);
        setbitu(rtcm->buff,i,4,lock_val); i+=4;
    }
    return i;
}
/* encode lock-time indicator with extended range and resolution -------------*/
static int encode_msm_lock_ex(rtcm_t *rtcm, int i, const int *lock, int ncell)
{
    int j,lock_val;
    
    for (j=0;j<ncell;j++) {
        lock_val=to_msm_lock_ex(lock[j]);
        setbitu(rtcm->buff,i,10,lock_val); i+=10;
    }
    return i;
}
/* encode half-cycle-ambiguity indicator -------------------------------------*/
static int encode_msm_half_amb(rtcm_t *rtcm, int i, const unsigned char *half,
                               int ncell)
{
    int j;
    
    for (j=0;j<ncell;j++) {
        setbitu(rtcm->buff,i,1,half[j]); i+=1;
    }
    return i;
}
/* encode signal cnr ---------------------------------------------------------*/
static int encode_msm_cnr(rtcm_t *rtcm, int i, const float *cnr, int ncell)
{
    int j,cnr_val;
    
    for (j=0;j<ncell;j++) {
        cnr_val=ROUND(cnr[j]/1.0);
        setbitu(rtcm->buff,i,6,cnr_val); i+=6;
    }
    return i;
}

/* encode rough phase-range-rate ---------------------------------------------*/
static int encode_msm_rrate(rtcm_t *rtcm, int i, const double *rrate, int nsat)
{
	int j, rrate_val;

	for (j = 0;j < nsat;j++) {
		if (fabs(rrate[j]) > 8191.0) {
			rrate_val = -8192;
		}
		else {
			rrate_val = ROUND(rrate[j] / 1.0);
		}
		setbits(rtcm->buff, i, 14, rrate_val); i += 14;
	}
	return i;
}

/* encode fine phase-range-rate ----------------------------------------------*/
static int encode_msm_rate(rtcm_t *rtcm, int i, const double *rate, int ncell)
{
	int j, rate_val;

	for (j = 0;j < ncell;j++) {
		if (rate[j] == 0.0) {
			rate_val = -16384;
		}
		else if (fabs(rate[j]) > 1.6384) {
			rate_val = -16384;
		}
		else {
			rate_val = ROUND(rate[j] / 0.0001);
		}
		setbitu(rtcm->buff, i, 15, rate_val); i += 15;
	}
	return i;
}

/* encode msm 4: full pseudorange and phaserange plus cnr --------------------*/
static int encode_msm4(rtcm_t *rtcm, int sys, int sync)
{
    double rrng[64],rrate[64],psrng[64],phrng[64];
    float cnr[64];
    unsigned char half[64];
    int i,nsat,ncell,lock[64];
    
    /* encode msm header */
    if (!(i=encode_msm_head(4,rtcm,sys,sync,&nsat,&ncell,rrng,rrate,NULL,psrng,
                            phrng,NULL,lock,half,cnr))) {
        return 0;
    }
    /* encode msm satellite data */
    i=encode_msm_int_rrng(rtcm,i,rrng ,nsat ); /* rough range integer ms */
    i=encode_msm_mod_rrng(rtcm,i,rrng ,nsat ); /* rough range modulo 1 ms */
    
    /* encode msm signal data */
    i=encode_msm_psrng   (rtcm,i,psrng,ncell); /* fine pseudorange */
    i=encode_msm_phrng   (rtcm,i,phrng,ncell); /* fine phase-range */
    i=encode_msm_lock    (rtcm,i,lock ,ncell); /* lock-time indicator */
    i=encode_msm_half_amb(rtcm,i,half ,ncell); /* half-cycle-amb indicator */
    i=encode_msm_cnr     (rtcm,i,cnr  ,ncell); /* signal cnr */
    rtcm->nbit=i;
    return 1;
}

/* encode msm 5: full pseudorange, phaserange, phaserangerate and cnr --------*/
static int encode_msm5(rtcm_t *rtcm, int sys, int sync)
{
	double rrng[64], rrate[64], psrng[64], phrng[64], rate[64];
	float cnr[64];
	unsigned char info[64], half[64];
	int i, nsat, ncell, lock[64];

	/* encode msm header */
	if (!(i = encode_msm_head(5, rtcm, sys, sync, &nsat, &ncell, rrng, rrate, info, psrng,
		phrng, rate, lock, half, cnr))) {
		return 0;
	}
	/* encode msm satellite data */
	i = encode_msm_int_rrng(rtcm, i, rrng, nsat); /* rough range integer ms */
	//i = encode_msm_info(rtcm, i, info, nsat); /* extended satellite info */
	i += nsat * 4;
	i = encode_msm_mod_rrng(rtcm, i, rrng, nsat); /* rough range modulo 1 ms */
	i = encode_msm_rrate(rtcm, i, rrate, nsat); /* rough phase-range-rate */

	/* encode msm signal data */
	i = encode_msm_psrng(rtcm, i, psrng, ncell); /* fine pseudorange */
	i = encode_msm_phrng(rtcm, i, phrng, ncell); /* fine phase-range */
	i = encode_msm_lock(rtcm, i, lock, ncell); /* lock-time indicator */
	i = encode_msm_half_amb(rtcm, i, half, ncell); /* half-cycle-amb indicator */
	i = encode_msm_cnr(rtcm, i, cnr, ncell); /* signal cnr */
	i = encode_msm_rate(rtcm, i, rate, ncell); /* fine phase-range-rate */
	rtcm->nbit = i;
	return 1;
}


/* encode rtcm ver.3 message -------------------------------------------------*/
int encode_rtcm3(rtcm_t *rtcm, int type, int sync)
{
  int ret=0;
  
  switch (type)
  {
  case 1019: ret = encode_type1019(rtcm, sync);     break;
  case 1074: ret = encode_msm4(rtcm, SYS_GPS, sync); break;
  case 1075: ret = encode_msm5(rtcm, SYS_GPS, sync); break;
  }
  return ret;
}
