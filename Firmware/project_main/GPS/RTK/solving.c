//Part of this code is taken from here: https://github.com/ndhuan/GPSRTK

//Finding receiver position by observations (time+pseudoranges) and 
//navigation data (ephemeris)
//Result is "gps_sol" and final_pos[] global variables

#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "gps_misc.h"
#include "nav_data.h"
#include "nav_data_decode.h"
#include "solving.h"
#include "config.h"
#include <math.h>
#include "delay_us_timer.h"


#define STD_BRDCCLK 30.0          /* error of broadcast clock (m) */
#define SQR(x)   ((x)*(x))
#define MAXDTOE     7200.0 
#define MU_GPS      3.9860050E14     /* gravitational constant */
#define OMGE        7.2921151467E-5     /* earth angular velocity (IS-GPS) (rad/s) */
#define RTOL_KEPLER 1E-14         /* relative tolerance for Kepler equation */
#define MAX_ITER_KEPLER 30        /* max number of iteration of Kelpler */


#define MAXITR      10          /* max number of iteration for point pos */
#define NX          (4+3)       /* # of estimated parameters */

#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */

#define ERR_CBIAS   0.3         /* code bias error std (m) */
#define EFACT_GPS   1.0
#define ERR_BRDCI   0.5         /* broadcast iono model error factor */
#define EVAR        0.003

#define REL_HUMI    0.7         /* relative humidity for saastamoinen model */
#define ERR_SAAS    0.3         /* saastamoinen model error std (m) */

#define sos4(x) (x[0]*x[0]+x[1]*x[1]+x[2]*x[2]+x[3]*x[3])

//******************************************************************
//ephemeris
nav_t nav_data = { 0 };
/// GPS positon solution
sol_t gps_sol = { {0} };
double final_pos[3];//geodetic position {lat,lon,h} (deg,m)
double azel[2 * MAXSAT];//satellites az/elevations, deg
uint8_t flag_solving_busy = 0;
uint8_t flag_processing_busy = 0;

volatile static uint32_t diff_solve_ms;//debug

double *mat(int n, int m);
int pntpos(const obsd_t *obs, int n, const nav_t *nav, sol_t *sol);
int satposs(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
	int ephopt, double *rs, double *dts, double *var, int *svh);
int satposs_iterative(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
  int ephopt, double *rs, double *dts, double *var, int *svh);
static int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav, double *dts);
int satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
	const nav_t *nav, double *rs, double *dts, double *var,
	int *svh);
static int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
	int iode, double *rs, double *dts, double *var, int *svh);
void eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts,
	double *var);
static int estpos(const obsd_t *obs, int n, const double *rs, const double *dts,
  const double *vare, const int *svh, const nav_t *nav, 
  sol_t *sol, double *azel, int *vsat, double *resp);
int estpos_iterative(const obsd_t *obs, int n, const double *rs, const double *dts,
  const double *vare, const int *svh, const nav_t *nav, 
  sol_t *sol, double *azel, int *vsat, double *resp);

void ecef2pos(const double *r, double *pos);
double geodist(const double *rs, const double *rr, double *e);
gtime_t timeadd(gtime_t t, double sec);

static int rescode(int iter, const obsd_t *obs, int n, const double *rs,
	const double *dts, const double *vare, const int *svh,
	const nav_t *nav, const double *x,
	double *v, double *H, double *var, double *azel, int *vsat,
	double *resp, int *ns);
static int rescode_iterative(int iter, const obsd_t *obs, int n, const double *rs,
  const double *dts, const double *vare, const int *svh,
  const nav_t *nav, const double *x,
  double *v, double *H, double *var, double *azel, int *vsat,
  double *resp, int *ns);

double satazel(const double *pos, const double *e, double *azel);
void ecef2enu(const double *pos, const double *r, double *e);
int lsq(const double *A, const double *y, int n, int m, double *x, double *Q);



//****************************************************
//****************************************************

//Initiaize pointers
void gps_pos_solve_init(gps_ch_t* channels)
{
  //Copy pointers
  for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
  {
    nav_data.eph[i] = &channels[i].eph_data.eph;
  }
  nav_data.n = GPS_SAT_CNT;
}

// Main function, disigned to be iterative
// User need to call it until solving_is_busy() return 0
// Expected that any iteration takees less than 1ms
void gps_pos_solve(obsd_t *obs_p)
{
  uint32_t start_t = get_dwt_value();
  if (flag_processing_busy)
  {
    //Convert result to angular form
    ecef2pos(gps_sol.rr, final_pos);//radians
    final_pos[0] = final_pos[0] * R2D;
    final_pos[1] = final_pos[1] * R2D;
    flag_processing_busy = 0;
    //This iteration is short - 100us
  }
  else if (pntpos_iterative(obs_p, GPS_SAT_CNT, &nav_data, &gps_sol) > 0)
  {
    flag_processing_busy = 1;
  }
  uint32_t stop_t = get_dwt_value();
  diff_solve_ms = stop_t - start_t;
  diff_solve_ms = diff_solve_ms / 168;
  printf("TIME: %d\n", diff_solve_ms);
  if (diff_solve_ms > 900)
    printf("TIMEOUT\n");
}

/* single-point positioning ----------------------------------------------------
* compute receiver position, velocity, clock bias by single-point positioning
* with pseudorange and doppler observables
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          sol_t  *sol      IO  solution
* return : status(1:ok,0:error)
* notes  : assuming sbas-gps, galileo-gps, qzss-gps, compass-gps time offset and
*          receiver bias are negligible (only involving glonass-gps time offset
*          and receiver bias)
*-----------------------------------------------------------------------------*/
int pntpos(const obsd_t *obs, int n, const nav_t *nav, sol_t *sol)
{
  double *rs, *dts, *var, *resp;
  
  int stat = 0, vsat[4] = { 0 }, svh[4];
  
  sol->stat = SOLQ_NONE;
  if (n <= 0) { return 0; }
  sol->time = obs[0].time;
  
  //__iar_dlmalloc_stats();
  
  rs = mat(6, n);
  dts = mat(2, n);
  var = mat(1, n);
  resp = mat(1, n);
  
  /* satellite positons, velocities and clocks */
  satposs(sol->time, obs, n, nav, 0, rs, dts, var, svh);
  
  /* estimate receiver position with pseudorange */
  stat = estpos(obs, n, rs, dts, var, svh, nav, sol, azel, vsat, resp);
  
  for (uint8_t i = 0; i < (2 * MAXSAT); i++)
    azel[i] = azel[i] * R2D;
  
  free(rs);free(dts);free(var);free(resp);
  return stat;
}

//Same as pntpos(), but iterative
//Return 0 if not completed, 1 if OK, <0 if error
int pntpos_iterative(const obsd_t *obs, int n, const nav_t *nav, sol_t *sol)
{
  static double *rs, *dts, *var, *resp; //malloc pointers
  static int vsat[4] = { 0 }, svh[4];
  static uint8_t satposs_busy = 0;
  static uint8_t estpos_busy = 0;
  
  if (flag_solving_busy == 0)
  {
    //Init
    rs = mat(6, n);
    dts = mat(2, n);
    var = mat(1, n);
    resp = mat(1, n);
    memset(vsat, 0, sizeof(vsat));
    memset(svh, 0, sizeof(svh));
    
    sol->stat = SOLQ_NONE;
    if (n <= 0) 
      return -2;
    sol->time = obs[0].time;
    
    flag_solving_busy = 1;
    satposs_busy = 1;
    printf("Code1\n");
  }
  
  int res;
  if (satposs_busy)
  {
    /* satellite positons, velocities and clocks */
    //Take near 600 us
    res = satposs_iterative(sol->time, obs, n, nav, 0, rs, dts, var, svh);
    if (res == 0)
      return 0;//not completed
    else if (res < 0)
    {
      //FAIL
      satposs_busy = 0;
    }
    else
    {
      //OK
      satposs_busy = 0;
      estpos_busy = 1;
      printf("Code2\n");
      return 0;//not completed
    }
  }
  
  if (estpos_busy)
  {
    /* estimate receiver position with pseudorange */
    //Take near 700 us
    res = estpos_iterative(obs, n, rs, dts, var, svh, nav, sol, azel, vsat, resp);
    
    if ((res < 0) || (res > 0))
    {
      estpos_busy = 0;
      flag_solving_busy = 0;
      if (res > 0)//Pos. found
      {
        printf("Code3\n");
        // Convert to deg.
        for (uint8_t i = 0; i < (2 * MAXSAT); i++)
          azel[i] = azel[i] * R2D;
      }
    }
    else
    {
      return 0;//not completed
    }
  }
  
  flag_solving_busy = 0;
  free(rs);free(dts);free(var);free(resp);
  
  if (res < 0)
    return -1;
  
  return 1;
}

uint8_t solving_is_busy(void)
{
  return (flag_solving_busy | flag_processing_busy);
}

/* new matrix ------------------------------------------------------------------
* allocate memory of matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
double *mat(int n, int m)
{
  double *p;
  if (n <= 0 || m <= 0) 
    return NULL;
  
  if (!(p = (double *)malloc(sizeof(double)*n*m))) {
    printf("matrix memory allocation error 1: n=%d,m=%d\n", n, m);
    return NULL;
  }
  return p;
}

/* new integer matrix ----------------------------------------------------------
* allocate memory of integer matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
int *imat(int n, int m)
{
  int *p;
  
  if (n<=0||m<=0) return NULL;
  if (!(p=(int *)malloc(sizeof(int)*n*m))) 
  {
    printf("integer matrix memory allocation error 2: n=%d,m=%d\n",n,m);
    return NULL;
  }
  return p;
}

/* zero matrix -----------------------------------------------------------------
* generate new zero matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern double *zeros(int n, int m)
{
  double *p;
  
  if (n <= 0 || m <= 0) return NULL;
  if (!(p = (double *)calloc(sizeof(double), n*m))) {
    printf("matrix memory allocation error 3: n=%d,m=%d\n", n, m);
    return NULL;
  }
  return p;
}

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
double dot(const double *a, const double *b, int n)
{
  double c = 0.0;
  while (--n >= 0) c += a[n] * b[n];
  return c;
}

double norm(const double *a, int n)
{
  return sqrt(dot(a, a, n));
}

void matcpy(double *A, const double *B, int n, int m)
{
  memcpy(A, B, sizeof(double)*n*m);
}

/*
void solve_test(void)
{
  double x[NX] = { 0 }, dx[NX], Q[NX*NX], *v, *H, *var, sig;
  uint8_t n = 4;
  v = mat(n + 4, 1); 
  H = mat(NX, n + 4); 
  int nv = 7;

  uint32_t start_t = get_dwt_value();
  int info = lsq(H, v, NX, nv, dx, Q);
  uint32_t stop_t = get_dwt_value();
  diff = stop_t - start_t;
  diff = diff / 168;
  
  printf("%f\n", Q[10]);
  printf("%d\n", diff);
  if (info > 0) 
  {
    printf("SOLVE ERR=%d\n", info);
    return;
  }
}
*/


/* estimate receiver position ------------------------------------------------*/
static int estpos(const obsd_t *obs, int n, const double *rs, const double *dts,
                  const double *vare, const int *svh, const nav_t *nav, sol_t *sol, double *azel, int *vsat, double *resp)
{
  double x[NX] = { 0 }, dx[NX], Q[NX*NX], *v, *H, *var, sig;
  int i, j, k, stat, nv, ns;
  
  v = mat(n + 4, 1); 
  H = mat(NX, n + 4); 
  var = mat(n + 4, 1);
  
  for (i = 0;i < 3;i++) 
    x[i] = sol->rr[i];
  
  for (i = 0;i < MAXITR;i++) 
  {
    /* pseudorange residuals */
    nv = rescode(i, obs, n, rs, dts, vare, svh, nav, x, v, H, var, azel, vsat, resp,
                 &ns);
    if (nv < NX) {
      //sprintf(msg, "lack of valid sats ns=%d", nv);
      break;
    }
    
    // weight by variance 
    for (j = 0;j < nv;j++)
    {
      sig = sqrt(var[j]);
      v[j] /= sig;
      for (k = 0;k < NX;k++) H[k + j * NX] /= sig;
    }
    
    // least square estimation 
    int info = lsq(H, v, NX, nv, dx, Q);
    /*
    if (info > 0) 
    {
      printf("SOLVE ERR=%d\n", info);
      break;
    }
    */
    
    for (j = 0;j < NX;j++) 
      x[j] += dx[j];
    
    if (sos4(dx) < 1E-8)
    {
      //POSITION FOUND!
      sol->type = 0;
      sol->time = timeadd(obs[0].time, -x[3] / CLIGHT);
      sol->dtr[0] = x[3] / CLIGHT; //receiver clock bias (s) 
      for (j = 0;j < 6;j++) sol->rr[j] = j < 3 ? x[j] : 0.0;
      for (j = 0;j < 3;j++) sol->qr[j] = (float)Q[j + j * NX];
      sol->qr[3] = (float)Q[1];    // cov xy 
      sol->qr[4] = (float)Q[2 + NX]; // cov yz 
      sol->qr[5] = (float)Q[2];    // cov zx 
      sol->ns = (unsigned char)ns;
      sol->age = sol->ratio = 0.0;
      stat = 1;
      
      // validate solution 
      //if ((stat = valsol(azel, vsat, n, v, nv, NX))) 
      {
        sol->stat = SOLQ_SINGLE;
      }
      free(v); free(H); free(var);
      
      return stat;
    }
  }
  sol->stat = SOLQ_NONE;
  free(v); free(H); free(var);
  
  return 0;
}

//Same as estpos(), but iterative
//Return 0 if not completed, 1 if OK, <0 if error
int estpos_iterative(const obsd_t *obs, int n, const double *rs, const double *dts,
  const double *vare, const int *svh, const nav_t *nav, 
  sol_t *sol, double *azel, int *vsat, double *resp)
{
  static uint8_t iteration_idx = 0;
  static double x[NX] = { 0 }, dx[NX], Q[NX*NX], *v, *H, *var, sig;
  static int ns, nv;
  int i, j, k, info;
  int res = 0;
  
  static uint8_t operation = 0;
  
  if ((iteration_idx == 0) && (operation == 0))
  { 
    //Init
    operation = 0;
    v = mat(n + 4, 1); 
    H = mat(NX, n + 4); 
    var = mat(n + 4, 1);
    memset(x, 0, sizeof(x));
    
    for (i = 0; i < 3; i++)
      x[i] = sol->rr[i];
  }
  
  while (1)
  {
    //rescode() take near 1.5ms if ionocorr() and tropcorr() are used
    //So iterative mode is needed.
    //rescode() can be used here without ionocorr() and tropcorr() enabled
    //So only two operations (rescode() and lsq()) will be needed
    
    if (operation < (n - 1))//n-1=3
    {
      int res2;
      res2 = rescode_iterative(i, obs, n, rs, dts, vare, svh,
        nav, x, v, H, var, azel, vsat, resp, &ns);
      if (res2 >= 0)
      {
        //unexpected
        res = -1;
        break;
      }
      
      operation++;
      return 0;//not completed
    }
    else if (operation == (n - 1))
    {
      nv = rescode_iterative(i, obs, n, rs, dts, vare, svh,
        nav, x, v, H, var, azel, vsat, resp, &ns);
      
      if (nv < NX)
      {
        //sprintf(msg, "lack of valid sats ns=%d", nv);
        res = -1;
        break;
      }
      else
      {
        // weight by variance 
        for (j = 0;j < nv;j++)
        {
          sig = sqrt(var[j]);
          v[j] /= sig;
          for (k = 0;k < NX;k++) 
            H[k + j * NX] /= sig;
        }
        
        operation++;
        return 0;
      }
    }
    else if (operation == n)
    {
      // least square estimation 
      info = lsq(H, v, NX, nv, dx, Q);
      if (info > 0)
      {
        //sprintf(msg, "lsq error info=%d", info);
        res = -2;
        break;
      }
      
      for (j = 0;j < NX;j++)
        x[j] += dx[j];
      
      if (sos4(dx) < 1E-8)
      {
        res = 1;//position found
      }
      else
      {
        //need new iteration
        iteration_idx++;
        operation = 0;
        res = 0;
      }
    }
    
    break;
  }//end of while
  
  if (iteration_idx > MAXITR)
  {
    //Too much iterations
    res = -1;
  }
  
  if (res < 0)
  {
    iteration_idx = 0;
    operation = 0;
    free(v); free(H); free(var);
  }
  
  if (res > 0)
  {
    sol->type = 0;
    sol->time = timeadd(obs[0].time, -x[3] / CLIGHT);
    sol->dtr[0] = x[3] / CLIGHT; //receiver clock bias (s) 
    for (j = 0;j < 6;j++) sol->rr[j] = j < 3 ? x[j] : 0.0;
    for (j = 0;j < 3;j++) sol->qr[j] = (float)Q[j + j * NX];
    sol->qr[3] = (float)Q[1];    // cov xy 
    sol->qr[4] = (float)Q[2 + NX]; // cov yz 
    sol->qr[5] = (float)Q[2];    // cov zx 
    sol->ns = (unsigned char)ns;
    sol->age = sol->ratio = 0.0;
    sol->stat = SOLQ_SINGLE;
    
    iteration_idx = 0;
    operation = 0;
    free(v); free(H); free(var);
  }
  return res;
}


static double varerr(double el)
{
  double fact, varr;
  fact = EFACT_GPS;
  varr = SQR(EVAR)*(SQR(EVAR) + SQR(EVAR) / sin(el));
  return SQR(fact)*varr;
}

/* get tgd parameter (m) -----------------------------------------------------*/
static double gettgd(int sat, const nav_t *nav)
{
  int i;
  for (i = 0;i < nav->n;i++)
  {
    if (nav->eph[i]->sat != sat) 
      continue;
    return CLIGHT * nav->eph[i]->tgd[0];
  }
  return 0.0;
}

/* ionosphere model ------------------------------------------------------------
* compute ionospheric delay by broadcast ionosphere model (klobuchar model)
* args   : gtime_t t        I   time (gpst)
*          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric delay (L1) (m)
*-----------------------------------------------------------------------------*/
extern double ionmodel(gtime_t t, const double *ion, const double *pos,
	const double *azel)
{
  const double ion_default[] = { /* 2004/1/1 */
    0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
    0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
  };
  double tt, f, psi, phi, lam, amp, per, x;
  int week;
  
  if (pos[2] < -1E3 || azel[1] <= 0) return 0.0;
  if (norm(ion, 8) <= 0.0) ion = ion_default;
  
  /* earth centered angle (semi-circle) */
  psi = 0.0137 / (azel[1] / PI + 0.11) - 0.022;
  
  /* subionospheric latitude/longitude (semi-circle) */
  phi = pos[0] / PI + psi * cos(azel[0]);
  if (phi > 0.416) phi = 0.416;
  else if (phi < -0.416) phi = -0.416;
  lam = pos[1] / PI + psi * sin(azel[0]) / cos(phi*PI);
  
  /* geomagnetic latitude (semi-circle) */
  phi += 0.064*cos((lam - 1.617)*PI);
  
  /* local time (s) */
  tt = 43200.0*lam + time2gpst(t, &week);
  tt -= floor(tt / 86400.0)*86400.0; /* 0<=tt<86400 */
  
  /* slant factor */
  f = 1.0 + 16.0*pow(0.53 - azel[1] / PI, 3.0);
  
  /* ionospheric delay */
  amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
  per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
  amp = amp < 0.0 ? 0.0 : amp;
  per = per < 72000.0 ? 72000.0 : per;
  x = 2.0*PI*(tt - 50400.0) / per;
  
  return CLIGHT * f*(fabs(x) < 1.57 ? 5E-9 + amp*(1.0 + x * x*(-0.5 + x * x / 24.0)) : 5E-9);
}

int ionocorr(gtime_t time, const nav_t *nav, int sat, const double *pos,
  const double *azel, double *ion, double *var)
{
  /* broadcast model */
  *ion = ionmodel(time, nav->ion_gps, pos, azel);
  *var = SQR(*ion*ERR_BRDCI);
  return 0;
}

/* troposphere model -----------------------------------------------------------
* compute tropospheric delay by standard atmosphere and saastamoinen model
* args   : gtime_t time     I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double humi      I   relative humidity
* return : tropospheric delay (m)
*-----------------------------------------------------------------------------*/
double tropmodel(gtime_t time, const double *pos, const double *azel,
	double humi)
{
  const double temp0 = 15.0; /* temparature at sea level */
  double hgt, pres, temp, e, z, trph, trpw;
  
  if (pos[2] < -100.0 || 1E4 < pos[2] || azel[1] <= 0) 
    return 0.0;
  
  /* standard atmosphere */
  hgt = pos[2] < 0.0 ? 0.0 : pos[2];
  
  pres = 1013.25*pow(1.0 - 2.2557E-5*hgt, 5.2568);
  temp = temp0 - 6.5E-3*hgt + 273.16;
  e = 6.108*humi*exp((17.15*temp - 4684.0) / (temp - 38.45));
  
  /* saastamoninen model */
  z = PI / 2.0 - azel[1];
  trph = 0.0022768*pres / (1.0 - 0.00266*cos(2.0*pos[0]) - 0.00028*hgt / 1E3) / cos(z);
  trpw = 0.002277*(1255.0 / temp + 0.05)*e / cos(z);
  return trph + trpw;
}

int tropcorr(gtime_t time, const nav_t *nav, const double *pos,
  const double *azel, double *trp, double *var)
{
  *trp = tropmodel(time, pos, azel, REL_HUMI);
  *var = SQR(ERR_SAAS / (sin(azel[1]) + 0.1));
  return 0;
}

/* pseudorange residuals -----------------------------------------------------*/
static int rescode(int iter, const obsd_t *obs, int n, const double *rs,
  const double *dts, const double *vare, const int *svh,
  const nav_t *nav, const double *x,
  double *v, double *H, double *var, double *azel, int *vsat,
  double *resp, int *ns)
{
  double r = 0.0;
  double dtrp = 0;
  double vmeas = 0;
  double dion = 0;
  double vion = 0;
  double vtrp = 0;
  double rr[3], pos[3], e[3], P;
  int i, j, nv = 0, mask[4] = { 0 };
  
  for (i = 0; i < 3; i++) 
    rr[i] = x[i];
  
  ecef2pos(rr, pos);
  
  for (i = *ns = 0; i < n && i < MAXSAT; i++) 
  {
    vsat[i] = 0; azel[i * 2] = azel[1 + i * 2] = resp[i] = 0.0;
    
    /* reject duplicated observation data */
    if (i < n - 1 && i < MAXSAT - 1 && obs[i].sat == obs[i + 1].sat) 
    {
      //trace(2, "duplicated observation data %s sat=%2d\n",
      //	time_str(obs[i].time, 3), obs[i].sat);
      i++;
      continue;
    }
    /* geometric distance/azimuth/elevation angle */
    if ((r = geodist(rs + i * 6, rr, e)) <= 0.0 ||
        satazel(pos, e, azel + i * 2) < 0.0) continue; //<<<<<<<<<<<<<<<<
        //satazel(pos, e, azel + i * 2) < opt->elmin) continue;
    
    /* psudorange with code bias correction */
    P = obs[i].P[0];
    double tgd = gettgd(obs[i].sat, nav);
    P -= tgd;
    
    /* excluded satellite? */
    //if (satexclude(obs[i].sat, svh[i], opt)) continue;
    if (svh[i]) //unhealsy
      continue;
    
    /* ionospheric corrections */
    ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2, &dion, &vion);
    
    /* tropospheric corrections */
    tropcorr(obs[i].time, nav, pos, azel + i * 2, &dtrp, &vtrp);
    
    /* pseudorange residual */
    double corr = (r + dion + dtrp + x[3] - CLIGHT * dts[i * 2]);
    v[nv] = P - corr;
    
    /* design matrix */
    for (j = 0;j < NX;j++) 
      H[j + nv * NX] = j < 3 ? -e[j] : (j == 3 ? 1.0 : 0.0);
      
      /* time system and receiver bias offset correction */
      mask[0] = 1;
      
      vsat[i] = 1; resp[i] = v[nv]; (*ns)++;
      
      /* error variance */
      var[nv++] = varerr(azel[1 + i * 2]) + vare[i] + vmeas + vion + vtrp;
      
      //trace(4, "sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f\n", obs[i].sat,
      //	azel[i * 2] * R2D, azel[1 + i * 2] * R2D, resp[i], sqrt(var[nv - 1]));
  }
  
  /* constraint to avoid rank-deficient */
  for (i = 0;i < 4;i++) 
  {
    if (mask[i]) 
      continue;
    v[nv] = 0.0;
    for (j = 0;j < NX;j++) H[j + nv * NX] = j == i + 3 ? 1.0 : 0.0;
    var[nv++] = 0.01;
  }
  return nv;
}

/* pseudorange residuals -----------------------------------------------------*/
// Same as rescode() but iterative
//Return -1 if result is not ready
static int rescode_iterative(int iter, const obsd_t *obs, int n, const double *rs,
  const double *dts, const double *vare, const int *svh,
  const nav_t *nav, const double *x,
  double *v, double *H, double *var, double *azel, int *vsat,
  double *resp, int *ns)
{
  double r = 0.0;
  double dtrp = 0;
  double vmeas = 0;
  double dion = 0;
  double vion = 0;
  double vtrp = 0;
  
  static double rr[3], pos[3], e[3], P;
  static int nv = 0;
  
  int i, j  = 0;
  
  static int iteration_sat_idx = 0;
  if (iteration_sat_idx == 0)
  {
    for (i = 0; i < 3; i++)
      rr[i] = x[i];
    
    ecef2pos(rr, pos);
    nv = 0;
  }
  
  i = iteration_sat_idx;
  
  //for (i = *ns = 0; i < n && i < MAXSAT; i++)
  while (1)
  {
    vsat[i] = 0; azel[i * 2] = azel[1 + i * 2] = resp[i] = 0.0;
    
    /* reject duplicated observation data */
    if (i < n - 1 && i < MAXSAT - 1 && obs[i].sat == obs[i + 1].sat)
    {
      i++;
      break;
    }
    /* geometric distance/azimuth/elevation angle */
    if ((r = geodist(rs + i * 6, rr, e)) <= 0.0 ||
        satazel(pos, e, azel + i * 2) < 0.0) 
      break; //<<<<<<<<<<<<<<<<
    
    /* psudorange with code bias correction */
    P = obs[i].P[0];
    double tgd = gettgd(obs[i].sat, nav);
    P -= tgd;
    
    /* excluded satellite? */
    if (svh[i]) //unhealsy
      break;
    
    /* ionospheric corrections */
    ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2, &dion, &vion);
    
    /* tropospheric corrections */
    tropcorr(obs[i].time, nav, pos, azel + i * 2, &dtrp, &vtrp);
    
    /* pseudorange residual */
    double corr = (r + dion + dtrp + x[3] - CLIGHT * dts[i * 2]);
    v[nv] = P - corr;
    
    /* design matrix */
    for (j = 0;j < NX;j++)
      H[j + nv * NX] = j < 3 ? -e[j] : (j == 3 ? 1.0 : 0.0);
      
      /* time system and receiver bias offset correction */
      vsat[i] = 1; resp[i] = v[nv]; (*ns)++;
      
      /* error variance */
      var[nv++] = varerr(azel[1 + i * 2]) + vare[i] + vmeas + vion + vtrp;
      break;
  }
  iteration_sat_idx++;
  
  if (iteration_sat_idx == n)
  {
    iteration_sat_idx = 0;
    /* constraint to avoid rank-deficient */
    for (i = 1; i < 4; i++)
    {
      v[nv] = 0.0;
      for (j = 0;j < NX;j++)
        H[j + nv * NX] = j == i + 3 ? 1.0 : 0.0;
        var[nv++] = 0.01;
    }
    return nv;
  }
  else
  {
    return -1;//not completed
  }
}


/* satellite positions and clocks ----------------------------------------------
* compute satellite positions, velocities and clocks
* args   : gtime_t teph     I   time to select ephemeris (gpst)
*          obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   satellite positions and velocities (ecef)
*          double *dts      O   satellite clocks
*          double *var      O   sat position and clock error variances (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : none
*-----------------------------------------------------------------------------*/
int  satposs(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
             int ephopt, double *rs, double *dts, double *var, int *svh)
{
  gtime_t time[4] = { {0} };
  double dt, pr;
  int i, j;
  uint8_t sat_ok_cnt = 0;
  
  for (i = 0; i < n && i < MAXSAT; i++)
  {
    for (j = 0;j < 6;j++) rs[j + i * 6] = 0.0;
    for (j = 0;j < 2;j++) dts[j + i * 2] = 0.0;
    var[i] = 0.0; 
    svh[i] = 0;
    
    /* search any psuedorange */
    j = 0;
    pr = obs[i].P[j];
    
    /* transmission time by satellite clock */
    time[i] = timeadd(obs[i].time, -pr / CLIGHT);
    
    /* satellite clock bias by broadcast ephemeris */
    if (!ephclk(time[i], teph, obs[i].sat, nav, &dt)) {
      //trace(2, "no broadcast clock %s sat=%2d\n", time_str(time[i], 3), obs[i].sat);
      continue;
    }
    time[i] = timeadd(time[i], -dt);
    
    /* satellite position and clock at transmission time */
    if (!satpos(time[i], teph, obs[i].sat, ephopt, nav, 
                rs + i * 6, dts + i * 2, var + i, svh + i)) 
    {
      //trace(2, "no ephemeris %s sat=%2d\n", time_str(time[i], 3), obs[i].sat);
      continue;
    }
    sat_ok_cnt++;
    /* if no precise clock available, use broadcast clock instead */
    if (dts[i * 2] == 0.0) 
    {
      if (!ephclk(time[i], teph, obs[i].sat, nav, dts + i * 2)) 
        continue;
      dts[1 + i * 2] = 0.0;
      *var = SQR(STD_BRDCCLK);
    }
  }
  
  if (sat_ok_cnt != n)
  {
    return -1;
  }
  
  return 1;
}

//Same as satposs(), but iterative
int satposs_iterative(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
  int ephopt, double *rs, double *dts, double *var, int *svh)
{
  //Index of satellite
  static uint8_t iteration_sat_idx = 0;
  
  static gtime_t time[4] = { {0} };
  double dt, pr;
  int j;
  static uint8_t sat_ok_cnt = 0;
  
  if (iteration_sat_idx == 0)
  {
    memset(time, 0, sizeof(time));
    sat_ok_cnt = 0;
  }
  
  //for (i = 0; i < n&&i < MAXSAT; i++)
  int i = iteration_sat_idx;
  while (1)
  {
    for (j = 0;j < 6;j++) 
      rs[j + i * 6] = 0.0;
    for (j = 0;j < 2;j++) dts[j + i * 2] = 0.0;
    var[i] = 0.0; svh[i] = 0;
    
    /* search any psuedorange */
    j = 0;
    pr = obs[i].P[j];
    
    /* transmission time by satellite clock */
    time[i] = timeadd(obs[i].time, -pr / CLIGHT);
    
    /* satellite clock bias by broadcast ephemeris */
    if (!ephclk(time[i], teph, obs[i].sat, nav, &dt)) {
      //trace(2, "no broadcast clock %s sat=%2d\n", time_str(time[i], 3), obs[i].sat);
      break;
    }
    time[i] = timeadd(time[i], -dt);
    
    /* satellite position and clock at transmission time */
    if (!satpos(time[i], teph, obs[i].sat, ephopt, nav, rs + i * 6, dts + i * 2, var + i,
                svh + i)) 
    {
      //trace(2, "no ephemeris %s sat=%2d\n", time_str(time[i], 3), obs[i].sat);
      break;
    }
    sat_ok_cnt++;
    /* if no precise clock available, use broadcast clock instead */
    if (dts[i * 2] == 0.0)
    {
      if (!ephclk(time[i], teph, obs[i].sat, nav, dts + i * 2)) 
        break;
      dts[1 + i * 2] = 0.0;
      *var = SQR(STD_BRDCCLK);
    }
    break;
  }
  
  //end of calculations
  iteration_sat_idx++;
  
  if ((iteration_sat_idx == n) || (iteration_sat_idx > MAXSAT))
  {
    //end of iterations
    iteration_sat_idx = 0;
    
    if (sat_ok_cnt != n)
      return -1;
    
    return 1;
  }
  else
    return 0;//not completed
}


// broadcast ephemeris to satellite clock bias ---------------------------------
double eph2clk(gtime_t time, const eph_t *eph)
{
  double t;
  int i;
  t = timediff(time, eph->toc);
  for (i = 0;i < 2;i++) 
  {
    t -= eph->f0 + eph->f1*t + eph->f2*t*t;
  }
  return eph->f0 + eph->f1*t + eph->f2*t*t;
}

/* select ephememeris --------------------------------------------------------*/
static eph_t *seleph(gtime_t time, int sat, int iode, const nav_t *nav)
{
  double t, tmax, tmin;
  int i, j = -1;
  
  tmax = MAXDTOE + 1.0;
  tmin = tmax + 1.0;
  
  for (i = 0;i < nav->n;i++) {
    if (nav->eph[i]->sat != sat) continue;
    if (iode >= 0 && nav->eph[i]->iode != iode) continue;
    if ((t = fabs(timediff(nav->eph[i]->toe, time))) > tmax) continue;
    if (iode >= 0) 
      return nav->eph[i];
    if (t <= tmin) { j = i; tmin = t; } /* toe closest to time */
  }
  if (iode >= 0 || j < 0) {
    //trace(2, "no broadcast ephemeris: %s sat=%2d iode=%3d\n", time_str(time, 0),
    //	sat, iode);
    return NULL;
  }
  return nav->eph[j];
}

/* satellite clock with broadcast ephemeris ----------------------------------*/
static int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav, double *dts)
{
  eph_t  *eph;
  
  if (!(eph = seleph(teph, sat, -1, nav))) 
    return 0;
  *dts = eph2clk(time, eph);
  
  return 1;
}

/* satellite position and clock ------------------------------------------------
* compute satellite position, velocity and clock
* args   : gtime_t time     I   time (gpst)
*          gtime_t teph     I   time to select ephemeris (gpst)
*          int    sat       I   satellite number
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   sat position and velocity (ecef)
*                               {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts      O   sat clock {bias,drift} (s|s/s)
*          double *var      O   sat position and clock error variance (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : status (1:ok,0:error)
* notes  : satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*-----------------------------------------------------------------------------*/
int satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
	const nav_t *nav, double *rs, double *dts, double *var,
	int *svh)
{
	*svh = 0;
	return ephpos(time, teph, sat, nav, -1, rs, dts, var, svh);
}

/* satellite position and clock by broadcast ephemeris -----------------------*/
static int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  int iode, double *rs, double *dts, double *var, int *svh)
{
  eph_t  *eph;
  double rst[3], dtst[1], tt = 1E-3;
  int i;
  
  *svh = -1;
  
  if (!(eph = seleph(teph, sat, iode, nav))) return 0;
  eph2pos(time, eph, rs, dts, var);
  time = timeadd(time, tt);
  eph2pos(time, eph, rst, dtst, var);
  *svh = eph->svh;
  
  /* satellite velocity and clock drift by differential approx */
  for (i = 0;i < 3;i++) 
    rs[i + 3] = (rst[i] - rs[i]) / tt;
  
  dts[1] = (dtst[0] - dts[0]) / tt;
  
  return 1;
}

/* variance by ura ephemeris (ref [1] 20.3.3.3.1.1) --------------------------*/
static double var_uraeph(int ura)
{
  const double ura_value[] = {
    2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
    3072.0,6144.0
  };
  return ura < 0 || 15 < ura ? SQR(6144.0) : SQR(ura_value[ura]);
}

/* broadcast ephemeris to satellite position and clock bias --------------------
* compute satellite position and clock bias with broadcast ephemeris (gps,
* galileo, qzss)
* args   : gtime_t time     I   time (gpst)
*          eph_t *eph       I   broadcast ephemeris
*          double *rs       O   satellite position (ecef) {x,y,z} (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [1],[7],[8]
*          satellite clock includes relativity correction without code bias
*          (tgd or bgd)
*-----------------------------------------------------------------------------*/
void eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts,
             double *var)
{
  double tk, M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;
  int n;
  
  if (eph->A <= 0.0) {
    rs[0] = rs[1] = rs[2] = *dts = *var = 0.0;
    return;
  }
  tk = timediff(time, eph->toe);
  
  mu = MU_GPS; 
  omge = OMGE;
  
  M = eph->M0 + (sqrt(mu / (eph->A*eph->A*eph->A)) + eph->deln)*tk;
  
  for (n = 0, E = M, Ek = 0.0;fabs(E - Ek) > RTOL_KEPLER&&n < MAX_ITER_KEPLER;n++) {
    Ek = E; E -= (E - eph->e*sin(E) - M) / (1.0 - eph->e*cos(E));
  }
  if (n >= MAX_ITER_KEPLER) {
    //trace(2, "kepler iteration overflow sat=%2d\n", eph->sat);
    return;
  }
  sinE = sin(E); cosE = cos(E);
  
  //trace(4, "kepler: sat=%2d e=%8.5f n=%2d del=%10.3e\n", eph->sat, eph->e, n, E - Ek);
  
  u = atan2(sqrt(1.0 - eph->e*eph->e)*sinE, cosE - eph->e) + eph->omg;
  r = eph->A*(1.0 - eph->e*cosE);
  i = eph->i0 + eph->idot*tk;
  sin2u = sin(2.0*u); cos2u = cos(2.0*u);
  u += eph->cus*sin2u + eph->cuc*cos2u;
  r += eph->crs*sin2u + eph->crc*cos2u;
  i += eph->cis*sin2u + eph->cic*cos2u;
  x = r * cos(u); y = r * sin(u); cosi = cos(i);
  
  O = eph->OMG0 + (eph->OMGd - omge)*tk - omge * eph->toes;
  sinO = sin(O); cosO = cos(O);
  rs[0] = x * cosO - y * cosi*sinO;
  rs[1] = x * sinO + y * cosi*cosO;
  rs[2] = y * sin(i);
  
  tk = timediff(time, eph->toc);
  *dts = eph->f0 + eph->f1*tk + eph->f2*tk*tk;
  
  /* relativity correction */
  *dts -= 2.0*sqrt(mu*eph->A)*eph->e*sinE / SQR(CLIGHT);
  
  /* position and clock error variance */
  *var = var_uraeph(eph->sva);
}

/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void ecef2pos(const double *r, double *pos)
{
  double e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = dot(r, r, 2), z, zk, v = RE_WGS84, sinp;
  
  for (z = r[2], zk = 0.0;fabs(z - zk) >= 1E-4;) {
    zk = z;
    sinp = z / sqrt(r2 + z * z);
    v = RE_WGS84 / sqrt(1.0 - e2 * sinp*sinp);
    z = r[2] + v * e2*sinp;
  }
  pos[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (r[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
  pos[1] = r2 > 1E-12 ? atan2(r[1], r[0]) : 0.0;
  pos[2] = sqrt(r2 + z * z) - v;
}

/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : double *rs       I   satellilte position (ecef at transmission) (m)
*          double *rr       I   receiver position (ecef at reception) (m)
*          double *e        O   line-of-sight vector (ecef)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes sagnac effect correction
*-----------------------------------------------------------------------------*/
double geodist(const double *rs, const double *rr, double *e)
{
  double r;
  int i;
  
  if (norm(rs, 3) < RE_WGS84) return -1.0;
  for (i = 0;i < 3;i++) e[i] = rs[i] - rr[i];
  r = norm(e, 3);
  for (i = 0;i < 3;i++) e[i] /= r;
  return r + OMGE * (rs[0] * rr[1] - rs[1] * rr[0]) / CLIGHT;
}

/* satellite azimuth/elevation angle -------------------------------------------
* compute satellite azimuth/elevation angle
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *e        I   receiver-to-satellilte unit vevtor (ecef)
*          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
*                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
* return : elevation angle (rad)
*-----------------------------------------------------------------------------*/
double satazel(const double *pos, const double *e, double *azel)
{
  double az = 0.0, el = PI / 2.0, enu[3];
  
  if (pos[2] > -RE_WGS84) {
    ecef2enu(pos, e, enu);
    az = dot(enu, enu, 2) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]);
    if (az < 0.0) az += 2 * PI;
    el = asin(enu[2]);
  }
  if (azel) { azel[0] = az; azel[1] = el; }
  return el;
}

/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
void xyz2enu(const double *pos, double *E)
{
  double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
  
  E[0] = -sinl;      E[3] = cosl;       E[6] = 0.0;
  E[1] = -sinp * cosl; E[4] = -sinp * sinl; E[7] = cosp;
  E[2] = cosp * cosl;  E[5] = cosp * sinl;  E[8] = sinp;
}

/* multiply matrix (wrapper of blas dgemm) -------------------------------------
* multiply matrix by matrix (C=alpha*A*B+beta*C)
* args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
*          int    n,k,m     I  size of (transposed) matrix A,B
*          double alpha     I  alpha
*          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
*          double beta      I  beta
*          double *C        IO matrix C (n x k)
* return : none
*-----------------------------------------------------------------------------*/
void matmul(const char *tr, int n, int k, int m, double alpha,
            const double *A, const double *B, double beta, double *C)
{
  double d;
  int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);
  
  for (i = 0;i < n;i++)
    for (j = 0;j < k;j++)
    {
      d = 0.0;
      switch (f)
      {
      case 1: for (x = 0;x < m;x++) d += A[i + x * n] * B[x + j * m]; break;
      case 2: for (x = 0;x < m;x++) d += A[i + x * n] * B[j + x * k]; break;
      case 3: for (x = 0;x < m;x++) d += A[x + i * m] * B[x + j * m]; break;
      case 4: for (x = 0;x < m;x++) d += A[x + i * m] * B[j + x * k]; break;
      }
      if (beta == 0.0)
        C[i + j * n] = alpha * d;
      else
        C[i + j * n] = alpha * d + beta * C[i + j * n];
    }
}

/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
void ecef2enu(const double *pos, const double *r, double *e)
{
  double E[9];
  
  xyz2enu(pos, E);
  matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}

/* LU decomposition ----------------------------------------------------------*/
static int ludcmp(double *A, int n, int *indx, double *d)
{
  double big, s, tmp;
  double *vv;
  int i, imax = 0, j, k;
  
  vv = mat(n, 1);
  if (!vv)
  {
    free(vv);
    return 2;
  }
  
  *d = 1.0;
  for (i = 0;i < n;i++) {
    big = 0.0; for (j = 0;j < n;j++) if ((tmp = fabs(A[i + j * n])) > big) big = tmp;
    if (big > 0.0) vv[i] = 1.0 / big; else { free(vv); return 1; }
  }
  for (j = 0;j < n;j++) {
    for (i = 0;i < j;i++) {
      s = A[i + j * n]; for (k = 0;k < i;k++) s -= A[i + k * n] * A[k + j * n]; A[i + j * n] = s;
    }
    big = 0.0;
    for (i = j;i < n;i++) {
      s = A[i + j * n]; for (k = 0;k < j;k++) s -= A[i + k * n] * A[k + j * n]; A[i + j * n] = s;
      if ((tmp = vv[i] * fabs(s)) >= big) { big = tmp; imax = i; }
    }
    if (j != imax) {
      for (k = 0;k < n;k++) {
        tmp = A[imax + k * n]; A[imax + k * n] = A[j + k * n]; A[j + k * n] = tmp;
      }
      *d = -(*d); vv[imax] = vv[j];
    }
    indx[j] = imax;
    if (A[j + j * n] == 0.0) { free(vv); return 1; }
    if (j != n - 1) {
      tmp = 1.0 / A[j + j * n]; for (i = j + 1;i < n;i++) A[i + j * n] *= tmp;
    }
  }
  free(vv);
  return 0;
}

/* LU back-substitution ------------------------------------------------------*/
static void lubksb(const double *A, int n, const int *indx, double *b)
{
  double s;
  int i, ii = -1, ip, j;
  
  for (i = 0;i < n;i++) {
    ip = indx[i]; s = b[ip]; b[ip] = b[i];
    if (ii >= 0) for (j = ii;j < i;j++) s -= A[i + j * n] * b[j]; else if (s) ii = i;
    b[i] = s;
  }
  for (i = n - 1;i >= 0;i--) {
    s = b[i]; for (j = i + 1;j < n;j++) s -= A[i + j * n] * b[j]; b[i] = s / A[i + i * n];
  }
}

/* inverse of matrix -----------------------------------------------------------
* inverse of matrix (A=A^-1)
* args   : double *A        IO  matrix (n x n)
*          int    n         I   size of matrix A
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
extern int matinv(double *A, int n)
{
  double d, *B;
  int i, j, *indx;

  indx = imat(n, 1); 
  B = mat(n, n);
  if (!indx || !B)
    return 1;
  
  matcpy(B, A, n, n);
  if (ludcmp(B, n, indx, &d)) 
  { 
    free(indx); 
    free(B); 
    return 2; 
  }
  
  for (j = 0;j < n;j++) {
    for (i = 0;i < n;i++) A[i + j * n] = 0.0; A[j + j * n] = 1.0;
    lubksb(B, n, indx, A + j * n);
  }
  free(indx); free(B);
  return 0;
}



/* least square estimation -----------------------------------------------------
* least square estimation by solving normal equation (x=(A*A')^-1*A*y)
* args   : double *A        I   transpose of (weighted) design matrix (n x m)
*          double *y        I   (weighted) measurements (m x 1)
*          int    n,m       I   number of parameters and measurements (n<=m)
*          double *x        O   estmated parameters (n x 1)
*          double *Q        O   esimated parameters covariance matrix (n x n)
* return : status (0:ok,0>:error)
* notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
*          matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
int lsq(const double *A, const double *y, int n, int m, double *x, double *Q)
{
  double *Ay = mat(n, 1);
  if (!Ay)
    return 1;
  if (m < n) 
    return 2;
  matmul("NN", n, 1, m, 1.0, A, y, 0.0, Ay);
  matmul("NT", n, n, m, 1.0, A, A, 0.0, Q);
  if (matinv(Q, n))
  {
    //error
    free(Ay);
    return 3;
  }
  matmul("NN", n, 1, n, 1.0, Q, Ay, 0.0, x);
  free(Ay);
  return 0;
}