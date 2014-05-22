#ifndef _CORE_H_
#define _CORE_H_
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <math.h>
#include "gps.h"

#define FILTER_PRE_INIT  1
#define USE_CURSES       1

#define TRUE             1
#define FALSE            0

#define GPSD_SAMPLE_RATE 50000
#define KM2M             1000
#define MA_N             20
#define YELLOWONBLUE     1
#define BLACKONWHITE     2
#define WHITEONBLACK     3
#define ATTRIBS          WA_BOLD 
#define COLORS           BLACKONWHITE

#define HANDLE           void*
#define norm(x, y)       fast_inv_sqrt((x)*(x) + (y)*(y))
#define max(a, b)        ((a)>(b)? (a):(b))
#define min(a, b)        ((a)<(b)? (a):(b))

typedef struct _kf
{
    double k;
    double p;
    double q; // process noise with known covariance
    double r; // measurement noise with known covariance
    double x;
}kf_t;

typedef struct _fifo
{
    double*  data;
    uint8_t size;
    uint8_t index;
}fifo_t;

typedef struct _filter
{
    fifo_t* fifo;
    double  mean; /* Average value, Sample mean */
    double  sd; /* Standard Deviation */
    double  var; /* Variance, Sample variance */
    double  max; /* Maximum */
    double  min; /* Minimum */
    double  sum;
}filter_t;

typedef struct _waypoint
{
    char* name;
    double lat;
    double lon;
}waypoint;

typedef struct _vect{
    double x;
    double y;
} vect;

double fast_inv_sqrt(double x);
double deg2rad(double deg);
double rad2deg(double rad);
kf_t* kf_create(double q, double r, double x);
double kf_update(kf_t* kf, double z);
fifo_t* fifo_create(double data, uint8_t size);
void filter_update(filter_t* f, double d);
void fifo_add(fifo_t* fifo, double data);
double distance(double lat1, double lon1, double lat2, double lon2, char unit);

#endif