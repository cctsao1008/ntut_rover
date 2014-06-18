#ifndef _CORE_H_
#define _CORE_H_
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/time.h>                // for gettimeofday()
#include <math.h>
#include "gps.h"
#include "imu.h"
#include "motor.h"
#include "socket.h"
#include "xively.h"

#ifndef M_PI
#define M_PI 3.14
#endif

#define XI_API_KEY       "yy2kkFRYdtEygqEJixIMQWIwXlN97Cj1wwwpC9oNKhAgi8X1"
#define XI_FEED_ID       1244498277
#define XI_STREAM_ID_1   "count"

#define RPI_UID          1000
#define RPI_GID          1000

#define FILTER_PRE_INIT  1
#define USE_CURSES       1

#define TRUE             1
#define FALSE            0

#define GPSD_SAMPLE_RATE 50000
#define KM2M             1000
#define MA_N             5
#define YELLOWONBLUE     1
#define BLACKONWHITE     2
#define WHITEONBLACK     3
#define ATTRIBS          WA_BOLD 
#define COLORS           BLACKONWHITE

#define DEVICE_NUM       "11208"
#define SENSOR_TEMP      "18553"
#define API_KEY          "5c14ba4565cb215fc9f813454fee28d5" /**< API-KEY */
#define DATA_MIN         1
#define HOST_NAME        "api.yeelink.net"
#define PORT             80

#define HANDLE           void*
#define inv_norm(x, y)   fast_inv_sqrt((x)*(x) + (y)*(y))
#define norm(x, y)       (1 / fast_inv_sqrt((x)*(x) + (y)*(y)))
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
    double  mean_t; /* Average value, Sample mean */
    double  sd; /* Standard Deviation */
    double  var; /* Variance, Sample variance */
    double  max; /* Maximum */
    double  min; /* Minimum */
    double  sum; /* Summation  */
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
double calc_distance(double lat1, double lon1, double lat2, double lon2, char unit);

#endif
