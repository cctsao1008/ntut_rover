/*********************************************************************
 *
 *   File : 
 *       core.c
 *
 *   Date : 
 *       11/05/2014
 *
 *   Contributors : 
 *       TSAO, CHIA-CHENG <chiacheng.tsao@gmail.com> , From NTUT
 *
 *********************************************************************/
#include <curses.h>
#include <signal.h>
#include <pthread.h>
#include "core.h"
#include "motor.h"

typedef struct _kf
{
    float k;
    float p;
    float q;
    float r;
    float x;
}kf_t;

/* Kalman Filter Settings */
float lat_Q = 0.022; //process noise cvariance
float lat_R = 0.617; //measurement noise covariance

float lon_Q = 0.022;
float lon_R = 0.617;

kf_t* lat_kf = NULL;
kf_t* lon_kf = NULL;

kf_t* kf_create(float q, float r, float x)
{
    kf_t* kf = malloc(sizeof(kf_t));

    if(NULL != kf)
    {
        kf->k = 0;
        //kf->p = sqrtf(q * q + r * r); // estimation error covariance
        kf->p = 1;
        kf->q = q; // process noise covariance
        kf->r = r; // measurement noise covariance
        kf->x = x;
    }

    return kf;
}

float kf_update(kf_t* kf, float z)
{
    if(NULL == kf)
        return 0;

    /* Kalman Prediction */
    /* Update error covariance ahead*/
    kf->p = kf->p + kf->q;

    /* Kalman Correction */
    /* Update kalman gain */
    kf->k = kf->p / (kf->p + kf->r);

    /* Update estimate */
    kf->x = kf->x + kf->k * (z - kf->x);

    /* Update error covariance */
    kf->p = (1 - kf->k) * kf->p;

    return (kf->x); 
}

typedef struct _waypoint
{
    char* name;
    float lat;
    float lon;
}waypoint;

//typedef float (* get_angle)(v2d);
//typedef float (* get_length)(v2d);

typedef struct {
    float x;
    float y;
    //get_angle angle;
    //get_length length;
} vect;

#if 0
v2d* v2d_create(float x, float y)
{
    v2d *vect = (v2d *)malloc(sizeof(v2d));

    vect.x = x;
    vect.y = y;
    
    if(NULL != vect)
        return vect;
}
#endif

float fast_inv_sqrt(float x) {
  float xhalf = 0.5f * x;
  int i = *(int*)&x;         // evil floating point bit level hacking
  i = 0x5f3759df - (i >> 1);  // what the fuck?
  x = *(float*)&i;
  x = x*(1.5f-(xhalf*x*x));
  return x;
}

#define norm(x, y) fast_inv_sqrt((x)*(x) + (y)*(y))

/*

  This routine calculates the distance between two points (given the
  latitude/longitude of those points). It is being used to calculate
  the distance between two locations using GeoDataSource(TM) products.

  Definitions:
    South latitudes are negative, east longitudes are positive

  Passed to function:
    lat1, lon1 = Latitude and Longitude of point 1 (in decimal degrees)
    lat2, lon2 = Latitude and Longitude of point 2 (in decimal degrees)
    unit = the unit you desire for results
           where: 'M' is statute miles
                  'K' is kilometers (default)
                  'N' is nautical miles
  Worldwide cities and other features databases with latitude longitude
  are available at http://www.geodatasource.com

  For enquiries, please contact sales@geodatasource.com

  Official Web site: http://www.geodatasource.com

           GeoDataSource.com (C) All Rights Reserved 2014

*/

double deg2rad(double deg) {
  return (deg * M_PI / 180);
}

double rad2deg(double rad) {
  return (rad * 180 / M_PI);
}

// dist = arccos(sin(lat1) ¡P sin(lat2) + cos(lat1) ¡P cos(lat2) ¡P cos(lon1 - lon2)) ¡P R
// R=6371 km
double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  double theta, dist;
  theta = lon1 - lon2;
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515; // geodetic longitude and latitude
  switch(unit) {
    case 'M':
      break;
    case 'K':
      dist = dist * 1.609344;
      break;
    case 'N':
      dist = dist * 0.8684;
      break;
  }
  return (dist);
}

#define MA_N 20
waypoint wp[2] = 
{
    [0] = {
        .name = "NTUT",
        .lat =  25.042583, // N
        .lon = 121.537674, // E
    },
};

#define max(a,b) (a>b?a:b)
#define min(a,b) (a<b?a:b)

typedef struct _fifo
{
    float*  data;
    uint8_t size;
    uint8_t index;
}fifo_t;

#define HANDLE void*

fifo_t* fifo_create(float data, uint8_t size)
{
    fifo_t* f = NULL;
    float* buf = NULL;

    f = calloc(1, sizeof(fifo_t));

    if(NULL != f)
    {
        buf = malloc(size * sizeof(float));

        if(NULL != buf)
        {
            f->data = buf;
            f->size = size;
            f->index = 0;

            memset(buf, data, size * sizeof(float));
        }
        else
        {
            free(f);
            fprintf(stderr, "buffer = null!\r\n\r\n");
            exit(EXIT_SUCCESS);
        }
    }

    return f;
}

void fifo_add(fifo_t* fifo, float data)
{
    fifo->data[fifo->index] = data;

    if(((++(fifo->index)) % (fifo->size)) == 0)
        fifo->index = 0;
}

typedef struct _filter
{
    fifo_t* fifo;
    float  mean; /* Average value, Sample mean */
    float  sd; /* Standard Deviation */
    float  var; /* Variance, Sample variance */
    float  max; /* Maximum */
    float  min; /* Minimum */
    float  sum;
}filter_t;

void filter_update(filter_t* f, float d)
{
    float sum = 0;
    uint8_t i = 0;

    f->fifo->data[f->fifo->index] = d;

    if(((++(f->fifo->index)) % (f->fifo->size)) == 0)
        f->fifo->index = 0;

    /* Calculate average */
    sum = 0;
    for(i = 0 ; i < (f->fifo->size) ; i++)
    {
        sum = sum + f->fifo->data[i];
    }

    f->sum = sum;
    f->mean = sum / (f->fifo->size);

    /*
           Calculate standard deviation

           Deviation = f->fifo->data[i] - f->mean
           
       */
    sum = 0;
    for(i = 0 ; i < (f->fifo->size) ; i++)
    {
        sum = sum + (f->fifo->data[i] - f->mean)*(f->fifo->data[i] - f->mean);
    }

    f->var = sum / f->fifo->size;
    f->sd = sqrtf(f->var);

    /* Find the maximum value of buffer */
    f->max = f->fifo->data[0];
    for(i = 0 ; i < (f->fifo->size) ; i++)
    {
        f->max = max(f->max, f->fifo->data[i]);
    }

    /* Find the  minimum value of buffer */
    f->min = f->fifo->data[0];
    for(i = 0 ; i < (f->fifo->size) ; i++)
    {
        f->min = min(f->min, f->fifo->data[i]);
    }
}

filter_t* filter_create(fifo_t* fio)
{
    filter_t* f;

    f = calloc(1, sizeof(filter_t));

    if(NULL != f)
    {
        if(NULL != fio)
            f->fifo = fio;
        else
        {
            fprintf(stderr, "fifo = null!\r\n");
            exit(EXIT_SUCCESS);
        }
    }

    return f;
}

fifo_t* lat_fifo;
fifo_t* lon_fifo;
filter_t* lat_filter;
filter_t* lon_filter;
float _distance = 0, _direction = 0;

struct gps_data_t _gps_data;

void die(int sig)
{
    if (!isendwin())
    {
        /* Move the cursor to the bottom left corner. */
        (void)mvcur(0, COLS - 1, LINES - 1, 0);

        /* Put input attributes back the way they were. */
        (void)echo();

        /* Done with curses. */
        (void)endwin();
    }

    /* We're done talking to gpsd. */
    (void)gps_close(&_gps_data);

    /* Stop motor */
    motor_update(M_BRK, 0, 0);

    printf("bye bye\r\n");
    exit(EXIT_SUCCESS);
}

char dir = 'B';
int pwm = 0;

void commander(void)
{
    int c = ' ';

    c = getch();
        
    if (c == KEY_DOWN)
    {
        pwm -= 10;

        if(pwm < -250)
            pwm = -250;
        else if(pwm < 0)
            motor_update(M_BWD, pwm, pwm);
        else
            motor_update(M_FWD, pwm, pwm);

        dir = 'D';
    }

    if (c == KEY_UP)
    {
        pwm += 10;

        if(pwm > 250)
            pwm = 250;
        else if(pwm > 0)
            motor_update(M_FWD, pwm, pwm);
        else
            motor_update(M_BWD, pwm, pwm);

        dir = 'U';
    }

    if (c == KEY_LEFT)
    {
        motor_update(M_TNL, pwm, pwm);
        dir = 'L';
    }

    if (c == KEY_RIGHT)
    {
        motor_update(M_TNR, pwm, pwm);
        dir = 'R';
    }

    if (c == ' ')
    {
        pwm = 0;
        motor_update(M_BRK, pwm, pwm );
        dir = 'B';
    }

}

void thread_cmd(void)
{
    for(;;)
    {
        commander();
        usleep(50000);
    }
}

#define KM2M 1000
#define USE_CURSES

#define YELLOWONBLUE 1
#define BLACKONWHITE 2
#define WHITEONBLACK 3

#define ATTRIBS  WA_BOLD 
//#define COLORS YELLOWONBLUE
#define COLORS BLACKONWHITE

bool init_colors()
{
    if(has_colors())
    {
        start_color();
        init_pair(YELLOWONBLUE ,COLOR_YELLOW, COLOR_BLUE );
        init_pair(BLACKONWHITE ,COLOR_BLACK ,COLOR_WHITE );
        init_pair(WHITEONBLACK ,COLOR_WHITE ,COLOR_BLACK );
        return(true);
    }
    else
        return(false);
}
    
bool set_colors(int colorscheme)
{
    if(has_colors())
    {
        attrset(colorscheme);
        return(true);
    }
    else
        return(false);
}

void clrscr(void)
{
    int y, x, maxy, maxx;
    getmaxyx(stdscr, maxy, maxx);
    for(y=0; y < maxy; y++)
        for(x=0; x < maxx; x++)
            mvaddch(y, x, ' ');
}

void disp_init(void)
{
    #ifdef USE_CURSES
    #if 0
    int row,col;    /* to store the number of rows and *
                     * the number of colums of the screen */
    #endif

    initscr(); /* start the curses mode */
    keypad(stdscr, TRUE);     /* We get key input    from the main window */
    noecho(); /* Don't echo() while we do getch */
    cbreak();
    timeout(0);
    //init_colors();
    //attrset(COLOR_PAIR(COLORS) | ATTRIBS);
    //clrscr();
    box(stdscr, ACS_VLINE, ACS_HLINE); /*draw a box*/
    curs_set(0); /* Makes the cursor invisible */
    #if 0
    getmaxyx(stdscr,row,col); /* get the number of rows and columns */
    #endif
    //start_color(); /* Be sure not to forget this, it will enable colors */
    //init_pair(1, COLOR_RED, COLOR_CYAN); /* You can make as much color pairs as you want, be sure to change the ID number */
    //init_pair(2, COLOR_YELLOW, COLOR_GREEN);
    //attron(A_UNDERLINE | COLOR_PAIR(1)); /* This turns on the underlined text and color pair 1 */
    //printw("This is underlined red text with cyan background\n");
    //attroff(A_UNDERLINE | COLOR_PAIR(1));
    //attron(A_BOLD | COLOR_PAIR(2));
    //printw("This is bold yellow text with green background");
    //attroff(A_BOLD | COLOR_PAIR(2));
    //move(LINES/2, COLS/2); /*move the cursor to the center*/
    refresh();
    #endif
}

void disp_update(int row, int col)
{
    #ifdef USE_CURSES
    int row_disp = row;
    #define MSG(a, b, ...) mvprintw(a, b, __VA_ARGS__)
    #else
    #define MSG(a, b, ...) printf(__VA_ARGS__); printf("\r\n")
    #endif

    #ifdef USE_CURSES
    erase();
    //attrset(COLOR_PAIR(COLORS) | ATTRIBS);
    //clrscr();
    box(stdscr, ACS_VLINE, ACS_HLINE); /*draw a box*/
    #endif

    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "GPS(Global Positioning System) informations that got from    ");
    MSG(row_disp++, col, "GPSd(a GPS service daemon)                                   ");
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "gps_data.status= %d", _gps_data.status);
    MSG(row_disp++, col, "gps_data.fix.track= %f", _gps_data.fix.track);
    MSG(row_disp++, col, "gps_data.fix.speed= %f", _gps_data.fix.speed);
    MSG(row_disp++, col, "gps_data.fix.latitude = %f", _gps_data.fix.latitude);
    MSG(row_disp++, col, "gps_data.fix.longitude = %f", _gps_data.fix.longitude);
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "Filter Informations                                          ");
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "#Latitude :                                                  ");
    MSG(row_disp++, col, "lat_filter->mean = %f", lat_filter->mean);
    MSG(row_disp++, col, "lat_filter->sd = %f", lat_filter->sd);
    MSG(row_disp++, col, "lat_filter->max = %f", lat_filter->max);
    MSG(row_disp++, col, "lat_filter->min = %f", lat_filter->min);
    MSG(row_disp++, col, "Kalman lat_est = %f", lat_kf->x);
    MSG(row_disp++, col, "#Longitude :                                                 ");
    MSG(row_disp++, col, "lon_filter->mean = %f", lon_filter->mean);
    MSG(row_disp++, col, "lon_filter->sd = %f", lon_filter->sd);
    MSG(row_disp++, col, "lon_filter->max = %f", lon_filter->max);
    MSG(row_disp++, col, "lon_filter->min = %f", lon_filter->min);
    MSG(row_disp++, col, "kalman lon_est = %f", lon_kf->x);
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "local way poipn to the way point in the sports field of NTUT ");
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "distance = %f m", _distance*KM2M);
    MSG(row_disp++, col, "direction = %f deg", _direction*180/M_PI);
    MSG(row_disp++, col, "motor ctrl = dir(%c), pwm(%03d)", dir, pwm);
    MSG(row_disp++, col, "-------------------------------------------------------------");

    #ifdef USE_CURSES
    refresh();
    #endif
}

#define GPSD_SAMPLE_RATE 80000
int main(int argc, char *argv[])
{
    int row = 1, col = 3, i = 0;
    pthread_t thread;

    (void)signal(SIGINT, die);

    /* collect gps data */
    if (gps_start() != 0)
        exit(EXIT_FAILURE);

    /* Latitude */
    lat_fifo = fifo_create(0, MA_N);

    if(NULL == lat_fifo)
    {
        exit(EXIT_SUCCESS);
    }

    lat_filter = filter_create(lat_fifo);

    if(NULL == lat_filter)
    {
        exit(EXIT_SUCCESS);
    }

    /* Longitude */
    lon_fifo = fifo_create(0, MA_N);

    if(NULL == lon_fifo)
    {
        exit(EXIT_SUCCESS);
    }

    lon_filter = filter_create(lon_fifo);

    if(NULL == lon_filter)
    {
        exit(EXIT_SUCCESS);
    }

    lat_kf = kf_create(lat_Q, lat_R, 0);

    if(NULL == lat_kf)
    {
        exit(EXIT_SUCCESS);
    }

    lon_kf = kf_create(lon_Q, lon_R, 0);

    if(NULL == lon_kf)
    {
        exit(EXIT_SUCCESS);
    }

    motor_initialize();

    disp_init();

    if(0 != pthread_create(&thread,NULL,(void *) thread_cmd,NULL))
    {
        exit(EXIT_SUCCESS);
    }

    /* Pre-Initialization */
    #if 1
    for(;;)
    {
        if(get_gps_data(&_gps_data) != 0)
            break;

        if (_gps_data.fix.mode > MODE_NO_FIX)
        {   
            for( i = 0 ; i < MA_N ; i++)
            {
                //filter_update(lat_filter, _gps_data.fix.latitude);
                //filter_update(lon_filter, _gps_data.fix.longitude);
                lat_filter->fifo->data[i] = _gps_data.fix.latitude;
                lon_filter->fifo->data[i] = _gps_data.fix.longitude;

                //kf_update(lat_kf, _gps_data.fix.latitude);
                //kf_update(lon_kf, _gps_data.fix.longitude);
            }

            lat_kf->x = _gps_data.fix.latitude;
            lon_kf->x = _gps_data.fix.longitude;

            disp_update(row, col);

            break;
        }

        //commander();
        disp_update(row, col);
         
        usleep(GPSD_SAMPLE_RATE);
    }
    #endif

    /* Main Loop */
    for(;;)
    {
        if(get_gps_data(&_gps_data) != 0)
            break;

        if (_gps_data.fix.mode > MODE_NO_FIX)
        {   

            filter_update(lat_filter, _gps_data.fix.latitude);
            filter_update(lon_filter, _gps_data.fix.longitude);

            kf_update(lat_kf, _gps_data.fix.latitude);
            kf_update(lon_kf, _gps_data.fix.longitude);

            //_distance = distance(_gps_data.fix.latitude, _gps_data.fix.longitude, wp[0].lat, wp[0].lon, 'K');
            _distance = distance(lat_kf->x, lon_kf->x, wp[0].lat, wp[0].lon, 'K');

            /* This is a simple approach due to a small distance */
            //_direction = atan2((wp[0].lat - _gps_data.fix.latitude), (wp[0].lon - _gps_data.fix.longitude));
            _direction = atan2((wp[0].lat - lat_kf->x), (wp[0].lon - lon_kf->x));

            //printf("%f, %f, %f, %f, %f, %f\r\n", _gps_data.fix.latitude, lat_filter->mean, lat_est, _gps_data.fix.longitude, lon_filter->mean, lon_est);
        }

        //commander();
        disp_update(row, col);

        usleep(GPSD_SAMPLE_RATE);

    }

    exit(EXIT_SUCCESS);
}

