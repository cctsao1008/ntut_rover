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

//#define USE_DATA_DUMP

/* Kalman Filter Settings */
kf_t* lat_kf = NULL;
kf_t* lon_kf = NULL;
fifo_t* lat_fifo;
fifo_t* lon_fifo;
filter_t* lat_filter;
filter_t* lon_filter;
double _distance = 0, _direction = 0;

char dir = 'B';
int pwm = 0;

waypoint wp[2] = 
{
    [0] = {
        .name = "NTUT",
        .lat =  25.042583, // N
        .lon = 121.537674, // E
    },
};

struct gps_data_t gps_data_curr;

kf_t* kf_create(double q, double r, double x)
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

double kf_update(kf_t* kf, double z)
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

fifo_t* fifo_create(double data, uint8_t size)
{
    fifo_t* f = NULL;
    double* buf = NULL;

    f = calloc(1, sizeof(fifo_t));

    if(NULL != f)
    {
        buf = malloc(size * sizeof(double));

        if(NULL != buf)
        {
            f->data = buf;
            f->size = size;
            f->index = 0;

            memset(buf, data, size * sizeof(double));
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

void fifo_add(fifo_t* fifo, double data)
{
    fifo->data[fifo->index] = data;

    if(((++(fifo->index)) % (fifo->size)) == 0)
        fifo->index = 0;
}

double filter_cov(filter_t* f1, filter_t* f2)
{
    double cov = 0;
    uint8_t i;

    if(f1->fifo->size != f2->fifo->size)
        return 0;

    for(i = 0 ; i < (f1->fifo->size) ; i++)
    {
        cov += (f1->fifo->data[i]- f1->mean) * (f2->fifo->data[i]- f2->mean);
    }

    return (cov / f1->fifo->size);
}

void filter_update(filter_t* f, double d)
{
    double sum_1 = 0, sum_2 = 0;
    uint8_t i = 0;

    f->fifo->data[f->fifo->index] = d;

    if(((++(f->fifo->index)) % (f->fifo->size)) == 0)
        f->fifo->index = 0;

    f->max = f->fifo->data[0];
    f->min = f->fifo->data[0];

    for(i = 0 ; i < (f->fifo->size) ; i++)
    {
        sum_1 += f->fifo->data[i];
        sum_2 += powf((f->fifo->data[i] - f->mean), 2);

        /* Find the maximum and minimum value of buffer */
        f->max = max(f->max, f->fifo->data[i]);
        f->min = min(f->min, f->fifo->data[i]);
    }

    f->sum = sum_1;

    /* Calculate moving average */
    f->mean = sum_1 / (f->fifo->size);

    f->var = sum_2 / f->fifo->size;

    /* Calculate standard deviation */
    f->sd = sqrtf(f->var);
    
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

double fast_inv_sqrt(double x) {
  double xhalf = 0.5f * x;
  int i = *(int*)&x;         // evil doubleing point bit level hacking
  i = 0x5f3759df - (i >> 1);  // what the fuck?
  x = *(double*)&i;
  x = x*(1.5f-(xhalf*x*x));
  return x;
}

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

/*

    dist = arccos(sin(lat1) ¡P sin(lat2) + cos(lat1) ¡P cos(lat2) ¡P cos(lon1 - lon2)) ¡P R
    R=6371 km

*/

double distance(double lat1, double lon1, double lat2, double lon2, char unit)
{
    double theta, dist;

    theta = lon1 - lon2;
    dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515; // geodetic longitude and latitude

    switch(unit)
    {
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

void signal_handler(int sig)
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
    (void)gps_close(&gps_data_curr);

    /* Stop motor */
    motor_update(M_BRK, 0, 0);

    printf("bye bye\r\n");
    exit(EXIT_SUCCESS);
}

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
    nonl();
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
    #define MSG(a, b, ...) do {printf(__VA_ARGS__); printf("\r\n")}while(0)
    #endif

    #ifdef USE_CURSES
    //erase();
    clear();
    //attrset(COLOR_PAIR(COLORS) | ATTRIBS);
    //clrscr();
    //box(stdscr, ACS_VLINE, ACS_HLINE); /*draw a box*/
    #endif

    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "GPS(Global Positioning System) informations that got from    ");
    MSG(row_disp++, col, "GPSd(a GPS service daemon)                                   ");
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "gps_data.status= %d", gps_data_curr.status);
    MSG(row_disp++, col, "gps_data.fix.track= %f", gps_data_curr.fix.track);
    MSG(row_disp++, col, "gps_data.fix.speed= %f", gps_data_curr.fix.speed);
    MSG(row_disp++, col, "gps_data.fix.latitude = %f", gps_data_curr.fix.latitude);
    MSG(row_disp++, col, "gps_data.fix.longitude = %f", gps_data_curr.fix.longitude);
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
        usleep(10000);
    }
}

void navigation(void)
{

}

void thread_nav(void)
{
    for(;;)
    {
        navigation();
        usleep(50000);
    }
}

void pre_initialize(void)
{
    int i = 0, rc = FALSE;
    pthread_t cmd, nav;

    (void)signal(SIGINT, signal_handler);

    /* collect gps data */
    if (gps_start() != 0)
        exit(EXIT_FAILURE);

    motor_initialize();

    do
    {
        i = 1;

        /* Latitude */
        lat_fifo = fifo_create(0, MA_N);

        if(NULL == lat_fifo)
            break;

        i++;

        lat_filter = filter_create(lat_fifo);

        if(NULL == lat_filter)
            break;

        i++;

        /* Longitude */
        lon_fifo = fifo_create(0, MA_N);

        if(NULL == lon_fifo)
            break;

        i++;

        lon_filter = filter_create(lon_fifo);

        if(NULL == lon_filter)
            break;

        i++;

        lat_kf = kf_create(0.022, 0.617, 0);

        lat_kf->p = 10;
        lat_kf->q = 0.0001;
        lat_kf->r = 0.05;

        if(NULL == lat_kf)
            break;

        i++;

        lon_kf = kf_create(0.022, 0.617, 0);

        lon_kf->p = 10;
        lon_kf->q = 0.0001;
        lon_kf->r = 0.05;

        if(NULL == lon_kf)
            break;

        i++;

        if(0 != pthread_create(&cmd,NULL,(void *) thread_cmd,NULL))
            break;

        i++;

        if(0 != pthread_create(&nav,NULL,(void *) thread_nav,NULL))
            break;

        rc = TRUE;
    }while(0);

    if(rc != TRUE)
    {
        printf("failed(%d)",rc);
        exit(EXIT_SUCCESS);
    }
        
    #ifndef USE_DATA_DUMP
    disp_init();
    #endif

    #ifdef FILTER_PRE_INIT
    for(;;)
    {
        if(get_gps_data(&gps_data_curr) != 0)
            break;
    
        if (gps_data_curr.fix.mode > MODE_NO_FIX)
        {   
            for( i = 0 ; i < MA_N ; i++)
            {
                lat_filter->fifo->data[i] = gps_data_curr.fix.latitude;
                lon_filter->fifo->data[i] = gps_data_curr.fix.longitude;
            }

            lat_kf->x = gps_data_curr.fix.latitude;
            lon_kf->x = gps_data_curr.fix.longitude;
    
            break;
        }

        #ifndef USE_DATA_DUMP
        disp_update(1, 3);
        #endif
             
        usleep(GPSD_SAMPLE_RATE);
    }
    #endif
}

int main(int argc, char *argv[])
{
    setpriority(PRIO_PROCESS, 0, -5);

    /* Pre-Initialization */
    pre_initialize();

    /* Main Loop */
    for(;;)
    {
        if(get_gps_data(&gps_data_curr) != 0)
            break;

        if (gps_data_curr.fix.mode > MODE_NO_FIX)
        {   

            filter_update(lat_filter, gps_data_curr.fix.latitude);
            filter_update(lon_filter, gps_data_curr.fix.longitude);

            kf_update(lat_kf, gps_data_curr.fix.latitude);
            kf_update(lon_kf, gps_data_curr.fix.longitude);

            _distance = distance(lat_kf->x, lon_kf->x, wp[0].lat, wp[0].lon, 'K');

            /* This is a simple approach due to a small distance */
            _direction = atan2((wp[0].lat - lat_kf->x), (wp[0].lon - lon_kf->x));

            #ifdef USE_DATA_DUMP
            printf("%f, %f, %f, %f, %f, %f\r\n", gps_data_curr.fix.latitude,
                                                 lat_filter->mean,
                                                 lat_kf->x,
                                                 gps_data_curr.fix.longitude,
                                                 lon_filter->mean,
                                                 lon_kf->x);
            #endif
        }

        #ifndef USE_DATA_DUMP
        disp_update(1, 3);
        #endif

        usleep(GPSD_SAMPLE_RATE);

    }

    exit(EXIT_SUCCESS);
}

