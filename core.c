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

/* Kalman Filter Settings */
kf_t* lat_kf = NULL;
kf_t* lon_kf = NULL;
kf_t* dist_kf = NULL;
fifo_t* lat_fifo;
fifo_t* lon_fifo;
filter_t* lat_filter;
filter_t* lon_filter;
double _distance = 0, _direction = 0;

char dir = 'B';
int pwm = 0;
int socket_iot = -1;

waypoint wp[] = 
{
    [0] = {
        .name = "wp0",
        .lat =  25.042137, // N
        .lon = 121.537444, // E
    },
    [1] = {
        .name = " ",
        .lat =  25.042137, // N
        .lon = 121.537444, // E
    },
    [2] = {
        .name = " ",
        .lat =  25.042137, // N
        .lon = 121.537444, // E
    },
    [3] = {
        .name = " ",
        .lat =  25.042137, // N
        .lon = 121.537444, // E
    },
    [4] = {
        .name = " ",
        .lat =  25.042137, // N
        .lon = 121.537444, // E
    },
};

struct gps_data_t gps_data_curr;
struct timeval t1, t2;
double elapsedTime;
FILE *fcsv;
FILE *fkml;

char fcsv_buf[256*1024];
char fkml_buf[256*1024];

static const char *pkml_hs =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
        "<kml xmlns=\"http://www.opengis.net/kml/2.2\"\r\n"
        "     xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\r\n"
        "<Folder>\r\n"
        "    <Placemark>\r\n"
        "        <Style>\r\n"
        "            <LineStyle>\r\n"
        "                <color>ff00ff00</color>\r\n"
        "            </LineStyle>\r\n"
        "        </Style>\r\n"
        "        <gx:Track>\r\n";

static const char *pkml_he =
        "        </gx:Track>\r\n"
        "    </Placemark>\r\n"
        "</Folder>\r\n"
        "</kml>";

static const char *pkml_ds = "           <gx:coord>";
static const char *pkml_de = "</gx:coord>";

const char *pDataPoint = "POST /v1.0/device/%s/sensor/%s/datapoints HTTP/1.1\r\n"
        "Host: api.yeelink.net\r\n"
        "Accept: */*\r\n"
        "U-ApiKey: %s\r\n"
        "Content-Length: %d\r\n"
        "Content-type: application/json;charset=utf-8\r\n"
        "Connection: close\r\n\r\n"
        "%s\r\n";

int navi_enabled = FALSE;


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
    double sum = 0, a = 0.688, min_t = 0, max_t = 0; // 50% 0.688
    uint8_t i = 0, i_t = 0;

    f->fifo->data[f->fifo->index] = d;

    if(((++(f->fifo->index)) % (f->fifo->size)) == 0)
        f->fifo->index = 0;

    f->max = f->fifo->data[0];
    f->min = f->fifo->data[0];

    sum = 0;

    for(i = 0 ; i < (f->fifo->size) ; i++)
    {
        /* Summation of all data */
        sum += f->fifo->data[i];

        /* Find the maximum and minimum value of buffer */
        f->max = max(f->max, f->fifo->data[i]);
        f->min = min(f->min, f->fifo->data[i]);
    }

    f->sum = sum;

    /* Calculate moving average */
    f->mean = sum / (f->fifo->size);

    sum = 0;

    for(i = 0 ; i < (f->fifo->size) ; i++)
        sum += pow((f->fifo->data[i] - f->mean), 2);

    //f->var = sum / (f->fifo->size); // sigma ^ 2,  (in population)
    f->var = sum / ((f->fifo->size) - 1 );

    /* Calculate standard deviation */
    f->sd = sqrt(f->var);

    /* Calculate Student-T mean */
    max_t = f->mean + (a * f->sd / sqrt(f->fifo->size));
    min_t = f->mean - (a * f->sd / sqrt(f->fifo->size));

    sum = 0;

    for(i = 0 ; i < (f->fifo->size) ; i++)
    {
        if( (max_t > (f->fifo->data[i])) && (min_t < (f->fifo->data[i])))
        {
            sum += f->fifo->data[i];
            i_t++;
        }
    }

    if(i_t == 0)
        f->mean_t = f->mean;
    else
        f->mean_t = sum / i_t;
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

double calc_distance(double lat1, double lon1, double lat2, double lon2, char unit)
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
    printf("close gps\r\n");
    (void)gps_close(&gps_data_curr);

    /* Stop motor */
    motor_update(M_BRK_H, 0, 0);

    if(fcsv != NULL)
    {
        printf("close file\r\n");
        fflush(fcsv);
        fclose(fcsv);
    }

    if(fkml != NULL)
    {
        fprintf(fkml, pkml_he);
        printf("close file\r\n");
        fflush(fkml);
        fclose(fkml);
    }

    if(socket_iot != 0)
    {
        printf("close socket\r\n");
        socket_close(socket_iot);
    }

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
    //clear();
    //attrset(COLOR_PAIR(COLORS) | ATTRIBS);
    //clrscr();
    //box(stdscr, ACS_VLINE, ACS_HLINE); /*draw a box*/
    #endif

    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "GPS(Global Positioning System) informations that got from    ");
    MSG(row_disp++, col, "GPSd(a GPS service daemon)                                   ");
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "gps_data.status= %d", gps_data_curr.status);
    MSG(row_disp++, col, "gps_data.fix.track= %10f", gps_data_curr.fix.track);
    MSG(row_disp++, col, "gps_data.fix.speed= %10f", gps_data_curr.fix.speed);
    MSG(row_disp++, col, "gps_data.fix.latitude = %10f", gps_data_curr.fix.latitude);
    MSG(row_disp++, col, "gps_data.fix.longitude = %10f", gps_data_curr.fix.longitude);
    MSG(row_disp++, col, "gps_data.dop.hdop = %10f", gps_data_curr.dop.hdop);
    MSG(row_disp++, col, "gps_data.dop.pdop = %10f", gps_data_curr.dop.pdop);
    MSG(row_disp++, col, "gps_data.fix.epx = %10f", gps_data_curr.fix.epx);
    MSG(row_disp++, col, "gps_data.fix.epy = %10f", gps_data_curr.fix.epy);
    MSG(row_disp++, col, "elapsedTime = %10f us", elapsedTime);
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "Filter Informations                                          ");
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, ">> Latitude << :                                                  ");
    MSG(row_disp++, col, "lat_filter->mean = %10f", lat_filter->mean);
    //MSG(row_disp++, col, "lat_filter->mean_t = %10f", lat_filter->mean_t);
    MSG(row_disp++, col, "lat_filter->sd = %10f", lat_filter->sd);
    //MSG(row_disp++, col, "lat_filter->max = %10f", lat_filter->max);
    //MSG(row_disp++, col, "lat_filter->min = %10f", lat_filter->min);
    MSG(row_disp++, col, "Kalman lat_est = %10f", lat_kf->x);
    MSG(row_disp++, col, ">> Longitude << :                                                 ");
    MSG(row_disp++, col, "lon_filter->mean = %10f", lon_filter->mean);
    //MSG(row_disp++, col, "lon_filter->mean_t = %10f", lon_filter->mean_t);
    MSG(row_disp++, col, "lon_filter->sd = %10f", lon_filter->sd);
    //MSG(row_disp++, col, "lon_filter->max = %10f", lon_filter->max);
    //MSG(row_disp++, col, "lon_filter->min = %10f", lon_filter->min);
    MSG(row_disp++, col, "kalman lon_est = %10f", lon_kf->x);
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "local way poipn to the way point in the sports field of NTUT ");
    MSG(row_disp++, col, "-------------------------------------------------------------");
    MSG(row_disp++, col, "distance = %10f m", _distance * KM2M);
    MSG(row_disp++, col, "direction = %10f deg", _direction * 180 / M_PI);
    MSG(row_disp++, col, "motor ctrl = dir(%c), pwm(%03d)", dir, pwm);
    MSG(row_disp++, col, "navi_enabled = %d", navi_enabled);
    MSG(row_disp++, col, "mag_chip_id = 0x%x", get_mag_id());
    MSG(row_disp++, col, "-------------------------------------------------------------");

    #ifdef USE_CURSES
    refresh();
    #endif
}


#define PWM_FC1 200
#define PWM_FC2 200
#define PWM_DL1 50000

void commander(void)
{
    int c = ' ';

    c = getch();

    /* Rover manual direction control */
    switch(c)
    {
        case KEY_UP :

            pwm = PWM_FC1;
            motor_update(M_FWD, pwm, pwm);
            dir = 'U';

            do{
                usleep(PWM_DL1);
            }while(getch() == KEY_UP);

            pwm = 0;
            motor_update(M_BRK_H, pwm, pwm );
            dir = 'B';

        break;

        case KEY_DOWN :

            pwm = PWM_FC1;
            motor_update(M_BWD, pwm, pwm);
            dir = 'D';

            do{
                usleep(PWM_DL1);
            }while(getch() == KEY_DOWN);

            pwm = 0;
            motor_update(M_BRK_H, pwm, pwm );
            dir = 'B';

        break;

        case KEY_LEFT :

            pwm = PWM_FC2;
            motor_update(M_TNL, pwm, pwm);
            dir = 'L';

            do{
                usleep(PWM_DL1);
            }while(getch() == KEY_LEFT);

            pwm = 0;
            motor_update(M_BRK_H, pwm, pwm );
            dir = 'B';

        break;

        case KEY_RIGHT :

            pwm = PWM_FC2;
            motor_update(M_TNR, pwm, pwm);
            dir = 'R';

            do{
                usleep(PWM_DL1);
            }while(getch() == KEY_RIGHT);

            pwm = 0;
            motor_update(M_BRK_H, pwm, pwm );
            dir = 'B';
        
        break;

        case 'S' :
        case 's' :

            navi_enabled = TRUE;

        break;

        case 'D' :
        case 'd' :

            pwm = 0;
            motor_update(M_BRK_H, pwm, pwm );
            dir = 'B';

            navi_enabled = FALSE;

        break;

        default :

        break;
    }
}

void thread_cmd(void)
{
    for(;;)
    {
        commander();
    }
}



void navigation(double lon, double lat, double alt)
{
    static double lon_old, lat_old, distance, heading;
    static int index = 0;
    //unsigned int timeout = (20*60*10);

    lon_old = lon;
    lat_old = lat;

    heading = atan2((lat - lat_old), (lon - lon_old));
    distance = calc_distance(lat, lon, wp[index].lat, wp[index].lon, 'K');

    if(((distance * KM2M) > 2.5) && (index < 1))
    {
        pwm = PWM_FC1;
        motor_update(M_FWD, pwm , pwm );

        #if 0
        if(heading > (_direction + 4))
        {
            motor_update(M_TNR, pwm , pwm );
        }
        else if(heading < (_direction- 4))
        {
            motor_update(M_TNL, pwm , pwm );
        }
        #endif
        dir = 'A';
    }
    else
    {
        motor_update(M_BRK_H, 0, 0 );
        index++;
        dir = 'P';
    }
}

void thread_nav(void)
{

    for(;;)
    {
        if((gps_data_curr.fix.mode > MODE_NO_FIX) && (navi_enabled == TRUE))
        {   
            navigation(gps_data_curr.fix.longitude,
                       gps_data_curr.fix.latitude,
                       gps_data_curr.fix.altitude);

            usleep(GPSD_SAMPLE_RATE);
        }
    }
}

void log_create(void)
{
    fcsv = fopen("gps.csv", "w+");
    //chown("gps.csv",getuid(), getgid());
    chown("gps.csv",RPI_UID, RPI_GID);
    setbuf(fcsv, fcsv_buf);

    fkml = fopen("gps.kml", "w+");
    //chown("gps.kml",getuid(), getgid());
    chown("gps.kml",RPI_UID, RPI_GID);
    setbuf(fkml, fkml_buf);

    fprintf(fkml, pkml_hs);
}

void csvlog(double lon, double lat, double alt)
{
    fprintf(fcsv,"%10f, %10f, %10f, %10f, %10f, %10f, %f, %f\r\n",
            lon,
            lat,
            lon_filter->mean,
            lat_filter->mean,
            lon_kf->x,
            lat_kf->x,
            _distance *KM2M,
            dist_kf->x *KM2M);
}

void kmllog(double lon, double lat, double alt)
{
    fprintf(fkml, pkml_ds);
    fprintf(fkml, "%.6f, %.6f, %.6f", lon, lat, alt);
    fprintf(fkml, pkml_de);
    fprintf(fkml, "\r\n");
}

void thread_log(void)
{
}

const char* send_data_to_yeelink(const char *device,
        const char *sensor,
        const char *key,
        time_t t,
        float v)
{
    static char sbuf[512];
    char json[64];
    int jsonlen = 0;
    #if 0
    struct tm curtm;
    t = (t == 0) ? time(NULL) : t;

    (void)localtime_r(&t, &curtm);
    (void)sprintf(json, "{\"timestamp\":\"%04d-%02d-%02dT%02d:%02d:%02d\","
            "\"value\":%.2f}",
            1900 + curtm.tm_year, curtm.tm_mon + 1, curtm.tm_mday, curtm.tm_hour, curtm.tm_min, curtm.tm_sec,
            v + 0.005f);
    #else
    (void)sprintf(json, "{\"value\":%f}", v);
    #endif
    jsonlen = strlen(json);

    sprintf(sbuf, pDataPoint, device, sensor, key, jsonlen, json);
    return sbuf;
}

/* Xively is the Public Cloud specifically built for the Internet of Things */
void iot_update(void)
{
    static int temp = 0;
    xi_datapoint_t datapoint;

    /* create the xi library context */
    xi_context_t* xi_context
        = xi_create_context(XI_HTTP, XI_API_KEY, XI_FEED_ID );

    xi_set_value_i32( &datapoint, temp++);

    {
        /*  get actual timestamp */
        time_t timer = 0;
        time( &timer );
        datapoint.timestamp.timestamp = timer;
    }

    xi_datastream_update( xi_context, XI_FEED_ID, XI_STREAM_ID_1, &datapoint );

    /* destroy the context cause we don't need it anymore */
    xi_delete_context( xi_context );
}

void thread_iot(void)
{
    for(;;)
    {
        iot_update();
        sleep(1);
    }
}


void pre_initialize(void)
{
    int i = 0, rc = FALSE;
    pthread_t cmd, iot, nav;

    (void)signal(SIGINT, signal_handler);

    /* collect gps data */
    if (gps_start() != 0)
        exit(EXIT_FAILURE);

    motor_initialize();
    imu_initialize();

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

        if(NULL == lat_kf)
            break;

        i++;

        lon_kf = kf_create(0.022, 0.617, 0);

        if(NULL == lon_kf)
            break;

        i++;

        dist_kf = kf_create(0.00001, 0.999, 0.5);

        if(NULL == lon_kf)
            break;

        i++;

        if(0 != pthread_create(&cmd,NULL,(void *) thread_cmd,NULL))
            break;

        i++;

        if(0 != pthread_create(&iot,NULL,(void *) thread_iot,NULL))
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
        
    disp_init();

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

        disp_update(1, 3);

        usleep(GPSD_SAMPLE_RATE);
    }
    #endif
}

int main(int argc, char *argv[])
{
    double lon, lat, alt;
    setpriority(PRIO_PROCESS, 0, -5);

    log_create();

    /* Pre-Initialization */
    pre_initialize();

    /* Main Loop */
    for(;;)
    {
        if(get_gps_data(&gps_data_curr) != 0)
            break;

        if (gps_data_curr.fix.mode > MODE_NO_FIX)
        {
            lon = gps_data_curr.fix.longitude;
            lat = gps_data_curr.fix.latitude;
            alt = gps_data_curr.fix.altitude;

            /* start timer */
            gettimeofday(&t1, NULL);

            #if 0
            filter_update(lat_filter, gps_data_curr.fix.latitude);
            filter_update(lon_filter, gps_data_curr.fix.longitude);

            if(!isnan(gps_data_curr.fix.epy))
                lat_kf->r = powf(gps_data_curr.fix.epy, 2);

            kf_update(lat_kf, gps_data_curr.fix.latitude);

            if(!isnan(gps_data_curr.fix.epx))
                lon_kf->r = powf(gps_data_curr.fix.epx, 2);

            kf_update(lon_kf, gps_data_curr.fix.longitude);
            #endif

            _distance = calc_distance(lat, lon, wp[0].lat, wp[0].lon, 'K');

            kf_update(dist_kf, _distance);

            /* This is a simple approach due to a small distance */
            _direction = atan2((wp[0].lat - lat), (wp[0].lon - lon));

            /* stop timer */
            gettimeofday(&t2, NULL);

            /* compute and print the elapsed time in millisec */
            elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
            elapsedTime += (t2.tv_usec - t1.tv_usec);   // ms to us
            //elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

            csvlog(lon, lat, alt);
            kmllog(lon, lat, alt);
            //navigation(lon, lat, alt);

            disp_update(1, 3);
        }

        usleep(GPSD_SAMPLE_RATE);

    }

    exit(EXIT_SUCCESS);
}

