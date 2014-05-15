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
#include "core.h"

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

//#define PI 3.14159265358979323846

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
}fifo;

#define HANDLE void*

fifo* buffer_create(float data, uint8_t size)
{
    fifo* _fifo = NULL;
    float* _buf = NULL;

    _fifo = calloc(1, sizeof(fifo));

    if(NULL != _fifo)
    {
        _buf = malloc(size * sizeof(float));

        if(NULL != _buf)
        {
            _fifo->data = _buf;
            _fifo->size = size;
            _fifo->index = 0;

            memset(_buf, data, size);
        }
        else
        {
            free(_fifo);
            fprintf(stderr, "buffer = null!\r\n");
            exit(EXIT_SUCCESS);
        }
    }

    return _fifo;
}

void buffer_add(fifo* _fifo, float _data)
{
    _fifo->data[_fifo->index] = _data;

    if(((++(_fifo->index)) % (_fifo->size)) == 0)
        _fifo->index = 0;
}

typedef struct _filter
{
    fifo*   buf;
    float  avg; /* Average */
    float  sd; /* Standard Deviation */
    float  max;
    float  min;
}data_filter;

void filter_update(data_filter* _filter)
{
    float sum;
    uint8_t i;

    /* Calculate average */
    sum = 0;
    for(i = 0 ; i < (_filter->buf->size) ; i++)
    {
        sum = sum + _filter->buf->data[i];
    }

    _filter->avg = sum / (_filter->buf->size);

    /* Calculate standard deviation */
    sum = 0;
    for(i = 0 ; i < (_filter->buf->size) ; i++)
    {
        sum = sum + (_filter->buf->data[i] - _filter->avg)*(_filter->buf->data[i] - _filter->avg);
    }

    /* Find the maximum value of buffer */
    _filter->max = _filter->buf->data[0];
    for(i = 0 ; i < (_filter->buf->size) ; i++)
    {
        _filter->max = max(_filter->max, _filter->buf->data[i]);
    }

    /* Find the  minimum value of buffer */
    _filter->min = _filter->buf->data[0];
    for(i = 0 ; i < (_filter->buf->size) ; i++)
    {
        _filter->min = min(_filter->min, _filter->buf->data[i]);
    }

    _filter->sd = sqrt((sum / (_filter->buf->size)));
}

data_filter* filter_create(fifo* _fifo)
{
    data_filter* _filter;

    _filter = calloc(1, sizeof(data_filter));

    if(NULL != _filter)
    {
        if(NULL != _fifo)
            _filter->buf = _fifo;
        else
        {
            fprintf(stderr, "fifo = null!");
            exit(EXIT_SUCCESS);
        }

        filter_update(_filter);
    }

    return _filter;
}

fifo* lat_buf;
fifo* lon_buf;
data_filter* lat_filter;
data_filter* lon_filter;
float _distance = 0, _direction = 0;
float lat_max, lat_min, lon_max, lon_min;
float sum_lat = 0, buff_lat[MA_N] = {0}, sum_lon = 0, buff_lon[MA_N] = {0};

struct gps_data_t _gps_data;


#define YELLOWONBLUE 1
#define BLACKONWHITE 2
#define WHITEONBLACK 3

#define ATTRIBS  WA_BOLD 
//#define COLORS YELLOWONBLUE
#define COLORS BLACKONWHITE

bool initColors()
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
	
bool setColors(int colorscheme)
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
    #if 0
    int row,col;	/* to store the number of rows and *
					 * the number of colums of the screen */
    #endif

    initscr(); /* start the curses mode */
    initColors();
	attrset(COLOR_PAIR(COLORS) | ATTRIBS);
	clrscr();
    box(stdscr, ACS_VLINE, ACS_HLINE); /*draw a box*/
    curs_set(0); /* Makes the cursor invisible */
    #if 0
    getmaxyx(stdscr,row,col); /* get the number of rows and columns */
    #endif
    start_color(); /* Be sure not to forget this, it will enable colors */
    init_pair(1, COLOR_RED, COLOR_CYAN); /* You can make as much color pairs as you want, be sure to change the ID number */
    init_pair(2, COLOR_YELLOW, COLOR_GREEN);
    attron(A_UNDERLINE | COLOR_PAIR(1)); /* This turns on the underlined text and color pair 1 */
    //printw("This is underlined red text with cyan background\n");
    attroff(A_UNDERLINE | COLOR_PAIR(1));
    attron(A_BOLD | COLOR_PAIR(2));
    //printw("This is bold yellow text with green background");
    attroff(A_BOLD | COLOR_PAIR(2));
    //move(LINES/2, COLS/2); /*move the cursor to the center*/
    refresh();
}

void disp_update(int row, int col)
{
    int row_disp = row;
                     
    erase();
    attrset(COLOR_PAIR(COLORS) | ATTRIBS);
    clrscr();
    box(stdscr, ACS_VLINE, ACS_HLINE); /*draw a box*/

    mvprintw(row_disp++, col, "-------------------------------------------------------------");
    mvprintw(row_disp++, col, "GPS(Global Positioning System) informations that got from");
    mvprintw(row_disp++, col, "GPSd(a GPS service daemon)");
    mvprintw(row_disp++, col, "-------------------------------------------------------------");
    mvprintw(row_disp++, col, "gps_data.status= %d", _gps_data.status);
    mvprintw(row_disp++, col, "gps_data.fix.track= %f", _gps_data.fix.track);
    mvprintw(row_disp++, col, "gps_data.fix.speed= %f", _gps_data.fix.speed);
    mvprintw(row_disp++, col, "gps_data.fix.latitude = %f", _gps_data.fix.latitude);
    mvprintw(row_disp++, col, "gps_data.fix.longitude = %f", _gps_data.fix.longitude);
    mvprintw(row_disp++, col, "lat_max = %f", lat_max);
    //mvprintw((row_disp++, col, "lat_min = %f", lat_min);
    mvprintw(row_disp++, col, "lon_max = %f", lon_max);
    //mvprintw((row_disp++, col, "lon_min = %f", lon_min);
    mvprintw(row_disp++, col, "lat_filter->avg = %f", lat_filter->avg);
    mvprintw(row_disp++, col, "lat_filter->sd = %f", lat_filter->sd);
    mvprintw(row_disp++, col, "lat_filter->max = %f", lat_filter->max);
    mvprintw(row_disp++, col, "lat_filter->min = %f", lat_filter->min);
    mvprintw(row_disp++, col, "lon_filter->avg = %f", lon_filter->avg);
    mvprintw(row_disp++, col, "lon_filter->sd = %f", lon_filter->sd);
    mvprintw(row_disp++, col, "lon_filter->max = %f", lon_filter->max);
    mvprintw(row_disp++, col, "lon_filter->min = %f", lon_filter->min);
    mvprintw(row_disp++, col, "-------------------------------------------------------------");
    mvprintw(row_disp++, col, "local way poipn to the way point in the sports field of NTUT");
    mvprintw(row_disp++, col, "-------------------------------------------------------------");
    mvprintw(row_disp++, col, "distance = %f m", _distance*1000);
    mvprintw(row_disp++, col, "direction = %f deg", _direction*180/M_PI);
    mvprintw(row_disp++, col, "unit(%f,%f)", _gps_data.fix.latitude * norm(_gps_data.fix.latitude, _gps_data.fix.longitude),
                                             _gps_data.fix.longitude * norm(_gps_data.fix.latitude, _gps_data.fix.longitude));
    mvprintw(row_disp++, col, "-------------------------------------------------------------");
    refresh();
}

int main(int argc, char *argv[])
{
    int row = 1, col = 3;

    disp_init();

    /* collect gps data */
    if (gps_start() == 0)
    {
        mvprintw(row++, col, "gpsd connection esthablished, collecting gps data");
    }
    else
    {
        mvprintw(row++, col, "gps data not available");
        exit(EXIT_FAILURE);
    }

    refresh();

    /* Latitude */
    lat_buf = buffer_create(_gps_data.fix.latitude, MA_N);

    if(NULL == lat_buf)
    {
        exit(EXIT_SUCCESS);
    }

    lat_filter = filter_create(lat_buf);

    /* Longitude */
    lon_buf = buffer_create(_gps_data.fix.longitude, MA_N);

    if(NULL == lon_buf)
    {
        exit(EXIT_SUCCESS);
    }

    lon_filter = filter_create(lon_buf);

    for(;;)
    {
        if (get_gps_data(&_gps_data) == 0)
        {
            buffer_add(lat_buf, _gps_data.fix.latitude);
            filter_update(lat_filter);

            buffer_add(lon_buf, _gps_data.fix.longitude);
            filter_update(lon_filter);

            _distance = distance(_gps_data.fix.latitude, _gps_data.fix.longitude, wp[0].lat, wp[0].lon, 'K');

            /* This is a simple approach due to a small distance */
            _direction = atan2((wp[0].lat - _gps_data.fix.latitude), (wp[0].lon - _gps_data.fix.longitude));

            lat_max = max(lat_max, _gps_data.fix.latitude);
            lon_max = max(lon_max, _gps_data.fix.longitude);

            disp_update(row, 3);
        }

        usleep(50000);

    }

    exit(EXIT_SUCCESS);
}

