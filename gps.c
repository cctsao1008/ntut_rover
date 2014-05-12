/*********************************************************************
 *
 *   File : 
 *       gps.c
 *
 *   Date : 
 *       11/05/2014
 *
 *   Contributors : 
 *       TSAO, CHIA-CHENG <chiacheng.tsao@gmail.com> , From NTUT
 *
 *********************************************************************/
#include "gps.h"

struct gps_data_t gpsdata;
char    gps_available = 0;

struct fixsource_t
/* describe a data source */
{
    char *spec;     /* pointer to actual storage */
    char *server;
    char *port;
    char *device;
};

/* pseudo-signals indicating reason for termination */
#define CGPS_QUIT   0   /* voluntary yterminastion */
#define GPS_GONE    -1  /* GPS device went away */
#define GPS_ERROR   -2  /* low-level failure in GPS read */
#define GPS_TIMEOUT -3  /* low-level failure in GPS waiting */

/* Function to call when we're all done.  Does a bit of clean-up. */
static void die(int sig)
{
    #if 0
    if (!isendwin())
    {
    /* Move the cursor to the bottom left corner. */
    (void)mvcur(0, COLS - 1, LINES - 1, 0);

    /* Put input attributes back the way they were. */
    (void)echo();

    /* Done with curses. */
    (void)endwin();
    }
    #endif

    /* We're done talking to gpsd. */
    (void)gps_close(&gpsdata);

    switch (sig) {
    case CGPS_QUIT:
    break;
    case GPS_GONE:
    (void)fprintf(stderr, "cgps: GPS hung up.\n");
    break;
    case GPS_ERROR:
    (void)fprintf(stderr, "cgps: GPS read returned error\n");
    break;
    case GPS_TIMEOUT:
    (void)fprintf(stderr, "cgps: GPS timeout\n");
    break;
    default:
    (void)fprintf(stderr, "cgps: caught signal %d\n", sig);
    break;
    }

    /* Bye! */
    exit(EXIT_SUCCESS);
}

void gpsd_source_spec(const char *arg, struct fixsource_t *source)
/* standard parsing of a GPS data source spec */
{
    /* the casts attempt to head off a -Wwrite-strings warning */
    source->server = (char *)"localhost";
    source->port = (char *)DEFAULT_GPSD_PORT;
    source->device = NULL;

    if (arg != NULL) {
    char *colon1, *skipto, *rbrk;
    source->spec = strdup(arg);
    assert(source->spec != NULL);

    skipto = source->spec;
    if (*skipto == '[' && (rbrk = strchr(skipto, ']')) != NULL) {
        skipto = rbrk;
    }
    colon1 = strchr(skipto, ':');

    if (colon1 != NULL) {
        char *colon2;
        *colon1 = '\0';
        if (colon1 != source->spec) {
        source->server = source->spec;
        }
        source->port = colon1 + 1;
        colon2 = strchr(source->port, ':');
        if (colon2 != NULL) {
        *colon2 = '\0';
        source->device = colon2 + 1;
        }
    } else if (strchr(source->spec, '/') != NULL) {
        source->device = source->spec;
    } else {
        source->server = source->spec;
    }
    }

    if (*source->server == '[') {
    char *rbrk = strchr(source->server, ']');
    ++source->server;
    if (rbrk != NULL)
        *rbrk = '\0';
    }
}

void
gps_stop(void) {

    if (!gps_available)
        return;

    gps_stream((struct gps_data_t *) &gpsdata, WATCH_DISABLE, NULL);
    gps_close((struct gps_data_t *) &gpsdata);

    gps_available = 0;
}


int
gps_start(void) {
    struct fixsource_t source;
    unsigned int flags;

    flags = WATCH_ENABLE;

    gpsd_source_spec(NULL, &source);

    if (gps_open(source.server, source.port, &gpsdata) != 0) {
    (void)fprintf(stderr,
              "gpspipe: could not connect to gpsd %s:%s, %s(%d)\n",
              source.server, source.port, strerror(errno), errno);

        return -1;
    }

    if (source.device != NULL)
    flags |= WATCH_DEVICE;

    gps_stream(&gpsdata, flags, source.device);

    gps_available = 1;

    return 0;
}

void
gps_reconnect(void) {
    fprintf(stderr, "reconnecting to gpsd...\n");
    gps_stop();
    gps_start();
}

int
//get_gps_data(struct gps_fix_t *g) {
get_gps_data(struct gps_data_t *gd) {

    if (!gps_available)
        return -1;

    #if 0
    /* heart of the client */
    if (!gps_waiting(&gpsdata, 5000000)) {
        fprintf(stderr, "no gps data available (timeout)\n");
        gps_reconnect();
        return -1;
    }

    errno = 0;
    if (gps_read(&gpsdata) == -1) {
        perror("gps_read()");
        die(errno == 0 ? GPS_GONE : GPS_ERROR);
        gps_reconnect();
        return -1;
    }
    #endif

	if (!gps_waiting(&gpsdata, 5000000)) {
	    die(GPS_TIMEOUT);
	} else {
	    errno = 0;
	    if (gps_read(&gpsdata) == -1) {
		fprintf(stderr, "cgps: socket error 4\n");
		die(errno == 0 ? GPS_GONE : GPS_ERROR);
	    }
	}

    //memcpy(g, &gpsdata.fix, sizeof(struct gps_fix_t));
    memcpy(gd, &gpsdata, sizeof(struct gps_data_t));

    // convert speed to km/h
    gd->fix.speed = gd->fix.speed * 3.6;
    gd->fix.eps = gd->fix.eps * 3.6;

    return 0;
}
